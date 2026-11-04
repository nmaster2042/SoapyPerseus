#include "SoapyPerseus.hpp"
#include <SoapySDR/Logger.hpp>
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <stdexcept>

// Microtelecom 24-bit packed memory mapper layout from perseus_test.c
typedef union {
    struct {
        int32_t i;
        int32_t q;
    } __attribute__((__packed__)) iq;
    struct {
        uint8_t i1; uint8_t i2; uint8_t i3; uint8_t i4;
        uint8_t q1; uint8_t q2; uint8_t q3; uint8_t q4;
    } __attribute__((__packed__));
} iq_sample;

SoapyPerseus::SoapyPerseus(const SoapySDR::Kwargs &args) :
_dev(nullptr), _deviceIndex(0), _frequency(14000000.0), _sampleRate(1000000.0), _gain(0.0),
_dithering(false), _preamp(false), _preselector(true), _streaming(false),
_head(0), _tail(0), _bufferCount(0), _callbackActiveCount(0)
{
    if (args.count("device_index") != 0) {
        _deviceIndex = std::stoi(args.at("device_index"));
    }
    if (args.count("serial") != 0) {
        _serial = args.at("serial");
    }

    perseus_exit();
    perseus_init();

    _dev = perseus_open(_deviceIndex);
    if (!_dev) {
        throw std::runtime_error("Failed to open physical Perseus SDR hardware device");
    }

    int err = perseus_firmware_download(_dev, NULL);
    if (err) {
        perseus_close(_dev);
        throw std::runtime_error("Critical failure during Perseus firmware upload sequence");
    }

    _circularBuffer.resize(2 * 1024 * 1024);
    perseus_set_sampling_rate(_dev, static_cast<unsigned int>(_sampleRate));
    perseus_set_adc(_dev, _dithering ? 1 : 0, _preamp ? 1 : 0);
}

SoapyPerseus::~SoapyPerseus()
{
    _streaming = false;
    if (_dev) {
        perseus_stop_async_input(_dev);

        // Wait safely until the native background threads have fully exited
        while (_callbackActiveCount.load(std::memory_order_acquire) > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        perseus_close(_dev);
    }
    perseus_exit();
}

size_t SoapyPerseus::getNumChannels(const int direction) const
{
    return (direction == SOAPY_SDR_RX) ? 1 : 0;
}

SoapySDR::Kwargs SoapyPerseus::getHardwareInfo(void) const
{
    SoapySDR::Kwargs info;
    info["serial"] = _serial;
    return info;
}

// === FREQUENCY MANAGEMENT ===
void SoapyPerseus::setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs &args)
{
    // Forward the global call straight to our named component handler
    SoapyPerseus::setFrequency(direction, channel, "RF", frequency, args);
}

void SoapyPerseus::setFrequency(const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args)
{
    if (direction != SOAPY_SDR_RX) return;
    (void)channel; (void)name; (void)args;

    _frequency = frequency;

    if (_dev) {
#ifdef ENABLE_LOGS
        SoapySDR::logf(SOAPY_SDR_INFO, "[SoapyPerseus] Tuning hardware oscillator to: %.3f MHz (Preselector: %s)",
                       _frequency / 1e6, _preselector ? "ON" : "OFF");
#endif
        perseus_set_ddc_center_freq(_dev, _frequency, _preselector ? 1 : 0);
    }
}

double SoapyPerseus::getFrequency(const int direction, const size_t channel) const
{
    return SoapyPerseus::getFrequency(direction, channel, "RF");
}

double SoapyPerseus::getFrequency(const int direction, const size_t channel, const std::string &name) const
{
    (void)direction; (void)channel; (void)name;
    return _frequency;
}

SoapySDR::RangeList SoapyPerseus::getFrequencyRange(const int direction, const size_t channel) const
{
    return SoapyPerseus::getFrequencyRange(direction, channel, "RF");
}

SoapySDR::RangeList SoapyPerseus::getFrequencyRange(const int direction, const size_t channel, const std::string &name) const
{
    (void)direction; (void)channel; (void)name;
    SoapySDR::RangeList ranges;
    // Déclare la couverture HF complète du Perseus (10 kHz à 30 MHz) pour le composant "RF"
    ranges.push_back(SoapySDR::Range(10000.0, 30000000.0));
    return ranges;
}

void SoapyPerseus::setSampleRate(const int direction, const size_t channel, const double rate)
{
    if (direction != SOAPY_SDR_RX) return;
    (void)channel;
    _sampleRate = rate;
    if (_dev) {
        perseus_set_sampling_rate(_dev, static_cast<unsigned int>(_sampleRate));
    }
}

double SoapyPerseus::getSampleRate(const int direction, const size_t channel) const
{
    (void)direction; (void)channel;
    return _sampleRate;
}

std::vector<double> SoapyPerseus::listSampleRates(const int direction, const size_t channel) const
{
    (void)direction; (void)channel;
    return {125000.0, 250000.0, 500000.0, 1000000.0, 2000000.0};
}


// === GAIN / ATTENUATOR MANAGEMENT ===
std::vector<std::string> SoapyPerseus::listGains(const int direction, const size_t channel) const
{
    (void)direction; (void)channel;
    // Expose the official explicit named component required by SDR++ and GQRX
    return {"ATTENUATION"};
}

void SoapyPerseus::setGain(const int direction, const size_t channel, const double value)
{
    // Forward the global gain slider call straight to our named component handler
    SoapyPerseus::setGain(direction, channel, "ATTENUATION", value);
}

void SoapyPerseus::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{
    if (direction != SOAPY_SDR_RX) return;
    (void)channel; (void)name;
    _gain = value;

    if (_dev) {
        // Secure quantization: clamp and round to the nearest hardware filter boundary
        int rounded_gain = static_cast<int>(std::round(_gain));
        int atten_idx = 0;
        if (rounded_gain >= 30)      atten_idx = 3;
        else if (rounded_gain >= 20) atten_idx = 2;
        else if (rounded_gain >= 10) atten_idx = 1;
        else                         atten_idx = 0;

#ifdef ENABLE_LOGS
        // Official SoapySDR Logger output for hardware monitoring
        SoapySDR::logf(SOAPY_SDR_INFO, "[SoapyPerseus] Attenuator slider moved: %d dB requested -> Choosing hardware index: %d", rounded_gain, atten_idx);
#endif
        perseus_set_attenuator_n(_dev, atten_idx);
    }
}

double SoapyPerseus::getGain(const int direction, const size_t channel) const
{
    return SoapyPerseus::getGain(direction, channel, "ATTENUATION");
}

double SoapyPerseus::getGain(const int direction, const size_t channel, const std::string &name) const
{
    (void)direction; (void)channel; (void)name;
    return _gain;
}

SoapySDR::Range SoapyPerseus::getGainRange(const int direction, const size_t channel) const
{
    return SoapyPerseus::getGainRange(direction, channel, "ATTENUATION");
}

SoapySDR::Range SoapyPerseus::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
    (void)direction; (void)channel; (void)name;
    // Strictly map discrete 10dB boundaries: min 0, max 30, step 10
    return SoapySDR::Range(0.0, 30.0, 10.0);
}

// === CUSTOM HARDWARE SETTINGS ===
SoapySDR::ArgInfoList SoapyPerseus::getSettingInfo(void) const
{
    SoapySDR::ArgInfoList settings;

    SoapySDR::ArgInfo ditherArg;
    ditherArg.key = "dither"; ditherArg.value = "false"; ditherArg.name = "ADC Dithering";
    ditherArg.description = "Enable ADC dither to reduce low-level spurious signals.";
    ditherArg.type = SoapySDR::ArgInfo::BOOL;
    settings.push_back(ditherArg);

    SoapySDR::ArgInfo preampArg;
    preampArg.key = "preamp"; preampArg.value = "false"; preampArg.name = "ADC Preamp";
    preampArg.description = "Modify ADC input front-end amplifier sensitivity threshold.";
    preampArg.type = SoapySDR::ArgInfo::BOOL;
    settings.push_back(preampArg);

    // FIX: Expose the Wideband Mode toggle switch to SDR++ / GQRX interface
    SoapySDR::ArgInfo wbArg;
    wbArg.key = "wideband";
    wbArg.value = _preselector ? "false" : "true";
    wbArg.name = "Wideband Mode (Bypass Filters)";
    wbArg.description = "Bypass the narrow analog preselector filters (WB LED ON).";
    wbArg.type = SoapySDR::ArgInfo::BOOL;
    settings.push_back(wbArg);

    return settings;
}

void SoapyPerseus::writeSetting(const std::string &key, const std::string &value)
{
    bool adc_changed = false;
    bool freq_changed = false;

    if (key == "dither") { _dithering = (value == "true"); adc_changed = true; }
    else if (key == "preamp") { _preamp = (value == "true"); adc_changed = true; }
    else if (key == "wideband") {
        _preselector = (value != "true");
        freq_changed = true;
    }

    if (_dev) {
        if (adc_changed) {
#ifdef ENABLE_LOGS
            // Official SoapySDR Logger output for hardware monitoring
            SoapySDR::logf(SOAPY_SDR_INFO, "[SoapyPerseus] Setting hardware ADC configuration -> Dither: %s, Preamp: %s",
                           _dithering ? "ON" : "OFF", _preamp ? "ON" : "OFF");
#endif
            perseus_set_adc(_dev, _dithering ? 1 : 0, _preamp ? 1 : 0);
        }
        if (freq_changed) {
#ifdef ENABLE_LOGS
            SoapySDR::logf(SOAPY_SDR_INFO, "[SoapyPerseus] Setting hardware preselector state -> Wideband Mode: %s",
                           !_preselector ? "ON (Bypass)" : "OFF (Filter active)");
#endif
            perseus_set_ddc_center_freq(_dev, _frequency, _preselector ? 1 : 0);
        }
    }
}

std::string SoapyPerseus::readSetting(const std::string &key) const
{
    if (key == "dither") return _dithering ? "true" : "false";
    if (key == "preamp") return _preamp ? "true" : "false";
    if (key == "wideband") return !_preselector ? "true" : "false"; // Return true if wideband is active
    return "";
}

// === STREAMING ENGINE ===
std::vector<std::string> SoapyPerseus::getStreamFormats(const int direction, const size_t channel) const
{
    (void)direction; (void)channel;
    return {SOAPY_SDR_CF32};
}

std::string SoapyPerseus::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const
{
    (void)direction; (void)channel;
    fullScale = 32768.0;
    return SOAPY_SDR_CF32;
}

SoapySDR::Stream *SoapyPerseus::setupStream(const int direction, const std::string &format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args)
{
    (void)channels; (void)args;
    if (direction != SOAPY_SDR_RX || format != SOAPY_SDR_CF32) {
        throw std::runtime_error("Unsupported stream direction or format");
    }
    return reinterpret_cast<SoapySDR::Stream *>(this);
}

void SoapyPerseus::closeStream(SoapySDR::Stream *stream) { (void)stream; }

int SoapyPerseus::activateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs, const size_t numElems)
{
    (void)stream; (void)flags; (void)timeNs; (void)numElems;

    std::lock_guard<std::mutex> lock(_bufferMutex);
    _head = 0; _tail = 0; _bufferCount = 0; _streaming = true;

    perseus_set_sampling_rate(_dev, static_cast<unsigned int>(_sampleRate));
    perseus_set_ddc_center_freq(_dev, _frequency, _preselector ? 1 : 0);

    // Initializing the async input with standard factory multi-block parameters (6 * 1024)
    int err = perseus_start_async_input(_dev, 6 * 1024, &SoapyPerseus::perseusCallback, this);
    if (err) { _streaming = false; return SOAPY_SDR_NOT_SUPPORTED; }
    return 0;
}

int SoapyPerseus::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs)
{
    (void)stream; (void)flags; (void)timeNs;
    _streaming = false;
    if (_dev) {
        perseus_stop_async_input(_dev);
    }
    _bufferCv.notify_all();
    return 0;
}

int SoapyPerseus::perseusCallback(void *raw_samples, int sample_count, void *ctx)
{
    if (!ctx) return 0;
    SoapyPerseus *_this = reinterpret_cast<SoapyPerseus *>(ctx);

    _this->_callbackActiveCount.fetch_add(1, std::memory_order_release);

    if (_this->_streaming.load(std::memory_order_acquire)) {
        _this->handleSamples(reinterpret_cast<uint8_t *>(raw_samples), sample_count);
    }

    _this->_callbackActiveCount.fetch_sub(1, std::memory_order_release);
    return 0;
}

void SoapyPerseus::handleSamples(uint8_t *samples, int sample_count)
{
    std::lock_guard<std::mutex> lock(_bufferMutex);
    if (!_streaming) return;

    int nSamples = sample_count / 6; // 6 raw bytes per full complex IQ sample
    size_t floatElementsToWrite = nSamples * 2;

    size_t currentHead = _head.load(std::memory_order_relaxed);
    size_t currentCount = _bufferCount.load(std::memory_order_relaxed);

    if (currentCount + floatElementsToWrite > _circularBuffer.size()) {
        return; // Guard overflow
    }

    iq_sample s;
    s.i1 = s.q1 = 0;

    for (int k = 0; k < nSamples; k++) {
        s.i2 = *samples++; s.i3 = *samples++; s.i4 = *samples++;
        s.q2 = *samples++; s.q3 = *samples++; s.q4 = *samples++;

        // Convert raw bytes directly to complex floats using official Microtelecom scalars
        _circularBuffer[currentHead] = (float)(s.iq.i) / (INT_MAX - 256);
        currentHead = (currentHead + 1) % _circularBuffer.size();

        _circularBuffer[currentHead] = (float)(s.iq.q) / (INT_MAX - 256);
        currentHead = (currentHead + 1) % _circularBuffer.size();
    }

    _head.store(currentHead, std::memory_order_release);
    _bufferCount.fetch_add(floatElementsToWrite, std::memory_order_release);
    _bufferCv.notify_one();
}

int SoapyPerseus::readStream(SoapySDR::Stream *stream, void * const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs)
{
    (void)stream; (void)timeNs;

    float *outBuffer = (float *)(buffs[0]);
    size_t requiredElements = numElems * 2;

    std::unique_lock<std::mutex> lock(_bufferMutex);

    if (_bufferCount.load(std::memory_order_acquire) < requiredElements) {
        _bufferCv.wait_for(lock, std::chrono::microseconds(timeoutUs), [this, requiredElements] {
            return (_bufferCount.load(std::memory_order_relaxed) >= requiredElements) || !_streaming;
        });
    }

    if (!_streaming) return SOAPY_SDR_NOT_SUPPORTED;
    if (_bufferCount.load(std::memory_order_relaxed) < requiredElements) return SOAPY_SDR_TIMEOUT;

    size_t currentTail = _tail.load(std::memory_order_relaxed);

    // Extraction ultra-rapide des floats pré-calculés vers le buffer DSP
    for (size_t i = 0; i < requiredElements; ++i) {
        outBuffer[i] = _circularBuffer[currentTail];
        currentTail = (currentTail + 1) % _circularBuffer.size();
    }

    _tail.store(currentTail, std::memory_order_release);
    _bufferCount.fetch_sub(requiredElements, std::memory_order_release);

    flags = 0;
    return numElems;
}


















