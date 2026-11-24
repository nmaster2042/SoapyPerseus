#pragma once

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <perseus-sdr.h>
#include <vector>
#include <string>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <climits>

class SoapyPerseus : public SoapySDR::Device
{
public:
    SoapyPerseus(const SoapySDR::Kwargs &args);
    ~SoapyPerseus() override;

    static std::atomic<int> s_openDeviceCount;

    // Identification & Channels API
    std::string getDriverKey(void) const override { return "perseus"; }
    std::string getHardwareKey(void) const override { return "PerseusSDR"; }
    size_t getNumChannels(const int direction) const override;
    SoapySDR::Kwargs getHardwareInfo(void) const override;

    // Frequency API (Overridden with Named Components)
    void setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs &args = SoapySDR::Kwargs()) override;
    void setFrequency(const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args = SoapySDR::Kwargs()) override;
    double getFrequency(const int direction, const size_t channel) const override;
    double getFrequency(const int direction, const size_t channel, const std::string &name) const override;
    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel) const override;
    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel, const std::string &name) const override;

    // Antenna API (For GQRX Wideband Mode mapping)
    std::vector<std::string> listAntennas(const int direction, const size_t channel) const override;
    void setAntenna(const int direction, const size_t channel, const std::string &name) override;
    std::string getAntenna(const int direction, const size_t channel) const override;

    // Sample Rate API
    void setSampleRate(const int direction, const size_t channel, const double rate) override;
    double getSampleRate(const int direction, const size_t channel) const override;
    std::vector<double> listSampleRates(const int direction, const size_t channel) const override;

    // Gain & Attenuator API (Overridden with Named Components for SDR++)
    std::vector<std::string> listGains(const int direction, const size_t channel) const override;
    void setGain(const int direction, const size_t channel, const double value) override;
    void setGain(const int direction, const size_t channel, const std::string &name, const double value) override;
    double getGain(const int direction, const size_t channel) const override;
    double getGain(const int direction, const size_t channel, const std::string &name) const override;
    SoapySDR::Range getGainRange(const int direction, const size_t channel) const override;
    SoapySDR::Range getGainRange(const int direction, const size_t channel, const std::string &name) const override;

    // Custom Hardware Settings API
    SoapySDR::ArgInfoList getSettingInfo(void) const override;
    void writeSetting(const std::string &key, const std::string &value) override;
    std::string readSetting(const std::string &key) const override;

    // Streaming API
    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const override;
    std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const override;
    SoapySDR::Stream *setupStream(const int direction, const std::string &format, const std::vector<size_t> &channels = std::vector<size_t>(), const SoapySDR::Kwargs &args = SoapySDR::Kwargs()) override;
    void closeStream(SoapySDR::Stream *stream) override;
    int activateStream(SoapySDR::Stream *stream, const int flags = 0, const long long timeNs = 0, const size_t numElems = 0) override;
    int deactivateStream(SoapySDR::Stream *stream, const int flags = 0, const long long timeNs = 0) override;
    int readStream(SoapySDR::Stream *stream, void * const *buffs, const size_t numElems, int &flags, long long &timeNs, const long timeoutUs = 100000) override;

private:
    perseus_descr* _dev;
    int _deviceIndex;
    std::string _serial;
    double _frequency;
    double _sampleRate;
    double _gain;
    std::string _streamFormat;

    bool _dithering;
    bool _preamp;
    bool _preselector;
    bool _gqrxMode;
    std::atomic<bool> _streaming;

    // Ordered atomic streaming variables
    std::vector<float> _circularBuffer;
    std::atomic<size_t> _head;
    std::atomic<size_t> _tail;
    std::atomic<size_t> _bufferCount;
    std::atomic<int> _callbackActiveCount;

    std::mutex _bufferMutex;
    std::condition_variable _bufferCv;

    static int perseusCallback(void *raw_samples, int sample_count, void *ctx);
    void handleSamples(uint8_t *samples, int sample_count);
};




