#include "SoapyPerseus.hpp"
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Modules.hpp>
#include <perseus-sdr.h>
#include <cstdio>
#include <string>
#include <mutex>

// Function called by SoapySDR to list connected Perseus devices
std::vector<SoapySDR::Kwargs> findPerseus(const SoapySDR::Kwargs &args)
{
    static std::mutex scanMutex;
    static std::vector<SoapySDR::Kwargs> cachedResults;

    std::lock_guard<std::mutex> lock(scanMutex);

    // CRITICAL FIX: Never call perseus_init() or walk the USB bus while a receiver is open
    if (SoapyPerseus::s_openDeviceCount.load(std::memory_order_acquire) > 0) {
        if (!cachedResults.empty()) {
            return cachedResults;
        }
        if (args.count("driver") != 0 && args.at("driver") == "perseus") {
            SoapySDR::Kwargs devArgs;
            devArgs["driver"] = "perseus";
            return {devArgs};
        }
        return {};
    }

    std::vector<SoapySDR::Kwargs> results;

    // Initialize the native library (returns the number of hardware devices found)
    int devCount = perseus_init();
    if (devCount < 0) {
        return results;
    }

    // Inspect each device found on the USB bus
    for (int i = 0; i < devCount; i++) {
        perseus_descr* dev = perseus_open(i);
        if (!dev) continue;

        // Firmware download with safety guard-rail against broken pipe or unstable USB power
        int fw_err = perseus_firmware_download(dev, NULL);
        if (fw_err != 0) {
            perseus_close(dev);
            continue;
        }

        // Securely retrieve production information from the internal EEPROM
        eeprom_prodid prodId;
        if (perseus_get_product_id(dev, &prodId) == 0) {
            char serial_buf[128] = {0};
            char label_buf[128] = {0};

            // Use snprintf to strictly prevent stack buffer overflows
            std::snprintf(serial_buf, sizeof(serial_buf), "%05d", (int)prodId.sn);
            std::snprintf(label_buf, sizeof(label_buf), "Perseus %d.%d [%s]",
                          (int)prodId.hwver, (int)prodId.hwrel, serial_buf);

            SoapySDR::Kwargs devArgs;
            devArgs["driver"] = "perseus";
            devArgs["label"] = std::string(label_buf);
            devArgs["device_index"] = std::to_string(i);
            devArgs["serial"] = std::string(serial_buf);

            results.push_back(devArgs);
        }

        // Close the device descriptor after the discovery phase
        perseus_close(dev);
    }

    // Always clean up global resources after scanning
    perseus_exit();

    // Update the cache with the fresh scan
    cachedResults = results;

    // Fallback if the user explicitly requests the "perseus" driver
    if (results.empty() && args.count("driver") != 0 && args.at("driver") == "perseus") {
        SoapySDR::Kwargs devArgs;
        devArgs["driver"] = "perseus";
        results.push_back(devArgs);
    }

    return results;
}

// Function called by SoapySDR to instantiate the device for active use
SoapySDR::Device *makePerseus(const SoapySDR::Kwargs &args)
{
    return new SoapyPerseus(args);
}

// Official factory registration within the SoapySDR framework
static SoapySDR::Registry registerPerseus("perseus", &findPerseus, &makePerseus, SOAPY_SDR_ABI_VERSION);
