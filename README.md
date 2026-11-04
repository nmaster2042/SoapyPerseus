# SoapyPerseus

SoapySDR plugin for the **Microtelecom Perseus SDR** receiver. This driver provides a modern, robust, and lock-free implementation to seamlessly integrate the Perseus HF receiver into standard open-source applications such as **SDR++**, **GQRX**, and **CubicSDR**.

## Features

- **High-Speed CF32 Streaming**: Delivers a rock-solid 1 MSps (and up to 2 MSps) complex float stream (~7.8 MB/s) with zero sample drops or overflows.
- **Native 24-bit Unpacking**: Implements Microtelecom's authentic hardware unpacking math directly inside the USB callback context.
- **Lock-Free Multithreading Architecture**: Eliminates mutual exclusions and condition variable stalls to prevent runtime CPU bottlenecks and immediate `input transfer overflow` errors.
- **Hardware Preselector Control**: Automatically switches physical analog bandpass filters when tuning across the HF spectrum.
- **Dedicated Wideband (WB) Mode Toggle**: Allows bypassing the narrow front-end filters and commanding the mechanical relay and front panel LED directly from the software interface.
- **Component Gain Mapping**: Fully supports the named `"ATTENUATION"` slider component (`0, 10, 20, 30 dB`) to click the hardware attenuation relays and light up the front panel LEDs.
- **Advanced ADC Settings**: Exposes `ADC Dithering` and `ADC Preamp` options directly through the Custom Hardware Settings panel.
- **Safe Lifecycle Management**: Implements atomic reference safety fences to guarantee quiet and crash-free shutdown sequences (`Ctrl+C` or application closures).

## Prerequisites

Before building this module, make sure you have installed the native Microtelecom controller library (`libperseus-sdr`) and the SoapySDR development framework on your Linux system.

### On Ubuntu/Debian:
```bash
sudo apt update
sudo apt install build-essential cmake git libusb-1.0-0-dev
sudo apt install soapysdr-tools libsoapysdr-dev
```

Ensure that your `libperseus-sdr` header (`perseus-sdr.h`) is accessible in your system include paths and that user permission rules (`udev`) are properly set up for the device.

## Building and Installation

### Standard Build (Production / Silent Mode)
By default, the driver compiles in high-performance mode with all console logs disabled to minimize CPU overhead in fast processing loops:

```bash
cd SoapyPerseus
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j\$(nproc)
sudo make install
sudo ldconfig
```

### Debug Build (With Console Logging Enabled)
If you wish to monitor hardware actions in real-time (attenuator steps, frequency tuning, ADC adjustments), you can explicitly inject the logging macro during the CMake generation step:

```bash
cd SoapyPerseus
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-DENABLE_LOGS" ..
make -j\$(nproc)
sudo make install
sudo ldconfig
```
When compiled with this flag, launching applications like SDR++ from a terminal will instantly display live hardware telemetry events.

## Verification

You can quickly verify that the driver is compiled, registered, and communicating with your hardware using the SoapySDR system utilities:

### 1. Check driver presence:
```bash
SoapySDRUtil --find="driver=perseus"
```

### 2. Launch a real-time rate benchmark test:
```bash
SoapySDRUtil --args="driver=perseus" --rate=1000000 --channels=0 --direction=RX
```
The terminal should stream continuously showing stable throughput metrics (`~0.98 Msps`). Press `Ctrl+C` to close the stream safely.

## Application Usage

Once installed, simply open any compatible SDR application like **SDR++** or **GQRX**:
1. Select **SoapySDR** or **Soapy** as your source device.
2. The receiver will be auto-detected with its specific parameters.
3. Access the **SDR Controls** or **Hardware Settings** tab to check the custom check-boxes for `Wideband Mode`, `ADC Dithering`, and `ADC Preamp`.

## License

This project is licensed under the MIT License - see the LICENSE file for details.  
The hardware interaction layer relies on `libperseus-sdr` (LGPL-2.1).
