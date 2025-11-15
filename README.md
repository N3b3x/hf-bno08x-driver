---
layout: default
title: "HardFOC BNO08x Driver"
description: "Hardware-agnostic BNO08x library for Hillcrest / CEVA BNO08x sensors with full-stack, zero-thread driver support"
nav_order: 1
permalink: /
---

# HF-BNO08x Driver

**Hardware-agnostic BNO08x library for Hillcrest / CEVA BNO08x sensors with full-stack, zero-thread driver support**

[![C++](https://img.shields.io/badge/C%2B%2B-11-blue.svg)](https://en.cppreference.com/w/cpp/11)
[![License](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![CI](https://github.com/N3b3x/hf-bno08x-driver/actions/workflows/esp32-examples-build-ci.yml/badge.svg?branch=main)](https://github.com/N3b3x/hf-bno08x-driver/actions/workflows/esp32-examples-build-ci.yml)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://n3b3x.github.io/hf-bno08x-driver/)

## 📚 Table of Contents
1. [Overview](#-overview)
2. [Features](#-features)
3. [Quick Start](#-quick-start)
4. [Installation](#-installation)
5. [API Reference](#-api-reference)
6. [Examples](#-examples)
7. [Documentation](#-documentation)
8. [Contributing](#-contributing)
9. [License](#-license)

## 📦 Overview

> **📖 [📚🌐 Live Complete Documentation](https://n3b3x.github.io/hf-bno08x-driver/)** - 
> Interactive guides, examples, and step-by-step tutorials

**HF-BNO08x** is a hardware-agnostic C++ library for the **Hillcrest / CEVA BNO08x** family of 9-axis IMU sensors (BNO080, BNO085, BNO086). The BNO08x sensors provide fused orientation data, calibrated IMU measurements, activity detection, step counting, and gesture recognition through the SH-2 (Sensor Hub 2) protocol.

The driver uses a CRTP-based communication interface design, allowing it to run on any platform (ESP32, STM32, Arduino, etc.) with zero runtime overhead. It provides access to all SH-2 sensor reports including rotation vectors, accelerometer, gyroscope, magnetometer, step counter, tap detector, and more. The driver also supports RVC (Robot Vacuum Cleaner) mode for simplified UART streaming and DFU (Device Firmware Update) for firmware updates.

## ✨ Features

- ✅ **Complete SH-2 Coverage**: Access every BNO085 SH-2 report (raw & calibrated IMU, rotation vectors, activity, tap/shake, step counter, etc.)
- ✅ **Hardware Agnostic**: CRTP-based `CommInterface` works with any I²C, SPI, or UART implementation
- ✅ **Zero Internal Threads**: You control timing - call `update()` in your loop, ISR, or RTOS task
- ✅ **Auto Re-Sync**: Detects sensor resets and seamlessly re-enables configured features
- ✅ **Float-Friendly API**: Handy structs (`Vector3`, `Quaternion`, `SensorEvent`) with SI units
- ✅ **RVC Mode Support**: Simplified UART streaming mode for basic orientation data
- ✅ **DFU Support**: Firmware update capability via Device Firmware Update protocol
- ✅ **Pin Control API**: Optional helpers to drive RSTN/BOOTN/WAKE and select I²C, UART, or SPI via PS pins

## 🚀 Quick Start

```cpp
#include "inc/bno08x.hpp"

// 1. Implement the communication interface (see platform_integration.md)
class MyComm : public bno08x::CommInterface<MyComm> {
    // ... implement required methods
};

// 2. Create driver instance
MyComm comm;
bno08x::BNO085 imu(comm);

// 3. Initialize
if (!imu.Begin()) {
    // Handle error
    return;
}

// 4. Enable sensors
imu.EnableSensor(bno08x::BNO085Sensor::RotationVector, 10);  // 100 Hz
imu.EnableSensor(bno08x::BNO085Sensor::StepCounter, 0);     // on-change

// 5. Set callback
imu.SetCallback([](const bno08x::SensorEvent& e) {
    if (e.sensor == bno08x::BNO085Sensor::RotationVector) {
        auto euler = e.toEuler();
        printf("Yaw: %.1f°\n", euler.yaw);
    }
});

// 6. Update loop
while (true) {
    imu.Update();  // Call as often as possible
    delay(5);
}
```

For detailed setup, see [Installation](docs/installation.md) and [Quick Start Guide](docs/quickstart.md).

## 🔧 Installation

1. **Clone or copy** the driver files into your project
2. **Implement the communication interface** for your platform (see [Platform Integration](docs/platform_integration.md))
3. **Include the header** in your code:
   ```cpp
   #include "inc/bno08x.hpp"
   ```
4. Compile with a **C++11** or newer compiler

For detailed installation instructions, see [docs/installation.md](docs/installation.md).

## 📖 API Reference

| Method | Description |
|--------|-------------|
| `Begin()` | Initialize the sensor |
| `EnableSensor()` | Enable periodic reporting for a sensor |
| `DisableSensor()` | Disable reporting for a sensor |
| `Update()` | Pump the SH-2 service loop |
| `SetCallback()` | Register callback for sensor events |
| `GetLatest()` | Get most recent event for a sensor |
| `HasNewData()` | Check if new data is available |

For complete API documentation, see [docs/api_reference.md](docs/api_reference.md).

## 📊 Examples

For ESP32 examples, see the [examples/esp32](examples/esp32/) directory.

Detailed example walkthroughs are available in [docs/examples.md](docs/examples.md).

## 📚 Documentation

For complete documentation, see the [docs directory](docs/index.md).

### Special Features

- **[RVC Mode](docs/special_feature_rvc.md)** - Simplified UART streaming mode
- **[DFU (Firmware Update)](docs/special_feature_dfu.md)** - Device firmware update guide

## 🤝 Contributing

Pull requests and suggestions are welcome! Please follow the existing code style and include tests for new features.

## 📄 License

- **C++ wrapper code**: [GNU GPL v3.0](LICENSE)
- **CEVA SH-2 backend**: Apache 2.0 (included)

By contributing you agree your code is released under the same GPLv3 license.
