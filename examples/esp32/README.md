# BNO08x ESP32-C6 Examples

This directory contains comprehensive examples demonstrating the BNO08x driver on ESP32-C6 platform.

## 🎯 Overview

The ESP32-C6 examples showcase real-world usage of the BNO08x IMU sensor with:

- **Hardware-specific HAL** implementation for ESP32-C6
- **Multiple example applications** covering different use cases
- **Automated build system** with configurable app types
- **Comprehensive documentation** for each example
- **Production-ready code** with proper error handling

## 🔧 Hardware Requirements

### ESP32-C6 Development Board
- ESP32-C6-DevKitC-1 or compatible
- USB-C cable for programming and power

### BNO08x Connections

| BNO08x Pin | ESP32-C6 GPIO | Function |
|------------|---------------|----------|
| SDA | GPIO21 | I2C Data |
| SCL | GPIO22 | I2C Clock |
| VDD | 3.3V | Logic Supply |
| GND | GND | Ground |
| INT (optional) | GPIO_NC | Interrupt (optional) |
| RST (optional) | GPIO_NC | Reset (optional) |

**Note:** The default I2C address is 0x4A. If ADR/SA0 is tied high, use address 0x4B.

## 🚀 Quick Start

### 1. Prerequisites

```bash
# Install ESP-IDF v5.5
curl -fsSL https://raw.githubusercontent.com/espressif/esp-idf/master/tools/install.sh | bash
source ~/esp/esp-idf/export.sh

# Verify installation
idf.py --version
```

### 2. Setup Repository

```bash
# Clone and setup
git clone --recursive https://github.com/n3b3x/hf-bno08x-driver.git
cd hf-bno08x-driver/examples/esp32

# Initialize build environment
./scripts/setup_repo.sh
```

### 3. Build and Flash

```bash
# Build driver integration test (default)
./scripts/build_app.sh driver_integration_test Release

# Flash to ESP32-C6
./scripts/flash_app.sh driver_integration_test Release

# Monitor output
idf.py monitor
```

## 📱 Available Examples

### 🟢 Test Suite

#### `driver_integration_test`
**Comprehensive Driver Integration Test Suite**
- Complete driver API validation (30+ tests)
- Multiple test sections covering all functionality
- Initialization, sensor enable/disable, data reading
- Polling and callback modes
- RVC mode testing
- Error condition validation
- FreeRTOS task-based test execution
- GPIO14 progress indicator
- Automatic pass/fail tracking
- No actual hardware required (driver test only)

**Build:**
```bash
./scripts/build_app.sh driver_integration_test Release
```

### 🟡 Basic Examples

#### `basic_polling`
**Basic Polling Mode Example**
- Simple polling loop
- Rotation vector and linear acceleration sensors
- Euler angle calculation
- Real hardware testing

**Build:**
```bash
./scripts/build_app.sh basic_polling Release
```

#### `event_driven_callback`
**Event-Driven Callback Example**
- Callback-based event handling
- Step counter and tap detector
- Gesture detection
- Real hardware testing

**Build:**
```bash
./scripts/build_app.sh event_driven_callback Release
```

#### `full_features`
**Comprehensive Example**
- Multiple sensor types enabled
- Both callback and polling methods
- Orientation, acceleration, gyroscope, gravity
- Step counter, tap detector, gesture events
- Real hardware testing

**Build:**
```bash
./scripts/build_app.sh full_features Release
```

#### `rvc_mode`
**RVC Mode Example**
- RVC mode initialization
- Frame callback handling
- Yaw, pitch, roll reading
- Real hardware testing

**Build:**
```bash
./scripts/build_app.sh rvc_mode Release
```

## 🔨 Building Examples

### Using Build Scripts (Recommended)

```bash
# List available app types
./scripts/build_app.sh list

# Build specific app
./scripts/build_app.sh <app_type> <build_type>

# Examples:
./scripts/build_app.sh driver_integration_test Release
./scripts/build_app.sh basic_polling Debug
```

### Using ESP-IDF Directly

```bash
# Set app type and build type
idf.py build -DAPP_TYPE=driver_integration_test -DBUILD_TYPE=Release

# Flash
idf.py flash

# Monitor
idf.py monitor
```

## 📊 Test Framework

All test examples use a standardized test framework with:

- **Test result tracking** - Automatic pass/fail counting
- **Execution timing** - Performance measurement
- **FreeRTOS task execution** - Isolated test environments
- **GPIO14 progress indicator** - Visual test progression
- **Standardized logging** - Consistent output format

### Test Framework Features

- `RUN_TEST(test_func)` - Run test inline
- `RUN_TEST_IN_TASK(name, func, stack_size, priority)` - Run test in FreeRTOS task
- `RUN_TEST_SECTION_IF_ENABLED(enabled, section_name, ...)` - Conditional test sections
- `g_test_results` - Global test results tracking

## 🔌 Hardware Configuration

### I2C Configuration

The default I2C configuration can be modified in the example files:

```cpp
Esp32Bno08xBus::I2CConfig config;
config.sda_pin = GPIO_NUM_21;      // SDA pin
config.scl_pin = GPIO_NUM_22;      // SCL pin
config.frequency = 400000;         // I2C frequency (400kHz)
config.device_address = 0x4A;      // I2C address (0x4A or 0x4B)
config.int_pin = GPIO_NUM_NC;      // Interrupt pin (optional)
config.rst_pin = GPIO_NUM_NC;      // Reset pin (optional)
```

### Pin Configuration

| Function | Default GPIO | Notes |
|----------|--------------|-------|
| I2C SDA | GPIO21 | Can be changed in code |
| I2C SCL | GPIO22 | Can be changed in code |
| Interrupt | GPIO_NC | Optional, for data ready |
| Reset | GPIO_NC | Optional, for hardware reset |
| Test Progress | GPIO14 | Used by test framework |

## 📚 Documentation

For more detailed documentation, see:

- [Main Driver README](../../README.md)
- [Hardware Wiring Guide](../../docs/HardwareWiring.md)
- [Porting Guide](../../docs/PortingGuide.md)
- [Examples Guide](../../docs/Examples.md)
- [BNO085 Complete Guide](../../docs/BNO085_Complete_Guide.md)

## 🐛 Troubleshooting

### Build Issues

**Error: "APP_TYPE not defined"**
- Use the build scripts: `./scripts/build_app.sh <app_type> <build_type>`
- Or set manually: `idf.py build -DAPP_TYPE=driver_integration_test -DBUILD_TYPE=Release`

**Error: "Failed to read valid app types from app_config.yml"**
- Ensure the scripts submodule is initialized: `git submodule update --init --recursive`
- Check that `scripts/get_app_info.py` exists

### Runtime Issues

**I2C Communication Errors**
- Check wiring connections (SDA, SCL, GND, VDD)
- Verify I2C address (0x4A or 0x4B)
- Check I2C pull-up resistors (usually on board)
- Verify I2C frequency (400kHz default)

**No Sensor Data**
- Ensure sensors are enabled: `imu.enableSensor(...)`
- Call `imu.update()` regularly in your loop
- Check interrupt pin if using data ready signal
- Verify sensor is powered and connected

**Initialization Failures**
- Check I2C bus initialization
- Verify device address
- Ensure proper power supply (3.3V)
- Check for I2C bus conflicts

## 📝 License

See the main driver [LICENSE](../../LICENSE) file.

## 🤝 Contributing

See the main driver [Contributing Guide](../../README.md#contributing-🤝).

---

**For questions or issues, please open an issue on GitHub.**

