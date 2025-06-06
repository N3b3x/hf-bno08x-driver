# BNO085 Complete Guide

[⬅️ Docs Hub](README.md) | [Main README](../README.md)

This document compiles information from the official BNO080/085 datasheet and expands on the existing README. It explains how the sensor works, covers wiring, available modes, and shows example code for both **Arduino** and **ESP32** platforms.

For a quick start see the [main README](../README.md). Additional topics are
covered in the [RVC mode guide](../src/rvc/README.md) and the
[DFU framework guide](../src/dfu/README.md).

## Overview

The **BNO085** is a 9‑axis absolute orientation sensor from Hillcrest Labs/CEVA. It combines a 3‑axis gyroscope, 3‑axis accelerometer and 3‑axis magnetometer with an onboard 32‑bit microcontroller running the **SH‑2** sensor fusion software. The IMU outputs a wide range of sensor data and high level events without requiring heavy computation on the host.

### Key Features

- Integrated 9‑DoF inertial and geomagnetic sensors
- Onboard sensor fusion with quaternion or Euler output
- Multiple communication interfaces: **I²C**, **SPI**, **UART**, and **RVC** (Robot Vacuum Cleaner) mode
- Report rates from 1 Hz up to 400 Hz depending on feature
- Single supply operation from 2.0–3.6 V (typical 3.3 V)
- Programmable interrupt for data ready or event notifications

Refer to `datasheet/BNO080_085-Datasheet.pdf` for detailed electrical characteristics and timing diagrams.

## Pin Functions

```
      .------- BNO085 -------.
     |                       |
 VIN  | 1 VDD          BOOTN | 12  ➜  Boot mode select
 GND  | 2 GND          RSTN  | 11  ➜  Active‑low reset
 SCL  | 3 SCL/PS0      WAKE  | 10  ➜  Wake (active‑high)
 SDA  | 4 SDA/PS1      INTN  | 9   ➜  Interrupt (active‑low)
 SA0  | 5 ADR          PS2   | 8   ➜  Not used on BNO085
     |                       |
     `-----------------------'
```

- **PS0 / PS1** pins select the interface:
  - `0,0` – I²C
  - `0,1` – UART
  - `1,1` – SPI
  - `1,0` – UART in RVC mode
- **SA0/ADR** chooses the I²C address (`0x4A` when low, `0x4B` when high)
- **RSTN** resets the device when pulled low for >10 µs
- **BOOTN** held low on reset enters DFU (bootloader) mode

## Wiring Diagrams

### I²C Connection

```
MCU 3V3 ─── VIN   BNO085
MCU GND ─── GND
MCU SCL ─── SCL  (4.7 kΩ pull‑up)
MCU SDA ─── SDA  (4.7 kΩ pull‑up)
MCU GPIO ─── INT (optional, active‑low)
MCU GPIO ─── RSTN (optional reset)
PS0 = 0, PS1 = 0 ➜ I²C mode
SA0 = 0 ➜ address 0x4A
```

### SPI Connection

```
MCU 3V3 ── VIN           BNO085
MCU GND ── GND
MCU SCK ── SCL/CLK
MCU MOSI ─ SDA/MOSI
MCU MISO ─ ────> (MISO on some boards)
MCU CS   ── CS          (if present)
MCU GPIO ── INT         (optional)
MCU GPIO ── RSTN        (optional reset)
PS0 = 1, PS1 = 1 ➜ SPI mode
```

The INT line can be polled or used as an interrupt. Using it allows very low‑power operation because the host only wakes when the sensor has new data ready.

## Building the Library

The library requires a C++11 compiler and an implementation of `IBNO085Transport` for your hardware platform. The `src` directory contains the platform‑agnostic driver while `examples` show how to implement the transport.

### Using CMake

```
# Clone somewhere inside your project
git clone --depth=1 https://github.com/yourOrg/bno085-cpp.git libs/bno085

# Add to your build
add_subdirectory(libs/bno085)
target_link_libraries(myApp PRIVATE bno085)
```

### Arduino Sketch

Copy the `src` folder into an Arduino library or use it with PlatformIO. Implement the transport with `Wire` as shown below.

```cpp
#include <Wire.h>
#include "BNO085.hpp"

class ArduinoTransport : public IBNO085Transport {
public:
  bool open() override { Wire.begin(); Wire.setClock(400000); delay(10); return true; }
  void close() override {}
  int write(const uint8_t *b, size_t n) override {
    Wire.beginTransmission(0x4A);
    Wire.write(b, n);
    return Wire.endTransmission() == 0 ? n : -1;
  }
  int read(uint8_t *b, size_t n) override {
    Wire.requestFrom(0x4A, n);
    size_t i = 0;
    while (Wire.available() && i < n) b[i++] = Wire.read();
    return i;
  }
  bool dataAvailable() override { return true; }
  void delay(uint32_t ms) override { ::delay(ms); }
  uint32_t getTimeUs() override { return micros(); }
};
```

### ESP32 (ESP‑IDF)

```cpp
#include "driver/i2c.h"
#include "BNO085.hpp"

class Esp32I2CTransport : public IBNO085Transport {
public:
  bool open() override {
    i2c_config_t cfg{};
    cfg.mode = I2C_MODE_MASTER;
    cfg.sda_io_num = GPIO_NUM_21;
    cfg.scl_io_num = GPIO_NUM_22;
    cfg.master.clk_speed = 400000;
    i2c_param_config(I2C_NUM_0, &cfg);
    return i2c_driver_install(I2C_NUM_0, cfg.mode, 0, 0, 0) == ESP_OK;
  }
  void close() override { i2c_driver_delete(I2C_NUM_0); }
  int write(const uint8_t *data, size_t len) override {
    return i2c_master_write_to_device(I2C_NUM_0, 0x4A, data, len, 1000) == ESP_OK ? len : -1;
  }
  int read(uint8_t *data, size_t len) override {
    return i2c_master_read_from_device(I2C_NUM_0, 0x4A, data, len, 1000) == ESP_OK ? len : -1;
  }
  bool dataAvailable() override { return true; }
  void delay(uint32_t ms) override { vTaskDelay(pdMS_TO_TICKS(ms)); }
  uint32_t getTimeUs() override { return esp_timer_get_time(); }
};
```

These snippets show minimal transports; the full examples in the `examples` folder include polling loops and callbacks.

## Sensor Reports and Modes

The BNO085 can output many different reports. Each can be enabled at a custom interval or on-change. The enumeration below (from `BNO085Sensor` in `src/BNO085.hpp`) lists the most common ones:

```
Accelerometer
Gyroscope
Magnetometer
LinearAcceleration
RotationVector
Gravity
GyroUncalibrated
GameRotationVector
GeomagneticRotationVector
Pressure
AmbientLight
Humidity
Proximity
Temperature
MagneticFieldUncalibrated
TapDetector
StepCounter
SignificantMotion
StabilityClassifier
RawAccelerometer
RawGyroscope
RawMagnetometer
StepDetector
ShakeDetector
FlipDetector
PickupDetector
StabilityDetector
PersonalActivityClassifier
SleepDetector
TiltDetector
PocketDetector
CircleDetector
HeartRateMonitor
ARVRStabilizedRV
ARVRStabilizedGameRV
GyroIntegratedRV
```

Use `enableSensor()` to turn on the reports you need. For example, to stream the fused orientation quaternion at 100 Hz:

```cpp
imu.enableSensor(BNO085Sensor::RotationVector, 10); // 10 ms interval
```

## RVC Mode

Robot Vacuum Cleaner mode is a simplified UART protocol. Instead of sending SH‑2 commands, the sensor streams 19‑byte frames at 115200 bps. `src/rvc/README.md` describes the frame format in detail. To use it with this library:

```cpp
#include "rvc/RvcHalEsp32C6.hpp"
BNO085 imu;
Esp32C6RvcHal hal;
imu.setRvcCallback([](const rvc_SensorValue_t &v) {
  printf("Yaw %.2f Pitch %.2f Roll %.2f\n", v.yaw_deg, v.pitch_deg, v.roll_deg);
});
if (imu.beginRvc(&hal)) {
  while (true) {
    imu.serviceRvc();
  }
}
```

## Experiments and Tips

- **Calibration** – Move the sensor in a figure‑eight pattern after power‑up to achieve accuracy level 3.
- **Tare** – Call `imu.tareNow()` to zero the orientation when the device is in your desired reference pose.
- **DFU** – Hold **BOOTN** low during reset and use `BNO085::dfu()` along with a transport implementing `IDfuTransport` to update firmware.
- **Power Saving** – Disable unused reports and use the INT pin to wake the host only when needed.
- **Host Timing** – Call `imu.update()` as often as possible. In an RTOS environment, poll when INT triggers to minimize latency.

## Example Application: Orientation Viewer

```cpp
#include "ArduinoTransport.h" // from above example
#include "BNO085.hpp"

ArduinoTransport transport;
BNO085 imu(&transport);

void setup() {
  Serial.begin(115200);
  if (!imu.begin()) {
    Serial.println("IMU not found!");
    while (1);
  }
  imu.enableSensor(BNO085Sensor::RotationVector, 10); // 100 Hz
}

void loop() {
  imu.update();
  if (imu.hasNewData(BNO085Sensor::RotationVector)) {
    auto e = imu.getLatest(BNO085Sensor::RotationVector);
    float yaw = atan2f(2.0f * (e.rotation.w * e.rotation.z + e.rotation.x * e.rotation.y),
                       1.0f - 2.0f * (e.rotation.y * e.rotation.y + e.rotation.z * e.rotation.z));
    Serial.print("Yaw: ");
    Serial.println(yaw * 180.0f / PI);
  }
  delay(10);
}
```

This example prints the yaw angle at 100 Hz on an Arduino‑compatible board.

---

For more information consult the official datasheet and the examples provided
with this repository. You can also explore firmware updates in the
[DFU framework guide](../src/dfu/README.md) or learn about the simplified
[RVC mode](../src/rvc/README.md).

---

[⬅️ Back to Docs Hub](README.md)
