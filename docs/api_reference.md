# API Reference

Complete reference documentation for all public methods and types in the BNO08x driver.

## Source Code

- **Main Header**: [`inc/bno08x.hpp`](../inc/bno08x.hpp)
- **Communication Interface**: [`inc/bno08x_comm_interface.hpp`](../inc/bno08x_comm_interface.hpp)
- **Implementation**: [`src/bno08x.cpp`](../src/bno08x.cpp)

## Core Class

### `BNO085<CommType>`

Main driver class for interfacing with the BNO08x IMU sensors.

**Template Parameter**: `CommType` - Your communication interface implementation (must inherit from `bno08x::CommInterface<CommType>`)

**Location**: [`inc/bno08x.hpp#L144`](../inc/bno08x.hpp#L144)

**Constructor:**
```cpp
explicit BNO085(CommType& comm) noexcept;
```

**Location**: [`inc/bno08x.hpp#L150`](../inc/bno08x.hpp#L150)

## Methods

### Initialization

| Method | Signature | Location |
|--------|-----------|----------|
| `Begin()` | `bool Begin() noexcept` | [`inc/bno08x.hpp#L153`](../inc/bno08x.hpp#L153) |

### Sensor Control

| Method | Signature | Location |
|--------|-----------|----------|
| `EnableSensor()` | `bool EnableSensor(BNO085Sensor sensor, uint32_t intervalMs, float sensitivity = 0.0f)` | [`inc/bno08x.hpp#L161`](../inc/bno08x.hpp#L161) |
| `DisableSensor()` | `bool DisableSensor(BNO085Sensor sensor)` | [`inc/bno08x.hpp#L163`](../inc/bno08x.hpp#L163) |

### Callbacks

| Method | Signature | Location |
|--------|-----------|----------|
| `SetCallback()` | `void SetCallback(SensorCallback cb)` | [`inc/bno08x.hpp#L166`](../inc/bno08x.hpp#L166) |
| `SetRvcCallback()` | `void SetRvcCallback(RvcCallback cb)` | [`inc/bno08x.hpp#L169`](../inc/bno08x.hpp#L169) |

### RVC Mode

| Method | Signature | Location |
|--------|-----------|----------|
| `BeginRvc()` | `bool BeginRvc(IRvcHal* hal)` | [`inc/bno08x.hpp#L172`](../inc/bno08x.hpp#L172) |
| `ServiceRvc()` | `void ServiceRvc()` | [`inc/bno08x.hpp#L174`](../inc/bno08x.hpp#L174) |
| `CloseRvc()` | `void CloseRvc()` | [`inc/bno08x.hpp#L176`](../inc/bno08x.hpp#L176) |

### Data Access

| Method | Signature | Location |
|--------|-----------|----------|
| `HasNewData()` | `bool HasNewData(BNO085Sensor sensor) const` | [`inc/bno08x.hpp#L179`](../inc/bno08x.hpp#L179) |
| `GetLatest()` | `SensorEvent GetLatest(BNO085Sensor sensor) const` | [`inc/bno08x.hpp#L181`](../inc/bno08x.hpp#L181) |

### Main Loop

| Method | Signature | Location |
|--------|-----------|----------|
| `Update()` | `void Update()` | [`inc/bno08x.hpp#L184`](../inc/bno08x.hpp#L184) |

### Error Handling

| Method | Signature | Location |
|--------|-----------|----------|
| `GetLastError()` | `int GetLastError() const` | [`inc/bno08x.hpp#L187`](../inc/bno08x.hpp#L187) |

### Hardware Control

| Method | Signature | Location |
|--------|-----------|----------|
| `HardwareReset()` | `void HardwareReset(uint32_t lowMs = 2)` | [`inc/bno08x.hpp#L197`](../inc/bno08x.hpp#L197) |
| `SetBootPin()` | `void SetBootPin(bool state)` | [`inc/bno08x.hpp#L200`](../inc/bno08x.hpp#L200) |
| `SetWakePin()` | `void SetWakePin(bool state)` | [`inc/bno08x.hpp#L202`](../inc/bno08x.hpp#L202) |
| `SelectInterface()` | `void SelectInterface(BNO085Interface iface)` | [`inc/bno08x.hpp#L205`](../inc/bno08x.hpp#L205) |

### Firmware Update

| Method | Signature | Location |
|--------|-----------|----------|
| `Dfu()` | `int Dfu(const HcBin_t& fw = firmware)` | [`inc/bno08x.hpp#L218`](../inc/bno08x.hpp#L218) |

## Types

### Enumerations

| Type | Values | Location |
|------|--------|----------|
| `BNO085Sensor` | `Accelerometer`, `Gyroscope`, `Magnetometer`, `LinearAcceleration`, `RotationVector`, `Gravity`, `GyroUncalibrated`, `GameRotationVector`, `GeomagneticRotationVector`, `Pressure`, `AmbientLight`, `Humidity`, `Proximity`, `Temperature`, `MagneticFieldUncalibrated`, `TapDetector`, `StepCounter`, `SignificantMotion`, `StabilityClassifier`, `RawAccelerometer`, `RawGyroscope`, `RawMagnetometer`, `StepDetector`, `ShakeDetector`, `FlipDetector`, `PickupDetector`, `StabilityDetector`, `PersonalActivityClassifier`, `SleepDetector`, `TiltDetector`, `PocketDetector`, `CircleDetector`, `HeartRateMonitor`, `ARVRStabilizedRV`, `ARVRStabilizedGameRV`, `GyroIntegratedRV` | [`inc/bno08x.hpp#L29`](../inc/bno08x.hpp#L29) |
| `BNO085Interface` | `I2C`, `UARTRVC`, `UART`, `SPI` | [`inc/bno08x.hpp#L71`](../inc/bno08x.hpp#L71) |

### Structures

| Type | Description | Location |
|------|-------------|----------|
| `Vector3` | 3-axis vector with accuracy flag | [`inc/bno08x.hpp#L82`](../inc/bno08x.hpp#L82) |
| `Quaternion` | Quaternion orientation with accuracy flag | [`inc/bno08x.hpp#L93`](../inc/bno08x.hpp#L93) |
| `TapEvent` | Tap detector event information | [`inc/bno08x.hpp#L105`](../inc/bno08x.hpp#L105) |
| `SensorEvent` | Container for a single sensor report | [`inc/bno08x.hpp#L114`](../inc/bno08x.hpp#L114) |

### Type Aliases

| Type | Definition | Location |
|------|------------|----------|
| `SensorCallback` | `std::function<void(const SensorEvent&)>` | [`inc/bno08x.hpp#L125`](../inc/bno08x.hpp#L125) |
| `RvcCallback` | `std::function<void(const rvc_SensorValue_t&)>` | [`inc/bno08x.hpp#L128`](../inc/bno08x.hpp#L128) |

## Communication Interface

### `CommInterface<Derived>`

CRTP-based communication interface that must be implemented for your platform.

**Location**: [`inc/bno08x_comm_interface.hpp#L44`](../inc/bno08x_comm_interface.hpp#L44)

### Required Methods

| Method | Signature | Location |
|--------|-----------|----------|
| `Open()` | `bool Open() noexcept` | [`inc/bno08x_comm_interface.hpp#L51`](../inc/bno08x_comm_interface.hpp#L51) |
| `Close()` | `void Close() noexcept` | [`inc/bno08x_comm_interface.hpp#L58`](../inc/bno08x_comm_interface.hpp#L58) |
| `Write()` | `int Write(const uint8_t* data, uint32_t length) noexcept` | [`inc/bno08x_comm_interface.hpp#L68`](../inc/bno08x_comm_interface.hpp#L68) |
| `Read()` | `int Read(uint8_t* data, uint32_t length) noexcept` | [`inc/bno08x_comm_interface.hpp#L78`](../inc/bno08x_comm_interface.hpp#L78) |
| `DataAvailable()` | `bool DataAvailable() noexcept` | [`inc/bno08x_comm_interface.hpp#L86`](../inc/bno08x_comm_interface.hpp#L86) |
| `Delay()` | `void Delay(uint32_t ms) noexcept` | [`inc/bno08x_comm_interface.hpp#L94`](../inc/bno08x_comm_interface.hpp#L94) |
| `GetTimeUs()` | `uint32_t GetTimeUs() noexcept` | [`inc/bno08x_comm_interface.hpp#L104`](../inc/bno08x_comm_interface.hpp#L104) |

### Optional Methods

| Method | Signature | Location |
|--------|-----------|----------|
| `SetReset()` | `void SetReset(bool state) noexcept` | [`inc/bno08x_comm_interface.hpp#L114`](../inc/bno08x_comm_interface.hpp#L114) |
| `SetBoot()` | `void SetBoot(bool state) noexcept` | [`inc/bno08x_comm_interface.hpp#L124`](../inc/bno08x_comm_interface.hpp#L124) |
| `SetWake()` | `void SetWake(bool state) noexcept` | [`inc/bno08x_comm_interface.hpp#L134`](../inc/bno08x_comm_interface.hpp#L134) |
| `SetPS0()` | `void SetPS0(bool state) noexcept` | [`inc/bno08x_comm_interface.hpp#L144`](../inc/bno08x_comm_interface.hpp#L144) |
| `SetPS1()` | `void SetPS1(bool state) noexcept` | [`inc/bno08x_comm_interface.hpp#L148`](../inc/bno08x_comm_interface.hpp#L148) |

---

**Navigation**
⬅️ [Configuration](configuration.md) | [Next: Examples ➡️](examples.md) | [Back to Index](index.md)

