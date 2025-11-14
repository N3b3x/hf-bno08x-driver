#pragma once

/**
 * @file bno08x.hpp
 * @brief High level C++ interface for the BNO08x IMU family.
 *
 * This header declares the ::BNO085 class which wraps the vendor supplied
 * SH-2 driver.  It also defines small helper structures used to represent
 * sensor values in a user friendly form.
 */

#include "bno08x_comm_interface.hpp"
#include "dfu/HcBin.h"
#include "dfu/firmware.h"
#include "rvc/Rvc.hpp"
#include <array>
#include <cstdint>
#include <functional>

extern "C" {
#include "sh2/sh2.h"
#include "sh2/sh2_SensorValue.h"
}

/**
 * @enum BNO085Sensor
 * @brief Identifiers for all sensor reports supported by the BNO08x.
 */
enum class BNO085Sensor : uint8_t {
  Accelerometer = 0x01,              ///< Calibrated acceleration vector
  Gyroscope = 0x02,                  ///< Calibrated angular velocity
  Magnetometer = 0x03,               ///< Calibrated magnetic field
  LinearAcceleration = 0x04,         ///< Acceleration minus gravity
  RotationVector = 0x05,             ///< Fused orientation quaternion
  Gravity = 0x06,                    ///< Gravity vector
  GyroUncalibrated = 0x07,           ///< Raw angular velocity
  GameRotationVector = 0x08,         ///< Orientation without heading
  GeomagneticRotationVector = 0x09,  ///< Fused orientation using magnetometer
  Pressure = 0x0A,                   ///< Barometric pressure
  AmbientLight = 0x0B,               ///< Ambient light level
  Humidity = 0x0C,                   ///< Relative humidity
  Proximity = 0x0D,                  ///< Proximity detector
  Temperature = 0x0E,                ///< Temperature sensor
  MagneticFieldUncalibrated = 0x0F,  ///< Raw magnetic field
  TapDetector = 0x10,                ///< Single or double tap event
  StepCounter = 0x11,                ///< Step counter
  SignificantMotion = 0x12,          ///< Significant motion event
  StabilityClassifier = 0x13,        ///< Device stability state
  RawAccelerometer = 0x14,           ///< Raw accelerometer data
  RawGyroscope = 0x15,               ///< Raw gyroscope data
  RawMagnetometer = 0x16,            ///< Raw magnetometer data
  StepDetector = 0x18,               ///< Step detected event
  ShakeDetector = 0x19,              ///< Shake gesture event
  FlipDetector = 0x1A,               ///< Flip gesture event
  PickupDetector = 0x1B,             ///< Pickup gesture event
  StabilityDetector = 0x1C,          ///< Stability detected event
  PersonalActivityClassifier = 0x1E, ///< Activity classification
  SleepDetector = 0x1F,              ///< Sleep state
  TiltDetector = 0x20,               ///< Tilt event
  PocketDetector = 0x21,             ///< Device is in pocket
  CircleDetector = 0x22,             ///< Circular motion
  HeartRateMonitor = 0x23,           ///< Heart rate monitor data
  ARVRStabilizedRV = 0x28,           ///< Stabilized rotation vector
  ARVRStabilizedGameRV = 0x29,       ///< Stabilized game rotation vector
  GyroIntegratedRV = 0x2A            ///< Gyro integrated rotation vector
};
/**
 * @enum BNO085Interface
 * @brief Host interface selection via PS pins.
 */
enum class BNO085Interface : uint8_t {
  I2C,     ///< PS1=0, PS0=0
  UARTRVC, ///< PS1=1, PS0=0
  UART,    ///< PS1=0, PS0=1
  SPI      ///< PS1=1, PS0=1
};

/**
 * @struct Vector3
 * @brief Simple 3‑axis vector with accuracy flag.
 */
struct Vector3 {
  float x{0};          ///< X component
  float y{0};          ///< Y component
  float z{0};          ///< Z component
  uint8_t accuracy{0}; ///< Sensor accuracy (0‑3)
};

/**
 * @struct Quaternion
 * @brief Quaternion orientation with accuracy flag.
 */
struct Quaternion {
  float w{1};          ///< Real component
  float x{0};          ///< i component
  float y{0};          ///< j component
  float z{0};          ///< k component
  uint8_t accuracy{0}; ///< Sensor accuracy (0‑3)
};

/**
 * @struct TapEvent
 * @brief Information for tap detector events.
 */
struct TapEvent {
  bool doubleTap{false}; ///< True if a double tap occurred
  uint8_t direction{0};  ///< Tap direction (0–5)
};

/**
 * @struct SensorEvent
 * @brief Container for a single sensor report.
 */
struct SensorEvent {
  BNO085Sensor sensor{BNO085Sensor::Accelerometer}; ///< Source sensor ID
  uint64_t timestamp{0};                            ///< Time in microseconds
  Vector3 vector{};                                 ///< 3‑axis data
  Quaternion rotation{};                            ///< Orientation data
  uint32_t stepCount{0};                            ///< Step counter value
  TapEvent tap{};                                   ///< Tap detector info
  bool detected{false};                             ///< Generic detection flag
};

/** Callback type invoked when a new ::SensorEvent is received. */
using SensorCallback = std::function<void(const SensorEvent&)>;

/** Callback type for decoded RVC frames. */
using RvcCallback = std::function<void(const rvc_SensorValue_t&)>;

/**
 * @class BNO085
 * @brief High level driver for the BNO08x IMU.
 *
 * The driver wraps the vendor SH-2 C API and exposes a simple C++ interface.
 * A platform specific communication interface implementing bno08x::CommInterface<Derived> must be
 * supplied to actually read and write bytes on the bus.
 *
 * @tparam CommType The communication interface type (must inherit from bno08x::CommInterface<CommType>)
 *
 * @note The driver uses CRTP-based communication interface for zero virtual call overhead.
 *       Communication interface implementations should inherit from bno08x::CommInterface<DerivedType>.
 */
template <typename CommType>
class BNO085 {
public:
  /**
   * @brief Construct the driver with a communication interface instance.
   * @param comm Communication interface used for communication (must outlive this object).
   */
  explicit BNO085(CommType& comm) noexcept : io_(comm) {}

  /** Initialize the sensor using the communication interface passed in the constructor. */
  bool Begin() noexcept;

  /**
   * @brief Enable periodic reporting for a sensor.
   * @param sensor     Which sensor to enable.
   * @param intervalMs Desired report interval in milliseconds.
   * @param sensitivity Change sensitivity for on-change sensors.
   */
  bool EnableSensor(BNO085Sensor sensor, uint32_t intervalMs, float sensitivity = 0.0f);
  /** Disable reporting for a sensor. */
  bool DisableSensor(BNO085Sensor sensor);

  /** Register a callback invoked for every received event. */
  void SetCallback(SensorCallback cb);

  /** Register a callback for decoded RVC frames. */
  void SetRvcCallback(RvcCallback cb);

  /** Begin processing in RVC mode using the given HAL. */
  bool BeginRvc(IRvcHal* hal);
  /** Poll the UART and dispatch any pending RVC frames. */
  void ServiceRvc();
  /** Stop RVC processing. */
  void CloseRvc();

  /** Check if new data is available for a sensor. */
  bool HasNewData(BNO085Sensor sensor) const;
  /** Return the most recent event for a sensor. */
  SensorEvent GetLatest(BNO085Sensor sensor) const;

  /** Pump the SH-2 service loop. Call this as often as possible. */
  void Update();

  /** Retrieve the last error code returned by the SH-2 driver. */
  int GetLastError() const {
    return last_error_;
  }

  /**
   * @brief Toggle the sensor's hardware reset line if available.
   *
   * Drives RSTN low for the specified time then releases it. Platforms not
   * providing the pin may leave the implementation empty.
   */
  void HardwareReset(uint32_t lowMs = 2);

  /** Set the BOOTN pin level (used to enter DFU). */
  void SetBootPin(bool state);
  /** Control the WAKE pin in SPI mode. */
  void SetWakePin(bool state);

  /** Select the host interface by driving PS pins. */
  void SelectInterface(BNO085Interface iface);

  /**
   * @brief Perform a firmware update using this object's communication interface.
   *
   * The sensor must already be in bootloader mode (BOOTN held low during
   * reset). Provide the firmware as an `HcBin_t` object—use the default
   * `firmware` from `src/dfu` or construct one at runtime with
   * `MemoryFirmware`.
   *
   * @param fw Firmware image to write.
   * @return SH2 status code from the DFU routine.
   */
  int Dfu(const HcBin_t& fw = firmware);

private:
  /**
   * @brief Internal wrapper converting CommInterface to the SH-2 HAL.
   */
  struct CommHal {
    sh2_Hal_t* asHal() {
      return &hal;
    }
    sh2_Hal_t hal;                    ///< SH-2 HAL structure
    CommType* comm{nullptr}; ///< User provided communication interface (pointer for C callbacks) - note: this is a struct member, not a class member, so no underscore needed
  } halWrapper{};

  /// @name SH-2 HAL callbacks
  /// @{
  static int halOpen(sh2_Hal_t* self);
  static void halClose(sh2_Hal_t* self);
  static int halRead(sh2_Hal_t* self, uint8_t* buf, unsigned len, uint32_t* t);
  static int halWrite(sh2_Hal_t* self, uint8_t* buf, unsigned len);
  static uint32_t halGetTimeUs(sh2_Hal_t* self);
  /// @}

  /// C trampoline for sensor callbacks
  static void sensorC(void* cookie, sh2_SensorEvent_t* event);
  /// C trampoline for async callbacks
  static void asyncC(void* cookie, sh2_AsyncEvent_t* event);
  /// C trampoline for RVC frames
  static void rvcC(void* cookie, rvc_SensorEvent_t* event);

  void handleSensorEvent(const sh2_SensorEvent_t* event);
  void handleAsyncEvent(const sh2_AsyncEvent_t* event);
  bool configure(BNO085Sensor sensor, uint32_t intervalUs, float sensitivity, uint32_t batchUs = 0);

  CommType& io_; ///< Communication interface reference (must outlive this object)
  SensorCallback callback_{};
  int last_error_{0};
  bool initialized_{false};

  std::array<sh2_SensorValue_t, 0x2B> latest_{};
  std::array<bool, 0x2B> new_flag_{};
  std::array<uint32_t, 0x2B> last_interval_{};
  std::array<float, 0x2B> last_sensitivity_{};

  Rvc rvc_{};
  RvcCallback rvc_cb_{};
};

// Include template implementation
#define BNO085_HEADER_INCLUDED
#include "../src/bno08x.cpp"
#undef BNO085_HEADER_INCLUDED
