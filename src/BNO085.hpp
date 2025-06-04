#pragma once

/**
 * @file BNO085.hpp
 * @brief High level C++ interface for the BNO08x IMU family.
 *
 * This header declares the ::BNO085 class which wraps the vendor supplied
 * SH-2 driver.  It also defines small helper structures used to represent
 * sensor values in a user friendly form.
 */

#include "BNO085_Transport.hpp"
#include <array>
#include <cstdint>
#include <functional>

extern "C" {
struct sh2_SensorValue_t;
struct sh2_SensorEvent_t;
struct sh2_AsyncEvent_t;
struct sh2_Hal_t;
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
using SensorCallback = std::function<void(const SensorEvent &)>;

/**
 * @class BNO085
 * @brief High level driver for the BNO08x IMU.
 *
 * The driver wraps the vendor SH-2 C API and exposes a simple C++ interface.
 * A platform specific transport implementing ::IBNO085Transport must be
 * supplied to actually read and write bytes on the bus.
 */
class BNO085 {
public:
  /**
   * @brief Construct the driver with an optional transport instance.
   * @param transport Transport used for communication. May be nullptr and
   *        supplied later to begin().
   */
  explicit BNO085(IBNO085Transport *transport = nullptr);

  /** Initialize the sensor using the transport passed in the constructor. */
  bool begin();
  /** Initialize the sensor with the specified transport. */
  bool begin(IBNO085Transport *transport);

  /**
   * @brief Enable periodic reporting for a sensor.
   * @param sensor     Which sensor to enable.
   * @param intervalMs Desired report interval in milliseconds.
   * @param sensitivity Change sensitivity for on-change sensors.
   */
  bool enableSensor(BNO085Sensor sensor, uint32_t intervalMs,
                    float sensitivity = 0.0f);
  /** Disable reporting for a sensor. */
  bool disableSensor(BNO085Sensor sensor);

  /** Register a callback invoked for every received event. */
  void setCallback(SensorCallback cb);

  /** Check if new data is available for a sensor. */
  bool hasNewData(BNO085Sensor sensor) const;
  /** Return the most recent event for a sensor. */
  SensorEvent getLatest(BNO085Sensor sensor) const;

  /** Pump the SH-2 service loop. Call this as often as possible. */
  void update();

  /** Retrieve the last error code returned by the SH-2 driver. */
  int getLastError() const { return lastError; }

  /**
   * @brief Toggle the sensor's hardware reset line if available.
   *
   * Drives RSTN low for the specified time then releases it. Platforms not
   * providing the pin may leave the implementation empty.
   */
  void hardwareReset(uint32_t lowMs = 2);

  /** Set the BOOTN pin level (used to enter DFU). */
  void setBootPin(bool state);
  /** Control the WAKE pin in SPI mode. */
  void setWakePin(bool state);

  /** Select the host interface by driving PS pins. */
  void selectInterface(BNO085Interface iface);

private:
  /**
   * @brief Internal wrapper converting ::IBNO085Transport to the SH-2 HAL.
   */
  struct TransportHal {
    sh2_Hal_t *asHal() { return &hal; }
    sh2_Hal_t hal;                        ///< SH-2 HAL structure
    IBNO085Transport *transport{nullptr}; ///< User provided transport
  } halWrapper{};

  /// @name SH-2 HAL callbacks
  /// @{
  static int halOpen(sh2_Hal_t *self);
  static void halClose(sh2_Hal_t *self);
  static int halRead(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t);
  static int halWrite(sh2_Hal_t *self, uint8_t *buf, unsigned len);
  static uint32_t halGetTimeUs(sh2_Hal_t *self);
  /// @}

  /// C trampoline for sensor callbacks
  static void sensorC(void *cookie, sh2_SensorEvent_t *event);
  /// C trampoline for async callbacks
  static void asyncC(void *cookie, sh2_AsyncEvent_t *event);

  void handleSensorEvent(const sh2_SensorEvent_t *event);
  void handleAsyncEvent(const sh2_AsyncEvent_t *event);
  bool configure(BNO085Sensor sensor, uint32_t intervalUs, float sensitivity,
                 uint32_t batchUs = 0);

  IBNO085Transport *io{nullptr};
  SensorCallback callback{};
  int lastError{0};
  bool initialized{false};

  std::array<sh2_SensorValue_t, 0x2B> latest{};
  std::array<bool, 0x2B> newFlag{};
  std::array<uint32_t, 0x2B> lastInterval{};
  std::array<float, 0x2B> lastSensitivity{};
};
