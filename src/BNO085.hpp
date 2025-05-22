/**
 * @file BNO085.hpp
 * @brief High-level C++ interface for the BNO085 IMU sensor using the SH-2 sensor hub library.
 *
 * This file provides the BNO085 class definition, which includes methods for initializing
 * the sensor, configuring sensor features, registering callbacks for sensor events, and
 * retrieving sensor data. It also defines data structures and enumerations used by the
 * BNO085 class, such as sensor types, vector and quaternion representations, and event
 * data structures.
 */

// BNO085.h - High-level interface for the BNO085 sensor (header file)
#ifndef BNO085_H
#define BNO085_H

#include <cstdint>
#include <functional>
#include <array>
#include "BNO085_Transport.hpp"

// Forward-declare SH2 types (to avoid requiring include in header if not desired)
extern "C" {
    struct sh2_SensorEvent_t;    // raw sensor event (from SH2)
    struct sh2_SensorValue_t;    // decoded sensor value (union of data types)
}

/**
 * @enum BNO085Sensor
 * @brief Sensor feature identifiers for BNO085 (matches SH2 sensor IDs).
 *
 * These identifiers are used to specify which sensor feature to configure or read data from.
 * They mirror the SH2 sensor IDs defined in the SH-2 Reference documentation.
 */
enum class BNO085Sensor {
    Accelerometer             = 0x01,
    Gyroscope                 = 0x02,  // calibrated gyro
    Magnetometer              = 0x03,  // calibrated magnetometer
    LinearAcceleration        = 0x04,
    RotationVector            = 0x05,
    Gravity                   = 0x06,
    GyroUncalibrated          = 0x07,
    GameRotationVector        = 0x08,
    GeomagneticRotationVector = 0x09,
    Pressure                  = 0x0A,
    AmbientLight              = 0x0B,
    Humidity                  = 0x0C,
    Proximity                 = 0x0D,
    Temperature               = 0x0E,
    MagneticFieldUncalibrated = 0x0F,
    TapDetector               = 0x10,
    StepCounter               = 0x11,
    SignificantMotion         = 0x12,
    StabilityClassifier       = 0x13,
    RawAccelerometer          = 0x14,
    RawGyroscope              = 0x15,
    RawMagnetometer           = 0x16,
    // 0x17 reserved
    StepDetector              = 0x18,
    ShakeDetector             = 0x19,
    FlipDetector              = 0x1A,
    PickupDetector            = 0x1B,
    StabilityDetector         = 0x1C,
    // 0x1D reserved
    PersonalActivityClassifier= 0x1E,
    SleepDetector             = 0x1F,
    TiltDetector              = 0x20,
    PocketDetector            = 0x21,
    CircleDetector            = 0x22,
    HeartRateMonitor          = 0x23,
    // 0x24-0x27 reserved (internal or not used)
    ARVRStabilizedRV          = 0x28,
    ARVRStabilizedGameRV      = 0x29,
    GyroIntegratedRV          = 0x2A
};

/**
 * @struct Vector3
 * @brief Represents a 3D vector with an accuracy indicator.
 *
 * Used for representing 3D directional data from sensors like accelerometers and
 * gyroscopes, along with an accuracy value indicating the reliability of the data.
 */
struct Vector3 {
    float x;                   /**< X component of the vector */
    float y;                   /**< Y component of the vector */
    float z;                   /**< Z component of the vector */
    uint8_t accuracy;          /**< Accuracy of the vector data (0-3) */
};

/**
 * @struct Quaternion
 * @brief Represents a quaternion for rotation data with accuracy.
 *
 * Quaternions are used to represent 3D orientations and rotations, avoiding issues
 * like gimbal lock. This structure also includes an accuracy value for the rotation data.
 */
struct Quaternion {
    float w;                   /**< Scalar component of the quaternion */
    float x;                   /**< X component of the quaternion */
    float y;                   /**< Y component of the quaternion */
    float z;                   /**< Z component of the quaternion */
    uint8_t accuracy;          /**< Accuracy of the quaternion data (0-3) */
};

/**
 * @enum ActivityType
 * @brief Classification types returned by the PersonalActivityClassifier sensor.
 *
 * This enum defines the various activity types that the PersonalActivityClassifier
 * can detect, such as walking, running, or stationary. The exact mapping of values
 * to activity types is defined in the SH-2 Reference documentation.
 */
enum class ActivityType {
    Unknown = 0,
    OnFoot  = 1,
    Stationary = 2,
    // ... (other categories as defined in SH-2 Reference)
};

/**
 * @struct TapEvent
 * @brief Details of a tap detection event from the TapDetector sensor.
 *
 * Contains a flag for double tap and the tap direction axis.
 */
struct TapEvent {
    bool doubleTap;            /**< Flag indicating if the tap was a double tap */
    uint8_t direction;         /**< Direction of the tap (1 = X, 2 = Y, 3 = Z) */
};

/**
 * @struct SensorEvent
 * @brief Unified container holding data for any sensor event.
 *
 * This structure is used to deliver sensor data to the user regardless of the
 * specific sensor type. It includes a timestamp, sensor type, and a union of
 * possible data fields depending on the sensor feature that generated the event.
 */
struct SensorEvent {
    BNO085Sensor sensor;       /**< Type of sensor generating the event */
    uint64_t timestamp;        /**< Event timestamp in microseconds */
    // Union of possible data:
    Vector3 vector;            /**< 3D vector data for accel/gyro/mag etc. */
    Quaternion rotation;       /**< Quaternion data for rotation vectors */
    uint32_t stepCount;        /**< Step count for the StepCounter sensor */
    ActivityType activity;     /**< Activity classification for PersonalActivityClassifier */
    TapEvent tap;              /**< Tap event data for TapDetector sensor */
    bool detected;             /**< Generic flag for boolean detection sensors */
};

/**
 * @typedef SensorCallback
 * @brief Function signature for receiving asynchronous SensorEvent callbacks.
 *
 * This typedef defines the function pointer type for callback functions that
 * will be called with SensorEvent data. Users of the BNO085 class can define
 * their own callback functions matching this signature to process sensor events.
 * @param event Reference to the SensorEvent data.
 */
using SensorCallback = std::function<void(const SensorEvent&)>;

/**
 * @class BNO085
 * @brief High-level driver for configuring, enabling, and retrieving data from the BNO085 sensor.
 *
 * The BNO085 class provides methods to initialize the sensor, configure its various
 * features (sensors), enable or disable sensors, and register callbacks to receive
 * sensor data asynchronously. It also includes methods for polling sensor data and
 * handling errors. The class is designed to abstract away the details of the SH-2
 * sensor hub library, providing a simpler interface for users.
 */
class BNO085 {
public:
    /** @brief Constructor, initializes internal state. */
    BNO085();
    /** @brief Destructor, closes transport if open. */
    ~BNO085();

    /**
     * @brief Initializes communication with the sensor and SH2 library.
     *
     * This method must be called before any other methods to initialize the sensor
     * and establish communication with the SH-2 library. It configures the transport
     * interface (I2C, SPI, or UART) used to communicate with the sensor.
     *
     * @param transport Pointer to an instance of IBNO085Transport configured for the platform bus.
     * @return true if initialization succeeds, false otherwise.
     */
    bool init(IBNO085Transport* transport);

    /**
     * @brief Enables a specific sensor feature with report interval and optional sensitivity.
     *
     * This method configures the specified sensor feature (e.g., accelerometer, gyroscope)
     * to start generating events at the given interval. The sensitivity parameter can be
     * used to set a threshold for change-based events.
     *
     * @param sensor Sensor type to enable.
     * @param interval_ms Reporting interval in milliseconds.
     * @param change_sensitivity Sensitivity threshold for change-based events (0 for none).
     * @return true if sensor feature is successfully enabled.
     */
    bool enableSensor(BNO085Sensor sensor, uint32_t interval_ms, float change_sensitivity = 0.0f);

    /**
     * @brief Disables a previously enabled sensor feature.
     *
     * This method stops the specified sensor feature from generating events. It can be
     * used to conserve power or disable sensors that are not needed.
     *
     * @param sensor Sensor type to disable.
     * @return true if sensor feature is successfully disabled.
     */
    bool disableSensor(BNO085Sensor sensor);

    /**
     * @brief Registers a callback to receive all sensor events as they occur.
     *
     * This method sets a global callback function that will be called with each new
     * sensor event. The callback function must match the SensorCallback typedef.
     *
     * @param callback Function to call with each SensorEvent.
     */
    void setSensorCallback(SensorCallback callback);

    /**
     * @brief Checks if new data is available for a specific sensor since last retrieval.
     *
     * This method can be used to poll the status of a sensor and check if new data
     * is available to be read. It returns true if there is new data since the last
     * call to getLatestData for the specified sensor.
     *
     * @param sensor Sensor type to query.
     * @return true if new data is available, false otherwise.
     */
    bool hasNewData(BNO085Sensor sensor) const;

    /**
     * @brief Retrieves the latest data for a specific sensor.
     *
     * This method returns the most recent data available for the specified sensor.
     * The data is returned in a SensorEvent struct, which includes the sensor type,
     * timestamp, and the relevant data fields (e.g., vector, rotation, step count).
     *
     * @param sensor Sensor type to get data for.
     * @return SensorEvent struct containing the most recent data.
     */
    SensorEvent getLatestData(BNO085Sensor sensor);

    /**
     * @brief Processes incoming SH2 packets and dispatches sensor events.
     *
     * This method should be called periodically to read and process incoming data
     * from the SH-2 event queue. It extracts sensor events from SH2 packets and
     * calls the appropriate callback functions registered by the user.
     *
     * @param maxPackets Maximum packets to handle this call (-1 for all available).
     */
    void update(int maxPackets = -1);

    /**
     * @brief Retrieves the last error code from transport or SH2 operations.
     *
     * This method returns the most recent error code encountered by the transport
     * layer or the SH-2 library. A return value of 0 indicates no error.
     *
     * @return Error code (0 means no error).
     */
    int getLastError() const { return lastError; }

    /**
     * @brief Performs a hardware reset of the sensor if transport supports reset line control.
     *
     * This method triggers a hardware reset of the BNO085 sensor. It is optional and
     * depends on the transport implementation whether the reset line is controlled
     * by the IBNO085Transport interface.
     */
    void hardwareReset();

private:
    IBNO085Transport* io;         // transport interface (I2C/SPI/UART)
    SensorCallback userCallback;  // user-provided callback for events (if set)
    int lastError;                // store last error code
    bool initialized;             // whether init() completed successfully

    // Internal buffers and state for SH2
    static const size_t MAX_PACKET_SIZE = 256;         // max bytes in an SHTP packet (per datasheet, typically < 256)
    std::array<uint8_t, MAX_PACKET_SIZE> rxBuffer;     // buffer for incoming data
    // Array to store latest value for each sensor (indexed by sensor ID value)
    std::array<sh2_SensorValue_t, 0x2B> latestValues;  // 0x2A is max ID, so size 0x2B to index by ID directly
    std::array<bool, 0x2B> newDataFlags;               // flags to indicate new data since last getLatestData
    std::array<bool, 0x2B> enabledSensors;             // track which sensors have been enabled

    // Callback handlers (static, for C linkage with SH2 library)
    static void sensorEventCallback(void* cookie, sh2_SensorEvent_t* event);
    static void asyncEventCallback(void* cookie, sh2_AsyncEvent_t* event);
    // Internal methods
    void handleSensorEvent(const sh2_SensorEvent_t* event);
    void handleAsyncEvent(const sh2_AsyncEvent_t* event);
    bool configureSensorFeature(BNO085Sensor sensor, uint32_t interval_us, float change_sensitivity, uint32_t batch_us);
    BNO085(const BNO085&) = delete;
    BNO085& operator=(const BNO085&) = delete;
};

#endif // BNO085_H
