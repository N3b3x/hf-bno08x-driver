#ifndef HF_BNO085_HPP
#define HF_BNO085_HPP

#include <array>
#include <cstdint>
#include <functional>
#include "BNO085_Transport.hpp"

extern "C" {
    struct sh2_SensorValue_t;
    struct sh2_SensorEvent_t;
    struct sh2_AsyncEvent_t;
    struct sh2_Hal_t;
}

enum class BNO085Sensor : uint8_t {
    Accelerometer             = 0x01,
    Gyroscope                 = 0x02,
    Magnetometer              = 0x03,
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
    StepDetector              = 0x18,
    ShakeDetector             = 0x19,
    FlipDetector              = 0x1A,
    PickupDetector            = 0x1B,
    StabilityDetector         = 0x1C,
    PersonalActivityClassifier= 0x1E,
    SleepDetector             = 0x1F,
    TiltDetector              = 0x20,
    PocketDetector            = 0x21,
    CircleDetector            = 0x22,
    HeartRateMonitor          = 0x23,
    ARVRStabilizedRV          = 0x28,
    ARVRStabilizedGameRV      = 0x29,
    GyroIntegratedRV          = 0x2A
};

struct Vector3 { float x{0}; float y{0}; float z{0}; uint8_t accuracy{0}; };
struct Quaternion { float w{1}; float x{0}; float y{0}; float z{0}; uint8_t accuracy{0}; };
struct TapEvent { bool doubleTap{false}; uint8_t direction{0}; };

struct SensorEvent {
    BNO085Sensor sensor{BNO085Sensor::Accelerometer};
    uint64_t timestamp{0};
    Vector3 vector{};
    Quaternion rotation{};
    uint32_t stepCount{0};
    TapEvent tap{};
    bool detected{false};
};

using SensorCallback = std::function<void(const SensorEvent&)>;

class BNO085 {
public:
    explicit BNO085(IBNO085Transport* transport = nullptr);

    bool begin();
    bool begin(IBNO085Transport* transport);

    bool enableSensor(BNO085Sensor sensor, uint32_t intervalMs, float sensitivity = 0.0f);
    bool disableSensor(BNO085Sensor sensor);

    void setCallback(SensorCallback cb);

    bool hasNewData(BNO085Sensor sensor) const;
    SensorEvent getLatest(BNO085Sensor sensor) const;

    void update();

    int getLastError() const { return lastError; }

private:
    struct TransportHal {
        sh2_Hal_t* asHal() { return &hal; }
        sh2_Hal_t hal;
        IBNO085Transport* transport{nullptr};
    } halWrapper{};

    static int halOpen(sh2_Hal_t* self);
    static void halClose(sh2_Hal_t* self);
    static int halRead(sh2_Hal_t* self, uint8_t* buf, unsigned len, uint32_t* t);
    static int halWrite(sh2_Hal_t* self, uint8_t* buf, unsigned len);
    static uint32_t halGetTimeUs(sh2_Hal_t* self);

    static void sensorC(void* cookie, sh2_SensorEvent_t* event);
    static void asyncC(void* cookie, sh2_AsyncEvent_t* event);

    void handleSensorEvent(const sh2_SensorEvent_t* event);
    void handleAsyncEvent(const sh2_AsyncEvent_t* event);
    bool configure(BNO085Sensor sensor, uint32_t intervalUs, float sensitivity, uint32_t batchUs = 0);

    IBNO085Transport* io{nullptr};
    SensorCallback callback{};
    int lastError{0};
    bool initialized{false};

    std::array<sh2_SensorValue_t, 0x2B> latest{};
    std::array<bool, 0x2B> newFlag{};
    std::array<uint32_t, 0x2B> lastInterval{};
    std::array<float, 0x2B> lastSensitivity{};
};

#endif
