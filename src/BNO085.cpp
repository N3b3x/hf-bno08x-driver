#include "BNO085.hpp"

#include <algorithm>
#include <cstdio>

extern "C" {
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
}

using namespace std;

BNO085::BNO085(IBNO085Transport* transport) : io(transport) {}

bool BNO085::begin() {
    if (!io) return false;
    return begin(io);
}

bool BNO085::begin(IBNO085Transport* transport) {
    io = transport;
    if (!io) return false;
    lastError = 0;
    if (!io->open()) {
        lastError = -1;
        return false;
    }

    halWrapper.transport = io;
    halWrapper.hal.open = halOpen;
    halWrapper.hal.close = halClose;
    halWrapper.hal.read = halRead;
    halWrapper.hal.write = halWrite;
    halWrapper.hal.getTimeUs = halGetTimeUs;

    sh2_initialize(asyncC, this);
    int status = sh2_open(halWrapper.asHal(), asyncC, this);
    if (status != SH2_OK) {
        lastError = status;
        return false;
    }

    sh2_setSensorCallback(sensorC, this);

    newFlag.fill(false);
    lastInterval.fill(0);
    lastSensitivity.fill(0);
    initialized = true;
    return true;
}

bool BNO085::enableSensor(BNO085Sensor sensor, uint32_t intervalMs, float sensitivity) {
    if (!initialized) return false;
    uint32_t intervalUs = intervalMs * 1000;
    if (!configure(sensor, intervalUs, sensitivity, 0)) return false;
    lastInterval[static_cast<uint8_t>(sensor)] = intervalUs;
    lastSensitivity[static_cast<uint8_t>(sensor)] = sensitivity;
    return true;
}

bool BNO085::disableSensor(BNO085Sensor sensor) {
    if (!initialized) return false;
    return configure(sensor, 0, 0, 0);
}

void BNO085::setCallback(SensorCallback cb) { callback = cb; }

bool BNO085::hasNewData(BNO085Sensor sensor) const {
    return newFlag[static_cast<uint8_t>(sensor)];
}

SensorEvent BNO085::getLatest(BNO085Sensor sensor) const {
    SensorEvent out{};
    out.sensor = sensor;
    auto id = static_cast<uint8_t>(sensor);
    const auto& val = latest[id];
    out.timestamp = val.timestamp;
    switch(sensor) {
    case BNO085Sensor::Accelerometer:
    case BNO085Sensor::LinearAcceleration:
    case BNO085Sensor::Gravity:
        out.vector.x = val.un.accelerometer.x;
        out.vector.y = val.un.accelerometer.y;
        out.vector.z = val.un.accelerometer.z;
        out.vector.accuracy = val.un.accelerometer.accuracy;
        break;
    case BNO085Sensor::Gyroscope:
        out.vector.x = val.un.gyroscope.x;
        out.vector.y = val.un.gyroscope.y;
        out.vector.z = val.un.gyroscope.z;
        out.vector.accuracy = val.un.gyroscope.accuracy;
        break;
    case BNO085Sensor::Magnetometer:
        out.vector.x = val.un.magneticField.x;
        out.vector.y = val.un.magneticField.y;
        out.vector.z = val.un.magneticField.z;
        out.vector.accuracy = val.un.magneticField.accuracy;
        break;
    case BNO085Sensor::RotationVector:
    case BNO085Sensor::GameRotationVector:
    case BNO085Sensor::GeomagneticRotationVector:
    case BNO085Sensor::ARVRStabilizedRV:
    case BNO085Sensor::ARVRStabilizedGameRV:
        out.rotation.w = val.un.rotationVector.real;
        out.rotation.x = val.un.rotationVector.i;
        out.rotation.y = val.un.rotationVector.j;
        out.rotation.z = val.un.rotationVector.k;
        out.rotation.accuracy = val.un.rotationVector.accuracy;
        break;
    case BNO085Sensor::GyroIntegratedRV:
        out.rotation.w = val.un.gyroIntegratedRV.real;
        out.rotation.x = val.un.gyroIntegratedRV.i;
        out.rotation.y = val.un.gyroIntegratedRV.j;
        out.rotation.z = val.un.gyroIntegratedRV.k;
        break;
    case BNO085Sensor::StepCounter:
        out.stepCount = val.un.stepCounter.numSteps;
        break;
    case BNO085Sensor::TapDetector:
        out.tap.doubleTap = val.un.tapDetector.tapDetected && val.un.tapDetector.tapCount == 2;
        out.tap.direction = val.un.tapDetector.tapDirection;
        out.detected = val.un.tapDetector.tapDetected;
        break;
    default:
        break;
    }
    return out;
}

void BNO085::update() {
    if (initialized) {
        sh2_service();
    }
}

int BNO085::halOpen(sh2_Hal_t* self) {
    auto* t = reinterpret_cast<TransportHal*>(self);
    return t->transport->open() ? SH2_OK : SH2_ERR;
}

void BNO085::halClose(sh2_Hal_t* self) {
    auto* t = reinterpret_cast<TransportHal*>(self);
    t->transport->close();
}

int BNO085::halRead(sh2_Hal_t* self, uint8_t* buf, unsigned len, uint32_t* t) {
    auto* th = reinterpret_cast<TransportHal*>(self);
    int ret = th->transport->read(buf, len);
    *t = th->transport->getTimeUs();
    return ret;
}

int BNO085::halWrite(sh2_Hal_t* self, uint8_t* buf, unsigned len) {
    auto* th = reinterpret_cast<TransportHal*>(self);
    return th->transport->write(buf, len);
}

uint32_t BNO085::halGetTimeUs(sh2_Hal_t* self) {
    auto* th = reinterpret_cast<TransportHal*>(self);
    return th->transport->getTimeUs();
}

void BNO085::sensorC(void* cookie, sh2_SensorEvent_t* event) {
    static_cast<BNO085*>(cookie)->handleSensorEvent(event);
}

void BNO085::asyncC(void* cookie, sh2_AsyncEvent_t* event) {
    static_cast<BNO085*>(cookie)->handleAsyncEvent(event);
}

void BNO085::handleSensorEvent(const sh2_SensorEvent_t* event) {
    sh2_SensorValue_t value;
    sh2_decodeSensorEvent(&value, event);
    uint8_t id = value.sensorId;
    if (id >= latest.size()) return;
    latest[id] = value;
    newFlag[id] = true;
    if (callback) {
        callback(getLatest(static_cast<BNO085Sensor>(id)));
        newFlag[id] = false;
    }
}

void BNO085::handleAsyncEvent(const sh2_AsyncEvent_t* event) {
    if (event->eventId == SH2_RESET) {
        for (uint8_t id = 0; id < lastInterval.size(); ++id) {
            if (lastInterval[id]) {
                configure(static_cast<BNO085Sensor>(id), lastInterval[id], lastSensitivity[id], 0);
            }
        }
    }
}

bool BNO085::configure(BNO085Sensor sensor, uint32_t intervalUs, float sensitivity, uint32_t batchUs) {
    sh2_SensorConfig_t cfg{};
    cfg.sensorId = static_cast<uint8_t>(sensor);
    cfg.reportInterval_us = intervalUs;
    cfg.batchInterval_us = batchUs;
    cfg.sensorSpecific = 0;
    cfg.changeSensitivity = static_cast<uint32_t>(sensitivity);
    int status = sh2_setSensorConfig(cfg.sensorId, &cfg);
    if (status != SH2_OK) {
        lastError = status;
        return false;
    }
    return true;
}
