#include "BNO085.hpp"

/**
 * @file BNO085.cpp
 * @brief Implementation of the BNO085 C++ driver.
 */

#include <algorithm>
#include <cstdio>

extern "C" {
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
}
#include "dfu/HalTransport.hpp"
#include "dfu/dfu.h"

using namespace std;

/**
 * @brief Construct the driver with an optional transport.
 */
BNO085::BNO085(IBNO085Transport *transport) : io(transport) {}

/**
 * @brief Initialise using the transport passed to the constructor.
 */
bool BNO085::begin() {
  if (!io)
    return false;
  return begin(io);
}

/**
 * @brief Initialise using the given transport instance.
 */
bool BNO085::begin(IBNO085Transport *transport) {
  io = transport;
  if (!io)
    return false;
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

  int status = sh2_open(halWrapper.asHal(), asyncC, this);
  if (status != SH2_OK) {
    lastError = status;
    return false;
  }
  // Send initialize command after opening the SH-2 interface
  sh2_reinitialize();

  sh2_setSensorCallback(sensorC, this);

  newFlag.fill(false);
  lastInterval.fill(0);
  lastSensitivity.fill(0);
  initialized = true;
  return true;
}

/**
 * @brief Enable periodic reporting for a sensor.
 */
bool BNO085::enableSensor(BNO085Sensor sensor, uint32_t intervalMs, float sensitivity) {
  if (!initialized)
    return false;
  uint32_t intervalUs = intervalMs * 1000;
  if (!configure(sensor, intervalUs, sensitivity, 0))
    return false;
  lastInterval[static_cast<uint8_t>(sensor)] = intervalUs;
  lastSensitivity[static_cast<uint8_t>(sensor)] = sensitivity;
  return true;
}

/** Disable reporting for a sensor. */
bool BNO085::disableSensor(BNO085Sensor sensor) {
  if (!initialized)
    return false;
  return configure(sensor, 0, 0, 0);
}

/** Set a callback for incoming sensor events. */
void BNO085::setCallback(SensorCallback cb) { callback = cb; }

/** Set a callback for decoded RVC frames. */
void BNO085::setRvcCallback(RvcCallback cb) { rvcCb = cb; }

/** Check if new data is available for a sensor. */
bool BNO085::hasNewData(BNO085Sensor sensor) const { return newFlag[static_cast<uint8_t>(sensor)]; }

/** Retrieve the most recent event for a sensor. */
SensorEvent BNO085::getLatest(BNO085Sensor sensor) const {
  SensorEvent out{};
  out.sensor = sensor;
  auto id = static_cast<uint8_t>(sensor);
  const auto &val = latest[id];
  out.timestamp = val.timestamp;
  uint8_t accuracy = val.status & 0x03;
  switch (sensor) {
  case BNO085Sensor::Accelerometer:
  case BNO085Sensor::LinearAcceleration:
  case BNO085Sensor::Gravity:
    out.vector.x = val.un.accelerometer.x;
    out.vector.y = val.un.accelerometer.y;
    out.vector.z = val.un.accelerometer.z;
    out.vector.accuracy = accuracy;
    break;
  case BNO085Sensor::Gyroscope:
    out.vector.x = val.un.gyroscope.x;
    out.vector.y = val.un.gyroscope.y;
    out.vector.z = val.un.gyroscope.z;
    out.vector.accuracy = accuracy;
    break;
  case BNO085Sensor::Magnetometer:
    out.vector.x = val.un.magneticField.x;
    out.vector.y = val.un.magneticField.y;
    out.vector.z = val.un.magneticField.z;
    out.vector.accuracy = accuracy;
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
    out.rotation.accuracy = accuracy;
    break;
  case BNO085Sensor::GyroIntegratedRV:
    out.rotation.w = val.un.gyroIntegratedRV.real;
    out.rotation.x = val.un.gyroIntegratedRV.i;
    out.rotation.y = val.un.gyroIntegratedRV.j;
    out.rotation.z = val.un.gyroIntegratedRV.k;
    break;
  case BNO085Sensor::StepCounter:
    out.stepCount = val.un.stepCounter.steps;
    break;
  case BNO085Sensor::TapDetector:
    out.tap.doubleTap = (val.un.tapDetector.flags & TAPDET_DOUBLE);
    if (val.un.tapDetector.flags & TAPDET_X) {
      out.tap.direction = (val.un.tapDetector.flags & TAPDET_X_POS) ? 0 : 1;
    } else if (val.un.tapDetector.flags & TAPDET_Y) {
      out.tap.direction = (val.un.tapDetector.flags & TAPDET_Y_POS) ? 2 : 3;
    } else if (val.un.tapDetector.flags & TAPDET_Z) {
      out.tap.direction = (val.un.tapDetector.flags & TAPDET_Z_POS) ? 4 : 5;
    } else {
      out.tap.direction = 0;
    }
    out.detected = val.un.tapDetector.flags & (TAPDET_X | TAPDET_Y | TAPDET_Z);
    break;
  default:
    break;
  }
  return out;
}

/** Service the SH-2 library. Call as often as possible. */
void BNO085::update() {
  if (initialized) {
    sh2_service();
  }
}

/** Begin processing RVC frames. */
bool BNO085::beginRvc(IRvcHal *hal) {
  rvc.setHal(hal);
  if (rvc.setCallback(rvcC, this) != RVC_OK)
    return false;
  return rvc.open() == RVC_OK;
}

/** Poll for RVC frames. */
void BNO085::serviceRvc() { rvc.service(); }

/** Stop RVC frame processing. */
void BNO085::closeRvc() { rvc.close(); }

/// @private
int BNO085::halOpen(sh2_Hal_t *self) {
  auto *t = reinterpret_cast<TransportHal *>(self);
  return t->transport->open() ? SH2_OK : SH2_ERR;
}

/// @private
void BNO085::halClose(sh2_Hal_t *self) {
  auto *t = reinterpret_cast<TransportHal *>(self);
  t->transport->close();
}

/// @private
int BNO085::halRead(sh2_Hal_t *self, uint8_t *buf, unsigned len, uint32_t *t) {
  auto *th = reinterpret_cast<TransportHal *>(self);
  int ret = th->transport->read(buf, len);
  *t = th->transport->getTimeUs();
  return ret;
}

/// @private
int BNO085::halWrite(sh2_Hal_t *self, uint8_t *buf, unsigned len) {
  auto *th = reinterpret_cast<TransportHal *>(self);
  return th->transport->write(buf, len);
}

/// @private
uint32_t BNO085::halGetTimeUs(sh2_Hal_t *self) {
  auto *th = reinterpret_cast<TransportHal *>(self);
  return th->transport->getTimeUs();
}

/// @private
void BNO085::sensorC(void *cookie, sh2_SensorEvent_t *event) {
  static_cast<BNO085 *>(cookie)->handleSensorEvent(event);
}

/// @private
void BNO085::asyncC(void *cookie, sh2_AsyncEvent_t *event) {
  static_cast<BNO085 *>(cookie)->handleAsyncEvent(event);
}

/** Internal handler for decoded sensor events. */
void BNO085::handleSensorEvent(const sh2_SensorEvent_t *event) {
  sh2_SensorValue_t value;
  sh2_decodeSensorEvent(&value, event);
  uint8_t id = value.sensorId;
  if (id >= latest.size())
    return;
  latest[id] = value;
  newFlag[id] = true;
  if (callback) {
    callback(getLatest(static_cast<BNO085Sensor>(id)));
    newFlag[id] = false;
  }
}

/** React to asynchronous sensor events (e.g. reset). */
void BNO085::handleAsyncEvent(const sh2_AsyncEvent_t *event) {
  if (event->eventId == SH2_RESET) {
    for (uint8_t id = 0; id < lastInterval.size(); ++id) {
      if (lastInterval[id]) {
        configure(static_cast<BNO085Sensor>(id), lastInterval[id], lastSensitivity[id], 0);
      }
    }
  }
}

/// @private Configure a report in the SH-2 driver
bool BNO085::configure(BNO085Sensor sensor, uint32_t intervalUs, float sensitivity,
                       uint32_t batchUs) {
  sh2_SensorConfig_t cfg{};
  cfg.reportInterval_us = intervalUs;
  cfg.batchInterval_us = batchUs;
  cfg.sensorSpecific = 0;
  cfg.changeSensitivity = static_cast<uint16_t>(sensitivity);
  cfg.changeSensitivityEnabled = sensitivity > 0;
  int status = sh2_setSensorConfig(static_cast<sh2_SensorId_t>(sensor), &cfg);
  if (status != SH2_OK) {
    lastError = status;
    return false;
  }
  return true;
}

/** Toggle the hardware reset line if implemented. */
void BNO085::hardwareReset(uint32_t lowMs) {
  if (!io)
    return;
  io->setReset(false);
  io->delay(lowMs);
  io->setReset(true);
  io->delay(50); // allow sensor to boot
}

/** Drive the BOOTN pin. */
void BNO085::setBootPin(bool state) {
  if (io)
    io->setBoot(state);
}

/** Control the WAKE pin. */
void BNO085::setWakePin(bool state) {
  if (io)
    io->setWake(state);
}

/** Select host interface using PS0/PS1. */
void BNO085::selectInterface(BNO085Interface iface) {
  if (!io)
    return;
  switch (iface) {
  case BNO085Interface::I2C:
    io->setPS1(false);
    io->setPS0(false);
    break;
  case BNO085Interface::UARTRVC:
    io->setPS1(true);
    io->setPS0(false);
    break;
  case BNO085Interface::UART:
    io->setPS1(false);
    io->setPS0(true);
    break;
  case BNO085Interface::SPI:
    io->setPS1(true);
    io->setPS0(true);
    break;
  }
}

/** Convenience wrapper to run DFU using this instance's transport. */
int BNO085::dfu(const HcBin_t &fw) {
  HalTransport t(halWrapper.asHal());
  return ::dfu(t, fw);
}

/// @private
void BNO085::rvcC(void *cookie, rvc_SensorEvent_t *ev) {
  auto *self = static_cast<BNO085 *>(cookie);
  if (!self->rvcCb)
    return;
  rvc_SensorValue_t val;
  Rvc::decode(&val, ev);
  self->rvcCb(val);
}
