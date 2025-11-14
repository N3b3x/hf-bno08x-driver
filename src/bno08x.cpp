#ifndef BNO085_HEADER_INCLUDED
#include "bno08x.hpp"
#endif

/**
 * @file bno08x.cpp
 * @brief Template implementation of the BNO08x C++ driver.
 *
 * This file contains the template method implementations for BNO08x.
 * It is included by bno08x.hpp when BNO085_HEADER_INCLUDED is defined.
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
 * @brief Initialise using the communication interface passed to the constructor.
 */
template <typename CommType>
bool BNO085<CommType>::Begin() noexcept {
  last_error_ = 0;
  if (!io_.Open()) {
    last_error_ = -1;
    return false;
  }

  halWrapper.comm = &io_;  // Store pointer for C callbacks
  halWrapper.hal.open = halOpen;
  halWrapper.hal.close = halClose;
  halWrapper.hal.read = halRead;
  halWrapper.hal.write = halWrite;
  halWrapper.hal.getTimeUs = halGetTimeUs;

  int status = sh2_open(halWrapper.asHal(), asyncC, this);
  if (status != SH2_OK) {
    last_error_ = status;
    return false;
  }
  // Send initialize command after opening the SH-2 interface
  sh2_reinitialize();

  sh2_setSensorCallback(sensorC, this);

  new_flag_.fill(false);
  last_interval_.fill(0);
  last_sensitivity_.fill(0);
  initialized_ = true;
  return true;
}

/**
 * @brief Enable periodic reporting for a sensor.
 */
template <typename CommType>
bool BNO085<CommType>::EnableSensor(BNO085Sensor sensor, uint32_t intervalMs, float sensitivity) {
  if (!initialized_)
    return false;
  uint32_t interval_us = intervalMs * 1000;
  if (!configure(sensor, interval_us, sensitivity, 0))
    return false;
  last_interval_[static_cast<uint8_t>(sensor)] = interval_us;
  last_sensitivity_[static_cast<uint8_t>(sensor)] = sensitivity;
  return true;
}

/** Disable reporting for a sensor. */
template <typename CommType>
bool BNO085<CommType>::DisableSensor(BNO085Sensor sensor) {
  if (!initialized_)
    return false;
  return configure(sensor, 0, 0, 0);
}

/** Set a callback for incoming sensor events. */
template <typename CommType>
void BNO085<CommType>::SetCallback(SensorCallback cb) {
  callback_ = cb;
}

/** Set a callback for decoded RVC frames. */
template <typename CommType>
void BNO085<CommType>::SetRvcCallback(RvcCallback cb) {
  rvc_cb_ = cb;
}

/** Check if new data is available for a sensor. */
template <typename CommType>
bool BNO085<CommType>::HasNewData(BNO085Sensor sensor) const {
  return new_flag_[static_cast<uint8_t>(sensor)];
}

/** Retrieve the most recent event for a sensor. */
template <typename CommType>
SensorEvent BNO085<CommType>::GetLatest(BNO085Sensor sensor) const {
  SensorEvent out{};
  out.sensor = sensor;
  auto id = static_cast<uint8_t>(sensor);
  const auto& val = latest_[id];
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
template <typename CommType>
void BNO085<CommType>::Update() {
  if (initialized_) {
    sh2_service();
  }
}

/** Begin processing RVC frames. */
template <typename CommType>
bool BNO085<CommType>::BeginRvc(IRvcHal* hal) {
  rvc_.SetHal(hal);
  if (rvc_.SetCallback(rvcC, this) != RVC_OK)
    return false;
  return rvc_.Open() == RVC_OK;
}

/** Poll for RVC frames. */
template <typename CommType>
void BNO085<CommType>::ServiceRvc() {
  rvc_.Service();
}

/** Stop RVC frame processing. */
template <typename CommType>
void BNO085<CommType>::CloseRvc() {
  rvc_.Close();
}

/// @private
template <typename CommType>
int BNO085<CommType>::halOpen(sh2_Hal_t* self) {
  auto* t = reinterpret_cast<CommHal*>(self);
  return t->comm->Open() ? SH2_OK : SH2_ERR;
}

/// @private
template <typename CommType>
void BNO085<CommType>::halClose(sh2_Hal_t* self) {
  auto* t = reinterpret_cast<CommHal*>(self);
  t->comm->Close();
}

/// @private
template <typename CommType>
int BNO085<CommType>::halRead(sh2_Hal_t* self, uint8_t* buf, unsigned len, uint32_t* t) {
  auto* th = reinterpret_cast<CommHal*>(self);
  int ret = th->comm->Read(buf, len);
  *t = th->comm->GetTimeUs();
  return ret;
}

/// @private
template <typename CommType>
int BNO085<CommType>::halWrite(sh2_Hal_t* self, uint8_t* buf, unsigned len) {
  auto* th = reinterpret_cast<CommHal*>(self);
  return th->comm->Write(buf, len);
}

/// @private
template <typename CommType>
uint32_t BNO085<CommType>::halGetTimeUs(sh2_Hal_t* self) {
  auto* th = reinterpret_cast<CommHal*>(self);
  return th->comm->GetTimeUs();
}

/// @private
template <typename CommType>
void BNO085<CommType>::sensorC(void* cookie, sh2_SensorEvent_t* event) {
  static_cast<BNO085<CommType>*>(cookie)->handleSensorEvent(event);
}

/// @private
template <typename CommType>
void BNO085<CommType>::asyncC(void* cookie, sh2_AsyncEvent_t* event) {
  static_cast<BNO085<CommType>*>(cookie)->handleAsyncEvent(event);
}

/** Internal handler for decoded sensor events. */
template <typename CommType>
void BNO085<CommType>::handleSensorEvent(const sh2_SensorEvent_t* event) {
  sh2_SensorValue_t value;
  sh2_decodeSensorEvent(&value, event);
  uint8_t id = value.sensorId;
  if (id >= latest_.size())
    return;
  latest_[id] = value;
  new_flag_[id] = true;
  if (callback_) {
    callback_(GetLatest(static_cast<BNO085Sensor>(id)));
    new_flag_[id] = false;
  }
}

/** React to asynchronous sensor events (e.g. reset). */
template <typename CommType>
void BNO085<CommType>::handleAsyncEvent(const sh2_AsyncEvent_t* event) {
  if (event->eventId == SH2_RESET) {
    for (uint8_t id = 0; id < last_interval_.size(); ++id) {
      if (last_interval_[id]) {
        configure(static_cast<BNO085Sensor>(id), last_interval_[id], last_sensitivity_[id], 0);
      }
    }
  }
}

/// @private Configure a report in the SH-2 driver
template <typename CommType>
bool BNO085<CommType>::configure(BNO085Sensor sensor, uint32_t intervalUs, float sensitivity,
                       uint32_t batchUs) {
  sh2_SensorConfig_t cfg{};
  cfg.reportInterval_us = intervalUs;
  cfg.batchInterval_us = batchUs;
  cfg.sensorSpecific = 0;
  cfg.changeSensitivity = static_cast<uint16_t>(sensitivity);
  cfg.changeSensitivityEnabled = sensitivity > 0;
  int status = sh2_setSensorConfig(static_cast<sh2_SensorId_t>(sensor), &cfg);
  if (status != SH2_OK) {
    last_error_ = status;
    return false;
  }
  return true;
}

/** Toggle the hardware reset line if implemented. */
template <typename CommType>
void BNO085<CommType>::HardwareReset(uint32_t lowMs) {
  io_.SetReset(false);
  io_.Delay(lowMs);
  io_.SetReset(true);
  io_.Delay(50); // allow sensor to boot
}

/** Drive the BOOTN pin. */
template <typename CommType>
void BNO085<CommType>::SetBootPin(bool state) {
  io_.SetBoot(state);
}

/** Control the WAKE pin. */
template <typename CommType>
void BNO085<CommType>::SetWakePin(bool state) {
  io_.SetWake(state);
}

/** Select host interface using PS0/PS1. */
template <typename CommType>
void BNO085<CommType>::SelectInterface(BNO085Interface iface) {
  switch (iface) {
  case BNO085Interface::I2C:
    io_.SetPS1(false);
    io_.SetPS0(false);
    break;
  case BNO085Interface::UARTRVC:
    io_.SetPS1(true);
    io_.SetPS0(false);
    break;
  case BNO085Interface::UART:
    io_.SetPS1(false);
    io_.SetPS0(true);
    break;
  case BNO085Interface::SPI:
    io_.SetPS1(true);
    io_.SetPS0(true);
    break;
  }
}

/** Convenience wrapper to run DFU using this instance's communication interface. */
template <typename CommType>
int BNO085<CommType>::Dfu(const HcBin_t& fw) {
  HalTransport t(halWrapper.asHal());
  return ::dfu(t, fw);
}

/// @private
template <typename CommType>
void BNO085<CommType>::rvcC(void* cookie, rvc_SensorEvent_t* ev) {
  auto* self = static_cast<BNO085<CommType>*>(cookie);
  if (!self->rvc_cb_)
    return;
  rvc_SensorValue_t val;
  Rvc::Decode(&val, ev);
  self->rvc_cb_(val);
}
