#ifndef BNO085_HEADER_INCLUDED
#include "bno08x.hpp"
#endif

/**
 * @file bno08x.cpp
 * @brief Template method implementations for the BNO085 driver class.
 *
 * @details
 * This file is **not** compiled directly. It is `#include`d from
 * `bno08x.hpp` with a header guard so the compiler can instantiate the
 * template methods for any user-provided CommType.
 *
 * The file is organised into four sections:
 * 1. **SH-2 Mode** -- Sensor Hub 2 protocol (Begin, Update, sensor config)
 * 2. **Pin Control** -- HardwareReset, SetBootPin, etc.
 * 3. **RVC Mode** -- UART frame parser and callback dispatch
 * 4. **DFU** -- Device Firmware Update protocol with CRC + retry
 *
 * @author  Nebiyu Tadesse
 * @date    2025
 * @copyright HardFOC -- GNU GPL v3.0
 */

#include <algorithm>

// ============================================================================
/// @name SH-2 Mode Implementation
/// @brief Full sensor hub protocol over I2C, SPI, or UART.
// ============================================================================

/**
 * @brief Initialise the sensor hub in SH-2 mode.
 *
 * Opens the communication bus, wires the SH-2 C library HAL callbacks to the
 * CRTP CommInterface, opens the SH-2 session, sends a reinitialize command,
 * and registers the internal sensor and async event handlers.
 *
 * @pre  io_.GetInterfaceType() must NOT be BNO085Interface::UARTRVC.
 * @post initialized_ == true on success.
 * @return true on success, false on failure or incompatible transport.
 */
template <typename CommType>
bool BNO085<CommType>::Begin() noexcept {
  if (io_.GetInterfaceType() == BNO085Interface::UARTRVC)
    return false;

  last_error_ = 0;
  if (!io_.Open()) {
    last_error_ = -1;
    return false;
  }

  halWrapper_.comm = &io_;
  halWrapper_.hal.open = halOpen;
  halWrapper_.hal.close = halClose;
  halWrapper_.hal.read = halRead;
  halWrapper_.hal.write = halWrite;
  halWrapper_.hal.getTimeUs = halGetTimeUs;

  int status = sh2_open(halWrapper_.asHal(), asyncCallback, this);
  if (status != SH2_OK) {
    last_error_ = status;
    return false;
  }
  sh2_reinitialize();
  sh2_setSensorCallback(sensorCallback, this);

  new_flag_.fill(false);
  last_interval_.fill(0);
  last_sensitivity_.fill(0);
  initialized_ = true;
  return true;
}

/**
 * @brief Enable periodic reporting for a sensor.
 *
 * Converts the millisecond interval to microseconds and forwards to the
 * SH-2 configuration API. The interval and sensitivity are cached so they
 * can be automatically re-applied after a sensor reset.
 */
template <typename CommType>
bool BNO085<CommType>::EnableSensor(BNO085Sensor sensor, uint32_t interval_ms,
                                    float sensitivity) noexcept {
  if (!initialized_)
    return false;
  uint32_t interval_us = interval_ms * 1000;
  if (!configure(sensor, interval_us, sensitivity, 0))
    return false;
  last_interval_[static_cast<uint8_t>(sensor)] = interval_us;
  last_sensitivity_[static_cast<uint8_t>(sensor)] = sensitivity;
  return true;
}

/** @brief Disable reporting for a sensor by setting its interval to zero. */
template <typename CommType>
bool BNO085<CommType>::DisableSensor(BNO085Sensor sensor) noexcept {
  if (!initialized_)
    return false;
  return configure(sensor, 0, 0, 0);
}

/** @brief Register a callback invoked for every received SH-2 sensor event. */
template <typename CommType>
void BNO085<CommType>::SetCallback(SensorCallback cb) noexcept {
  callback_ = cb;
}

/** @brief Register a callback invoked for every decoded RVC frame. */
template <typename CommType>
void BNO085<CommType>::SetRvcCallback(RvcCallback cb) noexcept {
  rvc_cb_ = cb;
}

/** @brief Check the new-data flag for a sensor. */
template <typename CommType>
bool BNO085<CommType>::HasNewData(BNO085Sensor sensor) const {
  return new_flag_[static_cast<uint8_t>(sensor)];
}

/**
 * @brief Retrieve the most recent event for a sensor.
 *
 * Constructs a SensorEvent from the cached sh2_SensorValue_t, mapping the
 * SH-2 union fields to the appropriate high-level struct members based on
 * the sensor type.
 */
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
    if (val.un.tapDetector.flags & TAPDET_X)
      out.tap.direction = (val.un.tapDetector.flags & TAPDET_X_POS) ? 0 : 1;
    else if (val.un.tapDetector.flags & TAPDET_Y)
      out.tap.direction = (val.un.tapDetector.flags & TAPDET_Y_POS) ? 2 : 3;
    else if (val.un.tapDetector.flags & TAPDET_Z)
      out.tap.direction = (val.un.tapDetector.flags & TAPDET_Z_POS) ? 4 : 5;
    else
      out.tap.direction = 0;
    out.detected = val.un.tapDetector.flags & (TAPDET_X | TAPDET_Y | TAPDET_Z);
    break;
  default:
    break;
  }
  return out;
}

/** @brief Service the SH-2 protocol. Invokes sensor and async callbacks. */
template <typename CommType>
void BNO085<CommType>::Update() noexcept {
  if (initialized_) {
    sh2_service();
  }
}

// ---- SH-2 HAL callback bridges (CRTP CommInterface -> C sh2_Hal_t) ---------

/// @private Bridge: sh2_Hal_t::open -> CommInterface::Open().
template <typename CommType>
int BNO085<CommType>::halOpen(sh2_Hal_t* self) {
  auto* t = reinterpret_cast<CommHal*>(self);
  return t->comm->Open() ? SH2_OK : SH2_ERR;
}

/// @private Bridge: sh2_Hal_t::close -> CommInterface::Close().
template <typename CommType>
void BNO085<CommType>::halClose(sh2_Hal_t* self) {
  reinterpret_cast<CommHal*>(self)->comm->Close();
}

/// @private Bridge: sh2_Hal_t::read -> CommInterface::Read() + GetTimeUs().
template <typename CommType>
int BNO085<CommType>::halRead(sh2_Hal_t* self, uint8_t* buf, unsigned len, uint32_t* t) {
  auto* th = reinterpret_cast<CommHal*>(self);
  int ret = th->comm->Read(buf, len);
  *t = th->comm->GetTimeUs();
  return ret;
}

/// @private Bridge: sh2_Hal_t::write -> CommInterface::Write().
template <typename CommType>
int BNO085<CommType>::halWrite(sh2_Hal_t* self, uint8_t* buf, unsigned len) {
  return reinterpret_cast<CommHal*>(self)->comm->Write(buf, len);
}

/// @private Bridge: sh2_Hal_t::getTimeUs -> CommInterface::GetTimeUs().
template <typename CommType>
uint32_t BNO085<CommType>::halGetTimeUs(sh2_Hal_t* self) {
  return reinterpret_cast<CommHal*>(self)->comm->GetTimeUs();
}

// ---- SH-2 event callback trampolines --------------------------------------

/// @private C trampoline: routes sh2_SensorEvent_t to handleSensorEvent().
template <typename CommType>
void BNO085<CommType>::sensorCallback(void* cookie, sh2_SensorEvent_t* event) {
  static_cast<BNO085<CommType>*>(cookie)->handleSensorEvent(event);
}

/// @private C trampoline: routes sh2_AsyncEvent_t to handleAsyncEvent().
template <typename CommType>
void BNO085<CommType>::asyncCallback(void* cookie, sh2_AsyncEvent_t* event) {
  static_cast<BNO085<CommType>*>(cookie)->handleAsyncEvent(event);
}

/**
 * @brief Decode a raw SH-2 sensor event and update the cache.
 *
 * Stores the decoded value in latest_[], sets the new-data flag, and
 * optionally invokes the user callback.
 */
template <typename CommType>
void BNO085<CommType>::handleSensorEvent(const sh2_SensorEvent_t* event) noexcept {
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

/**
 * @brief Handle asynchronous SH-2 events.
 *
 * On SH2_RESET, re-enables all previously configured sensors using the
 * cached interval and sensitivity values.
 */
template <typename CommType>
void BNO085<CommType>::handleAsyncEvent(const sh2_AsyncEvent_t* event) noexcept {
  if (event->eventId == SH2_RESET) {
    for (uint8_t id = 0; id < last_interval_.size(); ++id) {
      if (last_interval_[id])
        configure(static_cast<BNO085Sensor>(id), last_interval_[id], last_sensitivity_[id], 0);
    }
  }
}

/**
 * @brief Send a sensor configuration command to the SH-2 library.
 *
 * @param[in] sensor       Sensor to configure.
 * @param[in] interval_us Report interval in microseconds (0 = disable).
 * @param[in] sensitivity  Change sensitivity threshold.
 * @param[in] batch_us    Batch interval in microseconds (0 = no batching).
 * @return true on success, false on failure (error stored in last_error_).
 */
template <typename CommType>
bool BNO085<CommType>::configure(BNO085Sensor sensor, uint32_t interval_us, float sensitivity,
                                 uint32_t batch_us) noexcept {
  sh2_SensorConfig_t cfg{};
  cfg.reportInterval_us = interval_us;
  cfg.batchInterval_us = batch_us;
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

// ============================================================================
/// @name Pin Control
/// @brief Drive BNO08x hardware control pins via the CommInterface.
// ============================================================================

/**
 * @brief Assert RSTN for @p lowMs, then release and wait for boot.
 *
 * The 50 ms post-release delay allows the sensor to complete its boot
 * sequence before any I2C/SPI/UART transactions.
 */
template <typename CommType>
void BNO085<CommType>::HardwareReset(uint32_t lowMs) noexcept {
  io_.SetReset(true); // Drive RSTN LOW (assert)
  io_.Delay(lowMs);
  io_.SetReset(false); // Drive RSTN HIGH (release)
  io_.Delay(50);       // Wait for sensor boot
}

/** @brief Forward BOOTN control to the CommInterface. */
template <typename CommType>
void BNO085<CommType>::SetBootPin(bool state) noexcept {
  io_.SetBoot(state);
}

/** @brief Forward WAKE control to the CommInterface (SPI only). */
template <typename CommType>
void BNO085<CommType>::SetWakePin(bool state) noexcept {
  io_.SetWake(state);
}

/**
 * @brief Drive PS0/PS1 pins to select the host interface.
 *
 * PS pins are sampled at reset. This is only useful if the pins are
 * connected to controllable GPIOs.
 */
template <typename CommType>
void BNO085<CommType>::SelectInterface(BNO085Interface iface) noexcept {
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

// ============================================================================
/// @name RVC Mode Implementation
/// @brief Reads raw UART bytes via io_.Read() and parses 19-byte RVC frames.
///
/// ### RVC Frame Format (19 bytes)
/// | Byte(s) | Field              | Units           |
/// |---------|--------------------|-----------------|
/// | 0-1     | Header (0xAA 0xAA) | --              |
/// | 2       | Sequence index     | --              |
/// | 3-4     | Yaw (LE int16)     | 0.01 degrees    |
/// | 5-6     | Pitch              | 0.01 degrees    |
/// | 7-8     | Roll               | 0.01 degrees    |
/// | 9-10    | Accel X            | 0.001 g         |
/// | 11-12   | Accel Y            | 0.001 g         |
/// | 13-14   | Accel Z            | 0.001 g         |
/// | 15      | Motion intent      | enum            |
/// | 16      | Motion request     | enum            |
/// | 17      | Reserved           | --              |
/// | 18      | Checksum           | sum(bytes 2..17)|
// ============================================================================

/**
 * @brief Open the UART transport and start the RVC frame parser.
 * @pre  io_.GetInterfaceType() must be BNO085Interface::UARTRVC.
 * @return true on success, false on failure or incompatible transport.
 */
template <typename CommType>
bool BNO085<CommType>::BeginRvc() noexcept {
  if (io_.GetInterfaceType() != BNO085Interface::UARTRVC)
    return false;
  if (!io_.Open())
    return false;
  rvc_frame_len_ = 0;
  rvc_active_ = true;
  return true;
}

/**
 * @brief Read all available UART bytes and parse RVC frames.
 *
 * Each byte is fed into a 19-byte sliding window. When a valid frame is
 * detected (0xAA 0xAA header + matching checksum), the fields are extracted,
 * decoded to floating-point, and dispatched to the RVC callback.
 */
template <typename CommType>
void BNO085<CommType>::ServiceRvc() noexcept {
  if (!rvc_active_)
    return;
  uint8_t c;
  while (io_.Read(&c, 1) == 1) {
    rvcProcessByte(c);
  }
}

/** @brief Close the UART transport and deactivate the RVC parser. */
template <typename CommType>
void BNO085<CommType>::CloseRvc() noexcept {
  if (rvc_active_) {
    io_.Close();
    rvc_active_ = false;
    rvc_frame_len_ = 0;
  }
}

/**
 * @brief Feed one byte into the RVC frame accumulator.
 *
 * Uses a sliding window approach: once the buffer is full (19 bytes), each
 * new byte shifts the window left by one. When the buffer contains a valid
 * header (0xAA 0xAA) and the checksum matches, the frame is decoded and
 * the callback is invoked.
 *
 * @param[in] c  The incoming UART byte.
 */
template <typename CommType>
void BNO085<CommType>::rvcProcessByte(uint8_t c) noexcept {
  if (rvc_frame_len_ == RVC_FRAME_LEN_) {
    std::memmove(rvc_frame_, rvc_frame_ + 1, RVC_FRAME_LEN_ - 1);
    rvc_frame_[RVC_FRAME_LEN_ - 1] = c;
  } else {
    rvc_frame_[rvc_frame_len_++] = c;
  }

  if (rvc_frame_len_ == RVC_FRAME_LEN_ && rvc_frame_[0] == 0xAA && rvc_frame_[1] == 0xAA) {
    uint8_t check = 0;
    for (int i = 2; i < RVC_FRAME_LEN_ - 1; ++i)
      check += rvc_frame_[i];

    if (check == rvc_frame_[RVC_FRAME_LEN_ - 1]) {
      RvcSensorEvent event{};
      event.timestamp_uS = io_.GetTimeUs();
      event.index = rvc_frame_[2];
      event.yaw = static_cast<int16_t>((rvc_frame_[4] << 8) | rvc_frame_[3]);
      event.pitch = static_cast<int16_t>((rvc_frame_[6] << 8) | rvc_frame_[5]);
      event.roll = static_cast<int16_t>((rvc_frame_[8] << 8) | rvc_frame_[7]);
      event.acc_x = static_cast<int16_t>((rvc_frame_[10] << 8) | rvc_frame_[9]);
      event.acc_y = static_cast<int16_t>((rvc_frame_[12] << 8) | rvc_frame_[11]);
      event.acc_z = static_cast<int16_t>((rvc_frame_[14] << 8) | rvc_frame_[13]);
      event.mi = rvc_frame_[15];
      event.mr = rvc_frame_[16];

      if (rvc_cb_) {
        RvcSensorValue val;
        decodeRvc(&val, &event);
        rvc_cb_(val);
      }
      rvc_frame_len_ = 0;
    }
  }
}

/**
 * @brief Convert raw RVC fixed-point values to floating-point SI units.
 *
 * @param[out] out  Decoded values in degrees and g.
 * @param[in]  in   Raw frame values in 0.01-degree and 0.001-g units.
 */
template <typename CommType>
void BNO085<CommType>::decodeRvc(RvcSensorValue* out, const RvcSensorEvent* in) noexcept {
  out->index = in->index;
  out->yaw_deg = 0.01f * in->yaw;
  out->pitch_deg = 0.01f * in->pitch;
  out->roll_deg = 0.01f * in->roll;
  out->acc_x_g = 0.001f * in->acc_x;
  out->acc_y_g = 0.001f * in->acc_y;
  out->acc_z_g = 0.001f * in->acc_z;
  out->mi = in->mi;
  out->mr = in->mr;
  out->timestamp_uS = in->timestamp_uS;
}

// ============================================================================
/// @name DFU Implementation
/// @brief BNO08x bootloader firmware update protocol.
///
/// The DFU sequence:
/// 1. Validate firmware image (format "BNO_V1", valid part number, length).
/// 2. Open transport (same CommInterface used for SH-2).
/// 3. Send application size (4 bytes big-endian + CRC-16).
/// 4. Send packet size (1 byte + CRC-16).
/// 5. Send firmware data in packets (each with CRC-16 + ACK/NAK retry).
/// 6. Wait for flash write completion, then close.
///
/// CRC-16 uses the CCITT polynomial 0x1021 with initial value 0xFFFF.
// ============================================================================

/// @name DFU Protocol Constants
/// @{
static constexpr uint8_t DFU_ACK = 's';                 ///< Expected ACK byte from bootloader.
static constexpr uint32_t DFU_MAX_PACKET_LEN = 64;      ///< Maximum DFU packet payload size.
static constexpr uint32_t DFU_MAX_ATTEMPTS = 5;         ///< Retry count per packet.
static constexpr uint32_t DFU_DELAY_POST_US = 10000;    ///< Post-DFU flash write delay (us).
static constexpr uint32_t DFU_SEND_TIMEOUT_US = 100000; ///< Per-packet I/O timeout (us).
/// @}

/** @brief Write a 32-bit value in big-endian byte order. */
template <typename CommType>
void BNO085<CommType>::dfuWrite32be(uint8_t* buf, uint32_t value) noexcept {
  *buf++ = (value >> 24) & 0xFF;
  *buf++ = (value >> 16) & 0xFF;
  *buf++ = (value >> 8) & 0xFF;
  *buf++ = (value >> 0) & 0xFF;
}

/** @brief Compute and append CRC-16-CCITT to a packet. */
template <typename CommType>
void BNO085<CommType>::dfuAppendCrc(uint8_t* packet, uint8_t len) noexcept {
  uint16_t crc = 0xFFFF;
  for (int n = 0; n < len; n++) {
    uint16_t x = static_cast<uint16_t>(packet[n]) << 8;
    for (int i = 0; i < 8; i++) {
      if ((crc ^ x) & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc = crc << 1;
      x <<= 1;
    }
  }
  packet[len] = (crc >> 8) & 0xFF;
  packet[len + 1] = crc & 0xFF;
}

/**
 * @brief Send a DFU packet with ACK/NAK retry logic.
 *
 * Writes @p p_data, then reads back a 1-byte ACK ('s'). Retries up to
 * DFU_MAX_ATTEMPTS times on timeout or NAK.
 *
 * @return SH2_OK on success, or a negative SH-2 error code.
 */
template <typename CommType>
int BNO085<CommType>::dfuSend(uint8_t* dfu_buff, uint8_t* p_data, uint32_t len) noexcept {
  unsigned int retries = 0;
  int status = SH2_OK;
  uint8_t ack = 0;
  bool got_ack = false;
  uint32_t t;
  sh2_Hal_t* hal = halWrapper_.asHal();

  while (!got_ack && (retries < DFU_MAX_ATTEMPTS)) {
    uint32_t now = hal->getTimeUs(hal);
    uint32_t start = now;
    status = 0;
    while ((status == 0) && ((now - start) < DFU_SEND_TIMEOUT_US)) {
      status = hal->write(hal, p_data, len);
      now = hal->getTimeUs(hal);
    }
    if (status == 0)
      status = SH2_ERR_TIMEOUT;
    if (status > 0) {
      status = 0;
      while ((status == 0) && ((now - start) < DFU_SEND_TIMEOUT_US)) {
        status = hal->read(hal, &ack, 1, &t);
        now = hal->getTimeUs(hal);
      }
      if (status == 0)
        status = SH2_ERR_TIMEOUT;
    }
    if (status > 0) {
      if (ack == DFU_ACK) {
        got_ack = true;
        status = SH2_OK;
      } else {
        got_ack = false;
        status = SH2_ERR_HUB;
      }
    }
    if (!got_ack)
      retries++;
  }
  return (status >= 0) ? SH2_OK : status;
}

/** @brief Send the 4-byte application size (big-endian) + CRC. */
template <typename CommType>
int BNO085<CommType>::dfuSendAppSize(uint8_t* dfu_buff, uint32_t app_size) noexcept {
  dfuWrite32be(dfu_buff, app_size);
  dfuAppendCrc(dfu_buff, 4);
  return dfuSend(dfu_buff, dfu_buff, 6);
}

/** @brief Send the 1-byte packet size + CRC. */
template <typename CommType>
int BNO085<CommType>::dfuSendPktSize(uint8_t* dfu_buff, uint8_t packet_len) noexcept {
  dfu_buff[0] = packet_len;
  dfuAppendCrc(dfu_buff, 1);
  return dfuSend(dfu_buff, dfu_buff, 3);
}

/** @brief Send one firmware data packet + CRC. */
template <typename CommType>
int BNO085<CommType>::dfuSendPkt(uint8_t* dfu_buff, uint8_t* p_data, uint32_t len) noexcept {
  std::memcpy(dfu_buff, p_data, len);
  dfuAppendCrc(dfu_buff, len);
  return dfuSend(dfu_buff, dfu_buff, len + 2);
}

/**
 * @brief Execute the full DFU firmware transfer.
 *
 * Validates the firmware image (format, part number, length), opens the
 * bootloader transport, sends the application size, packet size, and all
 * firmware data packets with CRC and ACK/NAK retry, then waits for flash
 * writes to complete.
 *
 * @pre  io_.GetInterfaceType() must NOT be UARTRVC.
 * @param[in] fw  Firmware image (HcBin_t interface).
 * @return SH2_OK on success, or a negative SH-2 error code.
 */
template <typename CommType>
int BNO085<CommType>::Dfu(const HcBin_t& fw) noexcept {
  if (io_.GetInterfaceType() == BNO085Interface::UARTRVC)
    return SH2_ERR;

  int rc, status = SH2_OK;
  uint32_t app_len = 0;
  uint8_t packet_len = 0;
  uint32_t offset = 0;
  const char* s = nullptr;
  uint8_t dfu_buff[DFU_MAX_PACKET_LEN + 2];
  sh2_Hal_t* hal = halWrapper_.asHal();

  rc = fw.open();
  if (rc != 0) {
    status = SH2_ERR;
    goto dfu_end;
  }

  s = fw.getMeta("FW-Format");
  if (!s || std::strcmp(s, "BNO_V1") != 0) {
    status = SH2_ERR_BAD_PARAM;
    goto dfu_close;
  }

  s = fw.getMeta("SW-Part-Number");
  if (!s) {
    status = SH2_ERR_BAD_PARAM;
    goto dfu_close;
  }
  if (std::strcmp(s, "1000-3608") != 0 && std::strcmp(s, "1000-3676") != 0 &&
      std::strcmp(s, "1000-4148") != 0 && std::strcmp(s, "1000-4563") != 0) {
    status = SH2_ERR_BAD_PARAM;
    goto dfu_close;
  }

  app_len = fw.getAppLen();
  if (app_len < 1024) {
    status = SH2_ERR_BAD_PARAM;
    goto dfu_close;
  }

  packet_len = fw.getPacketLen();
  if (packet_len == 0 || packet_len > DFU_MAX_PACKET_LEN)
    packet_len = DFU_MAX_PACKET_LEN;

  status = hal->open(hal);
  if (status != SH2_OK)
    goto dfu_close;

  status = dfuSendAppSize(dfu_buff, app_len);
  if (status != SH2_OK)
    goto dfu_close;
  status = dfuSendPktSize(dfu_buff, packet_len);
  if (status != SH2_OK)
    goto dfu_close;

  offset = 0;
  while (offset < app_len) {
    uint32_t to_send = app_len - offset;
    if (to_send > packet_len)
      to_send = packet_len;
    status = fw.getAppData(dfu_buff, offset, to_send);
    if (status != SH2_OK)
      goto dfu_close;
    status = dfuSendPkt(dfu_buff, dfu_buff, to_send);
    if (status != SH2_OK)
      goto dfu_close;
    offset += to_send;
  }

dfu_close:
  fw.close();
  if (status == SH2_OK) {
    uint32_t now = hal->getTimeUs(hal), start = now;
    while ((now - start) < DFU_DELAY_POST_US)
      now = hal->getTimeUs(hal);
  }
  hal->close(hal);

dfu_end:
  return status;
}
