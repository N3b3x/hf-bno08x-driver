#pragma once
#include "HcBin.h"
#include <cstdint>
#include <cstring>

/**
 * @brief HcBin_t implementation for firmware stored in memory.
 *
 * Construct with a pointer to the binary image and its length. Optional
 * strings describe the format and part number. Pass the returned handle
 * to dfu() to stream the image to the device.
 */
class MemoryFirmware {
public:
  MemoryFirmware(const uint8_t *data, uint32_t len,
                 const char *format = "BNO_V1", const char *part = "unknown")
      : data_(data), len_(len), format_(format), part_(part) {}

  /** Obtain an HcBin handle for the DFU helpers. */
  const HcBin_t &hcbin() const {
    active_ = this;
    return impl_;
  }

private:
  static int open();
  static int close();
  static const char *getMeta(const char *key);
  static uint32_t getAppLen();
  static uint32_t getPacketLen();
  static int getAppData(uint8_t *packet, uint32_t offset, uint32_t len);

  static const HcBin_t impl_;
  static const MemoryFirmware *active_;

  const uint8_t *data_;
  uint32_t len_;
  const char *format_;
  const char *part_;
};

// --- Implementation ---------------------------------------------------------

inline int MemoryFirmware::open() { return 0; }
inline int MemoryFirmware::close() { return 0; }
inline const char *MemoryFirmware::getMeta(const char *key) {
  if (!active_)
    return nullptr;
  if (std::strcmp(key, "FW-Format") == 0)
    return active_->format_;
  if (std::strcmp(key, "SW-Part-Number") == 0)
    return active_->part_;
  return nullptr;
}
inline uint32_t MemoryFirmware::getAppLen() {
  return active_ ? active_->len_ : 0;
}
inline uint32_t MemoryFirmware::getPacketLen() { return 0; }
inline int MemoryFirmware::getAppData(uint8_t *packet, uint32_t offset,
                                      uint32_t len) {
  if (!active_ || offset + len > active_->len_)
    return -1;
  std::memcpy(packet, active_->data_ + offset, len);
  return 0;
}

inline const HcBin_t MemoryFirmware::impl_ = {
    MemoryFirmware::open,         MemoryFirmware::close,
    MemoryFirmware::getMeta,      MemoryFirmware::getAppLen,
    MemoryFirmware::getPacketLen, MemoryFirmware::getAppData};
inline const MemoryFirmware *MemoryFirmware::active_ = nullptr;
