#pragma once
#include "IDfuTransport.hpp"

/**
 * @brief Adapter converting an existing sh2_Hal_t implementation to
 *        the IDfuTransport interface.
 */
class HalTransport : public IDfuTransport {
public:
  explicit HalTransport(sh2_Hal_t *hal) : hal_(hal) {}

  int open() override { return hal_->open(hal_); }
  void close() override { hal_->close(hal_); }
  int read(uint8_t *data, unsigned len, uint32_t *timestamp) override {
    return hal_->read(hal_, data, len, timestamp);
  }
  int write(const uint8_t *data, unsigned len) override {
    // sh2 HAL uses non-const pointer
    return hal_->write(hal_, const_cast<uint8_t *>(data), len);
  }
  uint32_t getTimeUs() override { return hal_->getTimeUs(hal_); }
  sh2_Hal_t *nativeHal() override { return hal_; }

private:
  sh2_Hal_t *hal_;
};
