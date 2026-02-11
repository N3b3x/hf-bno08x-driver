#pragma once
#include "IDfuTransport.hpp"

/**
 * @brief Adapter converting an existing sh2_Hal_t implementation to
 *        the DfuTransportInterface using CRTP.
 */
class HalTransport : public DfuTransportInterface<HalTransport> {
public:
  explicit HalTransport(sh2_Hal_t* hal) noexcept : hal_(hal) {}

  int Open() noexcept {
    return hal_->open(hal_);
  }
  void Close() noexcept {
    hal_->close(hal_);
  }
  int Read(uint8_t* data, unsigned len, uint32_t* timestamp) noexcept {
    return hal_->read(hal_, data, len, timestamp);
  }
  int Write(const uint8_t* data, unsigned len) noexcept {
    // sh2 HAL uses non-const pointer
    return hal_->write(hal_, const_cast<uint8_t*>(data), len);
  }
  uint32_t GetTimeUs() noexcept {
    return hal_->getTimeUs(hal_);
  }
  sh2_Hal_t* NativeHal() noexcept {
    return hal_;
  }

private:
  sh2_Hal_t* hal_;
};
