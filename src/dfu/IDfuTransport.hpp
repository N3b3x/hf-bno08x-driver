#pragma once

/**
 * @file IDfuTransport.hpp
 * @brief CRTP-based hardware abstraction interface for DFU transport.
 *
 * Platform-specific implementations should inherit from this template
 * with themselves as the template parameter, providing compile-time
 * polymorphism with zero virtual call overhead.
 *
 * Example:
 * @code
 * class MyDfuTransport : public DfuTransportInterface<MyDfuTransport> {
 * public:
 *   int Open() noexcept { ... }
 *   void Close() noexcept { ... }
 *   int Read(uint8_t* data, unsigned len, uint32_t* timestamp) noexcept { ... }
 *   int Write(const uint8_t* data, unsigned len) noexcept { ... }
 *   uint32_t GetTimeUs() noexcept { ... }
 *   sh2_Hal_t* NativeHal() noexcept { ... }
 * };
 * @endcode
 */

#include <cstdint>

extern "C" {
#include "sh2/sh2_hal.h"
}

/**
 * @brief CRTP-based template interface for DFU hardware transport.
 *
 * @tparam Derived The derived class type (CRTP pattern).
 */
template <typename Derived>
class DfuTransportInterface {
public:
  /** Open the transport interface. */
  int Open() noexcept {
    return static_cast<Derived*>(this)->Open();
  }

  /** Close the transport interface. */
  void Close() noexcept {
    static_cast<Derived*>(this)->Close();
  }

  /**
   * Read bytes from the device.
   * @param data Buffer to fill
   * @param len  Number of bytes to read
   * @param timestamp Optional timestamp from the transport
   * @return Number of bytes read or negative error code
   */
  int Read(uint8_t* data, unsigned len, uint32_t* timestamp) noexcept {
    return static_cast<Derived*>(this)->Read(data, len, timestamp);
  }

  /**
   * Write bytes to the device.
   * @param data Buffer of data to send
   * @param len  Number of bytes to send
   * @return Number of bytes written or negative error code
   */
  int Write(const uint8_t* data, unsigned len) noexcept {
    return static_cast<Derived*>(this)->Write(data, len);
  }

  /** Return current time in microseconds. */
  uint32_t GetTimeUs() noexcept {
    return static_cast<Derived*>(this)->GetTimeUs();
  }

  /**
   * Retrieve the underlying C HAL pointer used by the vendor
   * SH-2 library. Implementations should return a pointer to
   * a sh2_Hal_t structure that dispatches to this object.
   */
  sh2_Hal_t* NativeHal() noexcept {
    return static_cast<Derived*>(this)->NativeHal();
  }

protected:
  DfuTransportInterface() = default;
  ~DfuTransportInterface() = default;

  // Prevent copying
  DfuTransportInterface(const DfuTransportInterface&) = delete;
  DfuTransportInterface& operator=(const DfuTransportInterface&) = delete;

  // Allow moving
  DfuTransportInterface(DfuTransportInterface&&) noexcept = default;
  DfuTransportInterface& operator=(DfuTransportInterface&&) noexcept = default;
};
