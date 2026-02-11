#pragma once

/**
 * @file RvcHal.hpp
 * @brief CRTP-based hardware abstraction interface for RVC mode.
 *
 * Platform-specific implementations should inherit from this template
 * with themselves as the template parameter, providing compile-time
 * polymorphism with zero virtual call overhead.
 *
 * Example:
 * @code
 * class MyRvcHal : public RvcHalInterface<MyRvcHal> {
 * public:
 *   int Open() noexcept { ... }
 *   void Close() noexcept { ... }
 *   int Read(rvc_SensorEvent_t* event) noexcept { ... }
 * };
 * @endcode
 */

#include "rvc.h"
#include <cstdint>

/**
 * @brief CRTP-based template interface for RVC hardware access.
 *
 * @tparam Derived The derived class type (CRTP pattern).
 */
template <typename Derived>
class RvcHalInterface {
public:
  /**
   * @brief Open and configure the hardware interface.
   * @return RVC_OK on success or a negative error code.
   */
  int Open() noexcept {
    return static_cast<Derived*>(this)->Open();
  }

  /**
   * @brief Close the interface and release resources.
   */
  void Close() noexcept {
    static_cast<Derived*>(this)->Close();
  }

  /**
   * @brief Read a sensor event from the device.
   *
   * Implementations should populate @p event with the next available
   * sensor data if one is ready. The method should return 1 when an
   * event was read, 0 when no event is available yet and a negative
   * value on error.
   */
  int Read(rvc_SensorEvent_t* event) noexcept {
    return static_cast<Derived*>(this)->Read(event);
  }

protected:
  RvcHalInterface() = default;
  ~RvcHalInterface() = default;

  // Prevent copying
  RvcHalInterface(const RvcHalInterface&) = delete;
  RvcHalInterface& operator=(const RvcHalInterface&) = delete;

  // Allow moving
  RvcHalInterface(RvcHalInterface&&) noexcept = default;
  RvcHalInterface& operator=(RvcHalInterface&&) noexcept = default;
};
