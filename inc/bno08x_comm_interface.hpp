#pragma once

/**
 * @file bno08x_comm_interface.hpp
 * @brief CRTP-based communication interface for communicating with the BNO085.
 *
 * This header defines the hardware-agnostic communication interface using the
 * Curiously Recurring Template Pattern (CRTP) for compile-time polymorphism.
 * Applications must implement this interface to adapt the driver to the
 * platform specific I/O mechanism (I²C, SPI, UART...).  The SH-2 library
 * calls back into these methods whenever it needs to exchange data with the
 * sensor.
 *
 * Benefits of CRTP:
 * - Compile-time polymorphism (no virtual function overhead)
 * - Static dispatch instead of dynamic dispatch
 * - Better optimization opportunities for the compiler
 *
 * Example usage:
 * @code
 * class MyTransport : public bno08x::CommInterface<MyTransport> {
 * public:
 *   bool open() { ... }
 *   void close() { ... }
 *   int write(const uint8_t* data, uint32_t length) { ... }
 *   int read(uint8_t* data, uint32_t length) { ... }
 *   // ... implement other methods
 * };
 * @endcode
 */
#include <cstdint>

namespace bno08x {

/**
 * @brief CRTP-based template interface for BNO085 communication
 *
 * This template class provides a hardware-agnostic interface for BNO085
 * communication using the CRTP pattern. Platform-specific implementations
 * should inherit from this template with themselves as the template parameter.
 *
 * @tparam Derived The derived class type (CRTP pattern)
 */
template <typename Derived>
class CommInterface {
public:
  /**
   * @brief Opens the communication bus.
   * @return true if successful, false otherwise.
   */
  bool Open() noexcept {
    return static_cast<Derived*>(this)->Open();
  }

  /**
   * @brief Closes the communication bus.
   */
  void Close() noexcept {
    static_cast<Derived*>(this)->Close();
  }

  /**
   * @brief Writes data to the sensor.
   * @param data Pointer to buffer containing data to write.
   * @param length Number of bytes to write.
   * @return Number of bytes written, or negative on error.
   */
  int Write(const uint8_t* data, uint32_t length) noexcept {
    return static_cast<Derived*>(this)->Write(data, length);
  }

  /**
   * @brief Reads data from the sensor.
   * @param data Pointer to buffer to receive data.
   * @param length Number of bytes to read.
   * @return Number of bytes read, 0 if no data, or negative on error.
   */
  int Read(uint8_t* data, uint32_t length) noexcept {
    return static_cast<Derived*>(this)->Read(data, length);
  }

  /**
   * @brief Checks if new data is available.
   * @return true if data is available or if not implemented, always true.
   */
  bool DataAvailable() noexcept {
    return static_cast<Derived*>(this)->DataAvailable();
  }

  /**
   * @brief Delays execution for a specified time.
   * @param ms Delay duration in milliseconds.
   */
  void Delay(uint32_t ms) noexcept {
    static_cast<Derived*>(this)->Delay(ms);
  }

  /**
   * @brief Get current time in microseconds.
   *
   * The SH-2 library requires a monotonic time source for timestamping.
   * The communication interface implementation must provide this in microseconds.
   */
  uint32_t GetTimeUs() noexcept {
    return static_cast<Derived*>(this)->GetTimeUs();
  }

  /**
   * @brief Control the hardware reset (RSTN) pin.
   *
   * The default implementation does nothing. Platforms that connect the
   * sensor's RSTN pin can override this to assert or release reset.
   */
  void SetReset(bool state) noexcept {
    static_cast<Derived*>(this)->SetReset(state);
  }

  /**
   * @brief Control the BOOTN pin used to enter DFU mode.
   *
   * Driving BOOTN low during a reset places the device in the bootloader.
   * Implementations may leave this empty if the pin is not wired.
   */
  void SetBoot(bool state) noexcept {
    static_cast<Derived*>(this)->SetBoot(state);
  }

  /**
   * @brief Control the WAKE pin (SPI mode only).
   *
   * When using SPI, pulling WAKE low brings the device out of suspend. Not
   * all designs expose this pin, so the default does nothing.
   */
  void SetWake(bool state) noexcept {
    static_cast<Derived*>(this)->SetWake(state);
  }

  /**
   * @brief Drive protocol-select pin PS0.
   *
   * PS0 is sampled during reset to choose the active host interface. Some
   * boards may expose this pin for dynamic control.
   */
  void SetPS0(bool state) noexcept {
    static_cast<Derived*>(this)->SetPS0(state);
  }

  /**
   * @brief Drive protocol-select pin PS1.
   *
   * Together with PS0 this determines whether I²C, UART or SPI is used.
   * Implementations may leave this empty if the pins are hard wired.
   */
  void SetPS1(bool state) noexcept {
    static_cast<Derived*>(this)->SetPS1(state);
  }

protected:
  /**
   * @brief Protected constructor to prevent direct instantiation
   */
  CommInterface() = default;

  // Prevent copying
  CommInterface(const CommInterface&) = delete;
  CommInterface& operator=(const CommInterface&) = delete;

  // Allow moving
  CommInterface(CommInterface&&) noexcept = default;
  CommInterface& operator=(CommInterface&&) noexcept = default;

  /**
   * @brief Protected destructor
   * @note Derived classes can have public destructors
   */
  ~CommInterface() = default;
};

} // namespace bno08x
