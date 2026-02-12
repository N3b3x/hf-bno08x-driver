#pragma once

/**
 * @file bno08x_comm_interface.hpp
 * @brief CRTP-based communication interface for the BNO08x IMU family.
 *
 * @details
 * This header defines the hardware-agnostic communication interface that all
 * platform-specific transport implementations must provide. It uses the
 * **Curiously Recurring Template Pattern (CRTP)** for compile-time
 * polymorphism, giving zero virtual-call overhead on the hot path
 * (Read/Write are called thousands of times per second during SH-2 service).
 *
 * ### Supported Transport Types
 *
 * | Transport     | BNO085Interface | Supported Modes          |
 * |---------------|-----------------|--------------------------|
 * | I2C           | `I2C`           | SH-2, DFU                |
 * | SPI           | `SPI`           | SH-2, DFU                |
 * | UART (SH-2)   | `UART`          | SH-2, DFU                |
 * | UART (RVC)    | `UARTRVC`       | RVC mode only            |
 *
 * ### How to Implement
 *
 * Inherit from `bno08x::CommInterface<YourClass>` and implement all required
 * methods. The driver calls `GetInterfaceType()` to determine which mode
 * of operation is valid for the transport.
 *
 * @code
 * class MyI2CBus : public bno08x::CommInterface<MyI2CBus> {
 * public:
 *   BNO085Interface GetInterfaceType() noexcept { return BNO085Interface::I2C; }
 *   bool Open() noexcept { ... }
 *   void Close() noexcept { ... }
 *   int  Write(const uint8_t* data, uint32_t length) noexcept { ... }
 *   int  Read(uint8_t* data, uint32_t length) noexcept { ... }
 *   bool DataAvailable() noexcept { return true; }
 *   void Delay(uint32_t ms) noexcept { ... }
 *   uint32_t GetTimeUs() noexcept { ... }
 *   // Pin control (no-op if not wired):
 *   void SetReset(bool) noexcept { ... }
 *   void SetBoot(bool) noexcept {}
 *   void SetWake(bool) noexcept {}
 *   void SetPS0(bool) noexcept {}
 *   void SetPS1(bool) noexcept {}
 * };
 * @endcode
 *
 * @author  Nebiyu Tadesse
 * @date    2025
 * @copyright HardFOC -- GNU GPL v3.0
 */

#include <cstdint>

// ============================================================================
// Interface Type Enum
// ============================================================================

/**
 * @enum BNO085Interface
 * @brief Identifies the host interface type that a CommInterface provides.
 *
 * Returned by CommInterface::GetInterfaceType(). The BNO085 driver uses this
 * value to guard which operations are valid at runtime:
 * - **SH-2 mode** (Begin, Update, EnableSensor, Dfu): requires `I2C`, `SPI`, or `UART`.
 * - **RVC mode** (BeginRvc, ServiceRvc, CloseRvc): requires `UARTRVC`.
 *
 * The enum values also correspond to the BNO08x PS1/PS0 pin states that
 * select the interface at boot time.
 */
enum class BNO085Interface : uint8_t {
  I2C = 0,     ///< I2C bus (PS1=0, PS0=0). Supports SH-2 and DFU.
  UARTRVC = 1, ///< UART in RVC mode (PS1=1, PS0=0). RVC frames only.
  UART = 2,    ///< UART in SH-2 mode (PS1=0, PS0=1). Supports SH-2 and DFU.
  SPI = 3      ///< SPI bus (PS1=1, PS0=1). Supports SH-2 and DFU.
};

// ============================================================================
// CommInterface CRTP Base
// ============================================================================

namespace bno08x {

/**
 * @brief CRTP base class for BNO08x communication interfaces.
 *
 * Platform-specific implementations inherit from this template with themselves
 * as the template parameter. All method calls are resolved at compile time
 * via `static_cast<Derived*>(this)->Method()` -- no vtable, no indirection.
 *
 * @tparam Derived  The concrete implementation class (CRTP pattern).
 *
 * @note  Implementations must provide **all** listed methods. Pin control
 *        methods (SetReset, SetBoot, SetWake, SetPS0, SetPS1) may be empty
 *        no-ops if the corresponding hardware pin is not wired.
 */
template <typename Derived>
class CommInterface {
public:
  // --------------------------------------------------------------------------
  /// @name Interface Identification
  /// @{

  /**
   * @brief Report the transport type this implementation provides.
   *
   * The BNO085 driver calls this once during Begin() or BeginRvc() to
   * determine which operations to allow.
   *
   * @return  BNO085Interface enum identifying the bus protocol.
   */
  BNO085Interface GetInterfaceType() noexcept {
    return static_cast<Derived*>(this)->GetInterfaceType();
  }

  /// @}

  // --------------------------------------------------------------------------
  /// @name Bus I/O -- Required Methods
  /// @{

  /**
   * @brief Open and initialise the communication bus.
   *
   * Called by BNO085::Begin() (SH-2 mode) or BNO085::BeginRvc() (RVC mode).
   * Implementations should configure the peripheral, set up GPIO pins, and
   * prepare for data transfer.
   *
   * @return  `true` on success, `false` on failure.
   */
  bool Open() noexcept {
    return static_cast<Derived*>(this)->Open();
  }

  /**
   * @brief Close the communication bus and release resources.
   *
   * Called during driver shutdown. Implementations should de-initialise the
   * peripheral and release any allocated resources.
   */
  void Close() noexcept {
    static_cast<Derived*>(this)->Close();
  }

  /**
   * @brief Write raw bytes to the sensor.
   *
   * For I2C/SPI this sends data on the bus. For UART-RVC this is typically
   * unused (RVC mode is read-only).
   *
   * @param[in]  data    Pointer to the buffer to transmit.
   * @param[in]  length  Number of bytes to write.
   * @return  Number of bytes written, or negative on error.
   */
  int Write(const uint8_t* data, uint32_t length) noexcept {
    return static_cast<Derived*>(this)->Write(data, length);
  }

  /**
   * @brief Read raw bytes from the sensor.
   *
   * For SH-2 mode, the vendor SHTP library calls this to receive packets.
   * For RVC mode, the driver calls `Read(&byte, 1)` repeatedly to feed the
   * internal 19-byte frame parser.
   *
   * @param[out] data    Buffer to fill with received bytes.
   * @param[in]  length  Maximum number of bytes to read.
   * @return  Number of bytes actually read, 0 if no data available,
   *          or negative on error.
   */
  int Read(uint8_t* data, uint32_t length) noexcept {
    return static_cast<Derived*>(this)->Read(data, length);
  }

  /**
   * @brief Check whether the sensor has data ready.
   *
   * Typically checks the interrupt (INT) pin level. For transports without
   * an interrupt pin, return `true` unconditionally to allow polling.
   *
   * @return  `true` if data is available (or if unknown), `false` otherwise.
   */
  bool DataAvailable() noexcept {
    return static_cast<Derived*>(this)->DataAvailable();
  }

  /**
   * @brief Block for a specified duration.
   *
   * Used by the driver for reset timing and post-DFU delays.
   *
   * @param[in] ms  Delay duration in milliseconds.
   */
  void Delay(uint32_t ms) noexcept {
    static_cast<Derived*>(this)->Delay(ms);
  }

  /**
   * @brief Return the current monotonic time in microseconds.
   *
   * The vendor SH-2 library requires a microsecond time source for
   * timestamping sensor events. The value does not need to represent
   * wall-clock time -- only monotonicity matters.
   *
   * @return  Current time in microseconds.
   */
  uint32_t GetTimeUs() noexcept {
    return static_cast<Derived*>(this)->GetTimeUs();
  }

  /// @}

  // --------------------------------------------------------------------------
  /// @name Optional Pin Control
  ///
  /// These methods control dedicated GPIO pins on the BNO08x. Implementations
  /// should provide real GPIO logic if the pin is wired, or a no-op body if
  /// the pin is not connected on the board.
  /// @{

  /**
   * @brief Assert or release the hardware reset (RSTN) pin.
   *
   * RSTN is **active-low**. Pass `true` to drive the pin LOW (assert reset),
   * `false` to drive it HIGH (release reset).
   *
   * @param[in] state  `true` = assert reset (LOW), `false` = release (HIGH).
   */
  void SetReset(bool state) noexcept {
    static_cast<Derived*>(this)->SetReset(state);
  }

  /**
   * @brief Control the BOOTN pin for entering DFU bootloader mode.
   *
   * BOOTN is **active-low**. Hold it LOW during a reset pulse to enter the
   * bootloader. Not needed for normal operation.
   *
   * @param[in] state  `true` = drive LOW (enter bootloader), `false` = HIGH.
   */
  void SetBoot(bool state) noexcept {
    static_cast<Derived*>(this)->SetBoot(state);
  }

  /**
   * @brief Control the WAKE pin (SPI mode only).
   *
   * Pulling WAKE LOW brings the sensor out of suspend. Only relevant for
   * SPI transport. No-op for I2C and UART implementations.
   *
   * @param[in] state  `true` = drive LOW (wake), `false` = release.
   */
  void SetWake(bool state) noexcept {
    static_cast<Derived*>(this)->SetWake(state);
  }

  /**
   * @brief Drive the PS0 protocol-select pin.
   *
   * PS0 is sampled at reset to select the host interface. Most boards
   * hard-wire this pin, making this method a no-op.
   *
   * @param[in] state  Desired logic level for PS0.
   */
  void SetPS0(bool state) noexcept {
    static_cast<Derived*>(this)->SetPS0(state);
  }

  /**
   * @brief Drive the PS1 protocol-select pin.
   *
   * Together with PS0, PS1 determines the active interface at boot.
   *
   * @param[in] state  Desired logic level for PS1.
   */
  void SetPS1(bool state) noexcept {
    static_cast<Derived*>(this)->SetPS1(state);
  }

  /// @}

protected:
  /// @brief Protected constructor -- prevents direct instantiation.
  CommInterface() = default;
  /// @brief Protected destructor -- prevent polymorphic deletion through base.
  ~CommInterface() = default;

  CommInterface(const CommInterface&) = delete;                 ///< Non-copyable.
  CommInterface& operator=(const CommInterface&) = delete;      ///< Non-copyable.
  CommInterface(CommInterface&&) noexcept = default;            ///< Movable.
  CommInterface& operator=(CommInterface&&) noexcept = default; ///< Movable.
};

} // namespace bno08x
