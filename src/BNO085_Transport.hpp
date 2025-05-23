#pragma once

/**
 * @file BNO085_Transport.hpp
 * @brief Abstract transport interface for communicating with the BNO085.
 *
 * Applications must implement this interface to adapt the driver to the
 * platform specific I/O mechanism (I²C, SPI, UART...).  The SH-2 library
 * calls back into these methods whenever it needs to exchange data with the
 * sensor.
 */
#include <cstdint>

/**
 * @class IBNO085Transport
 * @brief Abstract interface for BNO085 communication transport (I2C, SPI,
 * etc.).
 */
class IBNO085Transport {
public:
  /**
   * @brief Virtual destructor.
   */
  virtual ~IBNO085Transport() = default;
  /**
   * @brief Opens the communication bus.
   * @return true if successful, false otherwise.
   */
  virtual bool open() = 0;
  /**
   * @brief Closes the communication bus.
   */
  virtual void close() = 0;
  /**
   * @brief Writes data to the sensor.
   * @param data Pointer to buffer containing data to write.
   * @param length Number of bytes to write.
   * @return Number of bytes written, or negative on error.
   */
  virtual int write(const uint8_t *data, uint32_t length) = 0;
  /**
   * @brief Reads data from the sensor.
   * @param data Pointer to buffer to receive data.
   * @param length Number of bytes to read.
   * @return Number of bytes read, 0 if no data, or negative on error.
   */
  virtual int read(uint8_t *data, uint32_t length) = 0;
  /**
   * @brief Checks if new data is available.
   * @return true if data is available or if not implemented, always true.
   */
  virtual bool dataAvailable() { return true; }
  /**
   * @brief Delays execution for a specified time.
   * @param ms Delay duration in milliseconds.
   */
  virtual void delay(uint32_t ms) = 0;

  /**
   * @brief Get current time in microseconds.
   *
   * The SH-2 library requires a monotonic time source for timestamping.
   * The transport implementation must provide this in microseconds.
   */
  virtual uint32_t getTimeUs() = 0;
};
