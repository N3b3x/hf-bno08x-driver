/**
 * @file Esp32Bno08xBus.hpp
 * @brief ESP32 I2C transport implementation for BNO08x driver
 *
 * This file provides the ESP32-specific implementation of the IBNO085Transport
 * interface for communicating with BNO08x IMU sensors over I2C.
 *
 * @author N3b3x
 * @date 2025
 * @version 1.0.0
 */

#pragma once

#include "../../../inc/BNO085_Transport.hpp"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstdint>

/**
 * @class Esp32Bno08xBus
 * @brief ESP32 I2C transport implementation for BNO08x driver
 *
 * This class implements the IBNO085Transport interface using ESP-IDF's I2C driver.
 * It supports configurable I2C pins, address, and interrupt pin.
 */
class Esp32Bno08xBus : public IBNO085Transport {
public:
  /**
   * @brief I2C configuration structure
   */
  struct I2CConfig {
    i2c_port_t port = I2C_NUM_0;      ///< I2C port number
    gpio_num_t sda_pin = GPIO_NUM_21; ///< SDA pin (default GPIO21)
    gpio_num_t scl_pin = GPIO_NUM_22; ///< SCL pin (default GPIO22)
    uint32_t frequency = 400000;      ///< I2C frequency in Hz (default 400kHz)
    uint8_t device_address = 0x4A;    ///< I2C device address (7-bit, default 0x4A)
    gpio_num_t int_pin = GPIO_NUM_NC; ///< Interrupt pin (GPIO_NUM_NC if not used)
    gpio_num_t rst_pin = GPIO_NUM_NC; ///< Reset pin (GPIO_NUM_NC if not used)
  };

  /**
   * @brief Constructor with default I2C configuration
   */
  Esp32Bno08xBus() : Esp32Bno08xBus(I2CConfig{}) {}

  /**
   * @brief Constructor with custom I2C configuration
   * @param config I2C configuration parameters
   */
  explicit Esp32Bno08xBus(const I2CConfig& config);

  /**
   * @brief Destructor - cleans up I2C resources
   */
  ~Esp32Bno08xBus() override;

  /**
   * @brief Open the I2C bus and initialize communication
   * @return true if successful, false otherwise
   */
  bool open() override;

  /**
   * @brief Close the I2C bus and deinitialize
   */
  void close() override;

  /**
   * @brief Write data to the BNO08x sensor
   * @param data Pointer to data buffer
   * @param length Number of bytes to write
   * @return Number of bytes written, or negative on error
   */
  int write(const uint8_t* data, uint32_t length) override;

  /**
   * @brief Read data from the BNO08x sensor
   * @param data Pointer to receive buffer
   * @param length Number of bytes to read
   * @return Number of bytes read, 0 if no data, or negative on error
   */
  int read(uint8_t* data, uint32_t length) override;

  /**
   * @brief Check if new data is available
   * @return true if data is available (via interrupt pin or always true)
   */
  bool dataAvailable() override;

  /**
   * @brief Delay execution for specified time
   * @param ms Delay duration in milliseconds
   */
  void delay(uint32_t ms) override;

  /**
   * @brief Get current time in microseconds
   * @return Current time in microseconds
   */
  uint32_t getTimeUs() override;

  /**
   * @brief Control the hardware reset (RSTN) pin
   * @param state true to assert reset (low), false to release (high)
   */
  void setReset(bool state) override;

  /**
   * @brief Get the current I2C configuration
   * @return Current I2C configuration
   */
  const I2CConfig& getConfig() const noexcept {
    return config_;
  }

  /**
   * @brief Check if I2C bus is initialized
   * @return true if initialized, false otherwise
   */
  bool isInitialized() const noexcept {
    return initialized_;
  }

private:
  I2CConfig config_;                                   ///< I2C configuration
  bool initialized_ = false;                           ///< Initialization state
  static constexpr const char* TAG = "Esp32Bno08xBus"; ///< Logging tag

  /**
   * @brief Initialize I2C bus
   * @return true if successful, false otherwise
   */
  bool initializeI2C();

  /**
   * @brief Initialize GPIO pins (interrupt, reset)
   * @return true if successful, false otherwise
   */
  bool initializeGPIO();
};
