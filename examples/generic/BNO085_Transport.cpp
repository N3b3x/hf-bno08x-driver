/**
 * @file BNO085_Transport.cpp
 * @brief Example I2C transport implementation for BNO085 (platform-specific pseudocode).
 */
#include "BNO085_Transport.hpp"
#include <YourMCU_I2C_Driver.h> // Pseudocode include for underlying I2C functions

/**
 * @class I2CTransport
 * @brief Concrete IBNO085Transport implementation using I2C bus.
 */
class I2CTransport : public IBNO085Transport {
public:
  /**
   * @brief Constructs I2CTransport with bus handle, device address, and optional interrupt pin.
   * @param bus Pointer to HAL I2C handle.
   * @param deviceAddress 7-bit I2C address shifted left for HAL.
   * @param intPort GPIO port for interrupt line (or nullptr if unused).
   * @param intPin GPIO pin for interrupt line.
   */
  I2CTransport(I2C_HandleTypeDef *bus, uint8_t deviceAddress, GPIO_TypeDef *intPort = nullptr,
               uint16_t intPin = 0)
      : i2c(bus), addr(deviceAddress), intPort(intPort), intPin(intPin) {}

  /**
   * @copydoc IBNO085Transport::open
   */
  bool open() override {
    // Initialize I2C peripheral if needed (if using HAL, assume it’s already init)
    // Perhaps reset BNO085 here if desired via a reset pin (not shown).
    return true; // assume success
  }

  /**
   * @copydoc IBNO085Transport::close
   */
  void close() override {
    // De-initialize or power down I2C if needed
  }

  /**
   * @copydoc IBNO085Transport::write
   */
  int write(const uint8_t *data, uint32_t length) override {
    // Write data over I2C (blocking). The BNO085 expects the first byte as part of SHTP header.
    // Pseudocode using some HAL I2C API:
    if (HAL_I2C_Master_Transmit(i2c, addr, (uint8_t *)data, length, HAL_MAX_DELAY) == HAL_OK) {
      return length;
    }
    return -1; // error
  }

  /**
   * @copydoc IBNO085Transport::read
   */
  int read(uint8_t *data, uint32_t length) override {
    // Read data over I2C. If length bytes are not available, the I2C read may return an error.
    // BNO085 allows reading the first 2 bytes (length field) then a repeated read for remaining
    // bytes.
    HAL_StatusTypeDef result = HAL_I2C_Master_Receive(i2c, addr, data, length, /*timeout*/ 10);
    if (result == HAL_OK) {
      return length;
    } else if (result == HAL_ERROR) {
      return -1; // bus error
    } else {
      return 0; // no data (NACK or timeout)
    }
  }

  /**
   * @copydoc IBNO085Transport::dataAvailable
   */
  bool dataAvailable() override {
    if (intPort == nullptr) {
      return true; // no interrupt pin, always attempt read
    }
    // If an interrupt GPIO is provided, check if it's asserted (active low H_INTN)
    return (HAL_GPIO_ReadPin(intPort, intPin) == GPIO_PIN_RESET);
  }

  /**
   * @copydoc IBNO085Transport::delay
   */
  void delay(uint32_t ms) override {
    // Use the system delay (e.g., HAL_Delay or vTaskDelay)
    HAL_Delay(ms);
  }

private:
  GPIO_TypeDef *intPort;
  uint16_t intPin;
  I2C_HandleTypeDef *i2c;
  uint8_t addr;
};
