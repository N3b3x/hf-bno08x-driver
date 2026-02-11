/**
 * @file esp32_rvc_hal.hpp
 * @brief ESP32 RVC HAL implementation for BNO08x UART-based RVC mode
 *
 * This file provides an ESP32-specific implementation of the RvcHalInterface CRTP
 * using ESP-IDF's UART driver. It handles the RVC frame sync (0xAA 0xAA),
 * checksum validation, and GPIO reset/boot pin control.
 *
 * @note RVC mode uses UART at 115200 baud. The BNO08x must have PS0/PS1 set
 *       to RVC mode (PS1=HIGH, PS0=LOW) at reset time.
 *
 * @author Nebiyu Tadesse
 * @date 2025
 * @copyright HardFOC
 */

#pragma once

#include "../../../src/rvc/RvcHal.hpp"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include <cstring>
#include <esp_rom_sys.h>

/**
 * @class Esp32RvcHal
 * @brief ESP32 implementation of RvcHalInterface using ESP-IDF UART driver.
 *
 * Uses CRTP for zero-overhead dispatch, consistent with CommInterface.
 * Configurable UART port and GPIO pins via constructor parameters.
 * Default pins are set for ESP32-S3 development board configuration.
 */
class Esp32RvcHal : public RvcHalInterface<Esp32RvcHal> {
public:
  /**
   * @brief Construct RVC HAL with configurable pins.
   * @param port UART port number (default UART_NUM_1)
   * @param tx   UART TX GPIO (default GPIO21)
   * @param rx   UART RX GPIO (default GPIO20)
   * @param rst  Reset GPIO (default GPIO16, active-low)
   * @param boot Boot GPIO (default GPIO10)
   */
  explicit Esp32RvcHal(uart_port_t port = UART_NUM_1, gpio_num_t tx = GPIO_NUM_21,
                       gpio_num_t rx = GPIO_NUM_20, gpio_num_t rst = GPIO_NUM_16,
                       gpio_num_t boot = GPIO_NUM_10)
      : port_(port), tx_(tx), rx_(rx), rst_(rst), boot_(boot) {}

  int Open() noexcept {
    uart_config_t cfg{};
    cfg.baud_rate = 115200;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_param_config(port_, &cfg);
    uart_set_pin(port_, tx_, rx_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(port_, BUF_SIZE, 0, 0, nullptr, 0);

    // Reset the sensor into RVC mode
    gpio_set_level(boot_, 1);
    gpio_set_level(rst_, 0);
    esp_rom_delay_us(10000);
    gpio_set_level(rst_, 1);

    len_ = 0;
    ready_ = false;
    return RVC_OK;
  }

  void Close() noexcept {
    uart_driver_delete(port_);
  }

  int Read(rvc_SensorEvent_t* event) noexcept {
    uint8_t c;
    while (uart_read_bytes(port_, &c, 1, 0) == 1) {
      process(c);
    }
    if (ready_) {
      fillEvent(event);
      ready_ = false;
      len_ = 0;
      return 1;
    }
    return 0;
  }

private:
  static constexpr int BUF_SIZE = 256;    ///< UART RX buffer size
  static constexpr int FRAME_LEN = 19;    ///< RVC frame length in bytes

  uart_port_t port_;                       ///< UART port
  gpio_num_t tx_, rx_, rst_, boot_;        ///< GPIO pins
  uint8_t frame_[FRAME_LEN]{};            ///< Frame accumulation buffer
  size_t len_{0};                          ///< Current bytes in buffer
  bool ready_{false};                      ///< Complete frame available

  /// Validate RVC frame checksum (sum of bytes 2..17 == byte 18)
  static bool checksum(const uint8_t* f) {
    uint8_t check = 0;
    for (int i = 2; i < FRAME_LEN - 1; ++i)
      check += f[i];
    return check == f[FRAME_LEN - 1];
  }

  /// Process incoming byte into frame buffer
  void process(uint8_t c) {
    if (len_ == FRAME_LEN) {
      memmove(frame_, frame_ + 1, FRAME_LEN - 1);
      frame_[FRAME_LEN - 1] = c;
    } else {
      frame_[len_++] = c;
    }
    if (len_ == FRAME_LEN && frame_[0] == 0xAA && frame_[1] == 0xAA && checksum(frame_)) {
      ready_ = true;
    }
  }

  /// Extract sensor data from a validated RVC frame
  void fillEvent(rvc_SensorEvent_t* e) {
    e->timestamp_uS = esp_timer_get_time();
    e->index = frame_[2];
    e->yaw = static_cast<int16_t>((frame_[4] << 8) | frame_[3]);
    e->pitch = static_cast<int16_t>((frame_[6] << 8) | frame_[5]);
    e->roll = static_cast<int16_t>((frame_[8] << 8) | frame_[7]);
    e->acc_x = static_cast<int16_t>((frame_[10] << 8) | frame_[9]);
    e->acc_y = static_cast<int16_t>((frame_[12] << 8) | frame_[11]);
    e->acc_z = static_cast<int16_t>((frame_[14] << 8) | frame_[13]);
    e->mi = frame_[15];
    e->mr = frame_[16];
  }
};
