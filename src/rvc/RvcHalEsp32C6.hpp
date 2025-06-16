#pragma once
#include "RvcHal.hpp"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include <cstring>
#include <esp_rom_sys.h>

/**
 * @brief Example HAL implementation for ESP32-C6 using ESP-IDF UART driver.
 */
class Esp32C6RvcHal : public IRvcHal {
public:
  explicit Esp32C6RvcHal(uart_port_t port = UART_NUM_0, gpio_num_t tx = GPIO_NUM_21,
                         gpio_num_t rx = GPIO_NUM_20, gpio_num_t rst = GPIO_NUM_9,
                         gpio_num_t boot = GPIO_NUM_10)
      : _port(port), _tx(tx), _rx(rx), _rst(rst), _boot(boot) {}

  int open() override {
    uart_config_t cfg{};
    cfg.baud_rate = 115200;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_param_config(_port, &cfg);
    uart_set_pin(_port, _tx, _rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(_port, BUF_SIZE, 0, 0, nullptr, 0);
    gpio_set_level(_boot, 1);
    gpio_set_level(_rst, 0);
    esp_rom_delay_us(10000);
    gpio_set_level(_rst, 1);
    _len = 0;
    _ready = false;
    return RVC_OK;
  }

  void close() override { uart_driver_delete(_port); }

  int read(rvc_SensorEvent_t *event) override {
    uint8_t c;
    while (uart_read_bytes(_port, &c, 1, 0) == 1) {
      process(c);
    }
    if (_ready) {
      fillEvent(event);
      _ready = false;
      _len = 0;
      return 1;
    }
    return 0;
  }

private:
  static constexpr int BUF_SIZE = 256;
  static constexpr int FRAME_LEN = 19;
  uart_port_t _port;
  gpio_num_t _tx, _rx, _rst, _boot;
  uint8_t _frame[FRAME_LEN];
  size_t _len{0};
  bool _ready{false};

  static bool checksum(const uint8_t *f) {
    uint8_t check = 0;
    for (int i = 2; i < FRAME_LEN - 1; ++i)
      check += f[i];
    return check == f[FRAME_LEN - 1];
  }

  void process(uint8_t c) {
    if (_len == FRAME_LEN) {
      memmove(_frame, _frame + 1, FRAME_LEN - 1);
      _frame[FRAME_LEN - 1] = c;
    } else {
      _frame[_len++] = c;
    }
    if (_len == FRAME_LEN && _frame[0] == 0xAA && _frame[1] == 0xAA && checksum(_frame)) {
      _ready = true;
    }
  }

  void fillEvent(rvc_SensorEvent_t *e) {
    e->timestamp_uS = esp_timer_get_time();
    e->index = _frame[2];
    e->yaw = (int16_t)((_frame[4] << 8) | _frame[3]);
    e->pitch = (int16_t)((_frame[6] << 8) | _frame[5]);
    e->roll = (int16_t)((_frame[8] << 8) | _frame[7]);
    e->acc_x = (int16_t)((_frame[10] << 8) | _frame[9]);
    e->acc_y = (int16_t)((_frame[12] << 8) | _frame[11]);
    e->acc_z = (int16_t)((_frame[14] << 8) | _frame[13]);
    e->mi = _frame[15];
    e->mr = _frame[16];
  }
};
