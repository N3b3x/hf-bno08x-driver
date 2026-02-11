/**
 * @file esp32_bno08x_bus.cpp
 * @brief ESP32 I2C communication interface implementation for BNO08x driver
 *
 * Uses ESP-IDF I2C master driver (same API as hf-pcal95555-driver / hf-pca9685-driver)
 * so the same I2C port (GPIO4 SDA, GPIO5 SCL) can be shared across drivers.
 */

#include "esp32_bno08x_bus.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

Esp32Bno08xBus::Esp32Bno08xBus(const I2CConfig& config) : config_(config) {}

Esp32Bno08xBus::~Esp32Bno08xBus() {
  Close();
}

bool Esp32Bno08xBus::Open() noexcept {
  if (initialized_) {
    ESP_LOGW(TAG, "I2C bus already initialized");
    return true;
  }

  if (!initializeGPIO()) {
    ESP_LOGE(TAG, "Failed to initialize GPIO pins");
    return false;
  }

  if (!initializeI2C()) {
    ESP_LOGE(TAG, "Failed to initialize I2C bus");
    return false;
  }

  initialized_ = true;
  ESP_LOGI(TAG, "I2C bus initialized successfully");
  return true;
}

void Esp32Bno08xBus::Close() noexcept {
  if (!initialized_) {
    return;
  }

  if (bus_handle_ != nullptr) {
    i2c_del_master_bus(bus_handle_);
    bus_handle_ = nullptr;
  }

  if (config_.int_pin != GPIO_NUM_NC) {
    gpio_reset_pin(config_.int_pin);
  }
  if (config_.rst_pin != GPIO_NUM_NC) {
    gpio_reset_pin(config_.rst_pin);
  }

  initialized_ = false;
  ESP_LOGI(TAG, "I2C bus closed");
}

int Esp32Bno08xBus::Write(const uint8_t* data, uint32_t length) noexcept {
  if (!initialized_ || bus_handle_ == nullptr) {
    ESP_LOGE(TAG, "I2C bus not initialized");
    return -1;
  }

  i2c_master_dev_handle_t dev_handle = nullptr;
  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = config_.device_address,
      .scl_speed_hz = config_.frequency,
      .scl_wait_us = 0,
      .flags = {},
  };

  esp_err_t err = i2c_master_bus_add_device(bus_handle_, &dev_config, &dev_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add device 0x%02X: %s", config_.device_address, esp_err_to_name(err));
    return -1;
  }

  err = i2c_master_transmit(dev_handle, data, length, pdMS_TO_TICKS(1000));
  i2c_master_bus_rm_device(dev_handle);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(err));
    return -1;
  }

  return static_cast<int>(length);
}

int Esp32Bno08xBus::Read(uint8_t* data, uint32_t length) noexcept {
  if (!initialized_ || bus_handle_ == nullptr) {
    ESP_LOGE(TAG, "I2C bus not initialized");
    return -1;
  }

  i2c_master_dev_handle_t dev_handle = nullptr;
  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = config_.device_address,
      .scl_speed_hz = config_.frequency,
      .scl_wait_us = 0,
      .flags = {},
  };

  esp_err_t err = i2c_master_bus_add_device(bus_handle_, &dev_config, &dev_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add device 0x%02X: %s", config_.device_address, esp_err_to_name(err));
    return -1;
  }

  err = i2c_master_receive(dev_handle, data, length, pdMS_TO_TICKS(1000));
  i2c_master_bus_rm_device(dev_handle);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(err));
    return -1;
  }

  return static_cast<int>(length);
}

bool Esp32Bno08xBus::DataAvailable() noexcept {
  if (config_.int_pin == GPIO_NUM_NC) {
    // No interrupt pin configured, always return true
    return true;
  }

  // Check interrupt pin (active low)
  int level = gpio_get_level(config_.int_pin);
  return (level == 0);
}

void Esp32Bno08xBus::Delay(uint32_t ms) noexcept {
  vTaskDelay(pdMS_TO_TICKS(ms));
}

uint32_t Esp32Bno08xBus::GetTimeUs() noexcept {
  return static_cast<uint32_t>(esp_timer_get_time());
}

void Esp32Bno08xBus::SetReset(bool state) noexcept {
  if (config_.rst_pin == GPIO_NUM_NC) {
    return; // Reset pin not configured
  }

  // Reset is active low
  gpio_set_level(config_.rst_pin, state ? 0 : 1);
}

bool Esp32Bno08xBus::initializeI2C() {
  // Same I2C master bus pattern as hf-pcal95555-driver (GPIO4/5, same port)
  i2c_master_bus_config_t bus_config = {};
  bus_config.i2c_port = config_.port;
  bus_config.sda_io_num = config_.sda_pin;
  bus_config.scl_io_num = config_.scl_pin;
  bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_config.glitch_ignore_cnt = 7;
  bus_config.flags.enable_internal_pullup = true;
  bus_config.intr_priority = 0;
  bus_config.trans_queue_depth = 0;

  esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C master bus create failed: %s", esp_err_to_name(err));
    return false;
  }

  ESP_LOGI(TAG, "I2C bus configured: SDA=GPIO%d, SCL=GPIO%d, Freq=%lu Hz, Addr=0x%02X",
           config_.sda_pin, config_.scl_pin, config_.frequency, config_.device_address);

  return true;
}

bool Esp32Bno08xBus::initializeGPIO() {
  // Configure interrupt pin (input, pull-up enabled)
  if (config_.int_pin != GPIO_NUM_NC) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << config_.int_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to configure interrupt pin: %s", esp_err_to_name(err));
      return false;
    }
    ESP_LOGI(TAG, "Interrupt pin configured: GPIO%d", config_.int_pin);
  }

  // Configure reset pin (output, high by default)
  if (config_.rst_pin != GPIO_NUM_NC) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << config_.rst_pin);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to configure reset pin: %s", esp_err_to_name(err));
      return false;
    }

    // Release reset (set high)
    gpio_set_level(config_.rst_pin, 1);
    ESP_LOGI(TAG, "Reset pin configured: GPIO%d", config_.rst_pin);
  }

  return true;
}
