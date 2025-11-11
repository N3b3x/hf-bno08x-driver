/**
 * @file Esp32Bno08xBus.cpp
 * @brief ESP32 I2C transport implementation for BNO08x driver
 */

#include "Esp32Bno08xBus.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

Esp32Bno08xBus::Esp32Bno08xBus(const I2CConfig& config) : config_(config) {
}

Esp32Bno08xBus::~Esp32Bno08xBus() {
    close();
}

bool Esp32Bno08xBus::open() {
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

void Esp32Bno08xBus::close() {
    if (!initialized_) {
        return;
    }

    // Delete I2C driver
    esp_err_t err = i2c_driver_delete(config_.port);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete I2C driver: %s", esp_err_to_name(err));
    }

    // Reset GPIO pins
    if (config_.int_pin != GPIO_NUM_NC) {
        gpio_reset_pin(config_.int_pin);
    }
    if (config_.rst_pin != GPIO_NUM_NC) {
        gpio_reset_pin(config_.rst_pin);
    }

    initialized_ = false;
    ESP_LOGI(TAG, "I2C bus closed");
}

int Esp32Bno08xBus::write(const uint8_t *data, uint32_t length) {
    if (!initialized_) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return -1;
    }

    esp_err_t err = i2c_master_write_to_device(
        config_.port,
        config_.device_address,
        data,
        length,
        pdMS_TO_TICKS(1000)  // 1 second timeout
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(err));
        return -1;
    }

    return static_cast<int>(length);
}

int Esp32Bno08xBus::read(uint8_t *data, uint32_t length) {
    if (!initialized_) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return -1;
    }

    esp_err_t err = i2c_master_read_from_device(
        config_.port,
        config_.device_address,
        data,
        length,
        pdMS_TO_TICKS(1000)  // 1 second timeout
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(err));
        return -1;
    }

    return static_cast<int>(length);
}

bool Esp32Bno08xBus::dataAvailable() {
    if (config_.int_pin == GPIO_NUM_NC) {
        // No interrupt pin configured, always return true
        return true;
    }

    // Check interrupt pin (active low)
    int level = gpio_get_level(config_.int_pin);
    return (level == 0);
}

void Esp32Bno08xBus::delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

uint32_t Esp32Bno08xBus::getTimeUs() {
    return static_cast<uint32_t>(esp_timer_get_time());
}

void Esp32Bno08xBus::setReset(bool state) {
    if (config_.rst_pin == GPIO_NUM_NC) {
        return;  // Reset pin not configured
    }

    // Reset is active low
    gpio_set_level(config_.rst_pin, state ? 0 : 1);
}

bool Esp32Bno08xBus::initializeI2C() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = config_.sda_pin;
    conf.scl_io_num = config_.scl_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = config_.frequency;
    conf.clk_flags = 0;

    esp_err_t err = i2c_param_config(config_.port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter config failed: %s", esp_err_to_name(err));
        return false;
    }

    err = i2c_driver_install(config_.port, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "I2C bus configured: SDA=%d, SCL=%d, Freq=%lu Hz, Addr=0x%02X",
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

