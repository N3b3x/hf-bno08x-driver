/**
 * @file rvc_mode_example.cpp
 * @brief RVC (Reduced Vector Computation) mode example
 *
 * This example demonstrates:
 * - RVC mode initialization
 * - Frame callback handling
 * - Yaw, pitch, roll reading
 *
 * Note: RVC mode requires specific HAL implementation (RvcHalEsp32C6)
 *
 * @author N3b3x
 * @date 2025
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <memory>
#include <stdio.h>

#include "../../../inc/bno08x.hpp"
#include "../../../src/rvc/RvcHalEsp32C6.hpp"
#include "esp32_bno08x_bus.hpp"

static const char* TAG = "BNO08x_RvcMode";

// RVC frame callback
static void on_rvc_frame(const rvc_SensorValue_t& value) {
  ESP_LOGI(TAG, "RVC Frame: Yaw=%.2f°, Pitch=%.2f°, Roll=%.2f°", value.yaw_deg, value.pitch_deg,
           value.roll_deg);
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "BNO08x RVC Mode Example");
  ESP_LOGI(TAG, "========================");

  // Create RVC HAL instance
  // Note: Configure pins/port as needed for your hardware
  Esp32C6RvcHal hal;

  // Create transport (required for BNO085 constructor, even if not used for RVC)
  Esp32Bno08xBus::I2CConfig config;
  config.sda_pin = GPIO_NUM_21;
  config.scl_pin = GPIO_NUM_22;
  config.frequency = 400000;
  config.device_address = 0x4A;
  auto transport = std::make_unique<Esp32Bno08xBus>(config);
  if (!transport->Open()) {
    ESP_LOGE(TAG, "Failed to open transport");
    return;
  }

  // Create IMU instance (transport required by constructor, but RVC mode uses different HAL)
  BNO085<Esp32Bno08xBus> imu(*transport);

  // Set RVC callback
  imu.SetRvcCallback(on_rvc_frame);

  // Initialize in RVC mode
  if (!imu.BeginRvc(&hal)) {
    ESP_LOGE(TAG, "Failed to initialize BNO085 in RVC mode");
    return;
  }

  ESP_LOGI(TAG, "BNO085 initialized in RVC mode successfully");
  ESP_LOGI(TAG, "Starting RVC service loop...");

  // Main RVC service loop
  while (true) {
    imu.ServiceRvc();              // Service RVC mode
    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay
  }
}
