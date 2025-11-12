/**
 * @file EventDrivenCallbackExample.cpp
 * @brief Event-driven callback example with step counter and tap detector
 *
 * This example demonstrates:
 * - Event-driven sensor callbacks
 * - Step counter detection
 * - Tap detector (single and double tap)
 * - Gesture event handling
 *
 * @author N3b3x
 * @date 2025
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <memory>
#include <stdio.h>

#include "../../../inc/BNO085.hpp"
#include "Esp32Bno08xBus.hpp"

static const char* TAG = "BNO08x_EventCallback";

// Callback function for sensor events
static void sensor_callback(const BNO085::SensorEvent& event) {
  switch (event.sensor) {
  case BNO085Sensor::StepCounter:
    ESP_LOGI(TAG, "Steps counted: %lu", event.stepCount);
    break;

  case BNO085Sensor::TapDetector:
    if (event.detected) {
      if (event.tap.doubleTap) {
        ESP_LOGI(TAG, "Double tap detected!");
      } else {
        ESP_LOGI(TAG, "Single tap detected on axis %d", event.tap.direction);
      }
    }
    break;

  case BNO085Sensor::StepDetector:
    ESP_LOGI(TAG, "Step detected!");
    break;

  case BNO085Sensor::ShakeDetector:
    ESP_LOGI(TAG, "Shake detected!");
    break;

  default:
    ESP_LOGI(TAG, "Event received for sensor: %d", static_cast<int>(event.sensor));
    break;
  }
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "BNO08x Event-Driven Callback Example");
  ESP_LOGI(TAG, "====================================");

  // Configure I2C transport
  Esp32Bno08xBus::I2CConfig config;
  config.sda_pin = GPIO_NUM_21;
  config.scl_pin = GPIO_NUM_22;
  config.frequency = 400000;
  config.device_address = 0x4A;

  auto transport = std::make_unique<Esp32Bno08xBus>(config);

  if (!transport->open()) {
    ESP_LOGE(TAG, "Failed to open I2C transport");
    return;
  }

  // Create IMU instance
  BNO085 imu(transport.get());

  // Initialize IMU
  if (!imu.begin()) {
    ESP_LOGE(TAG, "Failed to initialize BNO085");
    return;
  }

  ESP_LOGI(TAG, "BNO085 initialized successfully");

  // Register callback for all events
  imu.setCallback(sensor_callback);

  // Enable step counter (on-change events)
  imu.enableSensor(BNO085Sensor::StepCounter, 0);

  // Enable tap detector (event-driven)
  imu.enableSensor(BNO085Sensor::TapDetector, 0);

  // Enable step detector (event-driven)
  imu.enableSensor(BNO085Sensor::StepDetector, 0);

  // Enable shake detector (event-driven)
  imu.enableSensor(BNO085Sensor::ShakeDetector, 0);

  ESP_LOGI(TAG, "Event-driven sensors enabled. Waiting for events...");

  // Main loop - call update() to process events
  while (true) {
    imu.update();                  // Process incoming events and call callbacks
    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay
  }
}
