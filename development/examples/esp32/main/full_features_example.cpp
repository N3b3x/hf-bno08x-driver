/**
 * @file full_features_example.cpp
 * @brief Comprehensive example demonstrating all BNO08x features
 *
 * This example demonstrates:
 * - Multiple sensor types enabled simultaneously
 * - Both callback and polling methods
 * - Orientation, acceleration, gyroscope, gravity
 * - Step counter, tap detector, gesture events
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
#include "esp32_bno08x_bus.hpp"

static const char* TAG = "BNO08x_FullFeatures";

// Callback function for event-driven sensors
static void event_callback(const BNO085::SensorEvent& event) {
  switch (event.sensor) {
  case BNO085Sensor::RotationVector:
    ESP_LOGI(TAG, "Yaw %.1f pitch %.1f roll %.1f", event.rotation.z, event.rotation.y,
             event.rotation.x);
    break;

  case BNO085Sensor::LinearAcceleration:
    ESP_LOGI(TAG, "Linear accel %.2f %.2f %.2f m/s^2", event.vector.x, event.vector.y,
             event.vector.z);
    break;

  case BNO085Sensor::Gyroscope:
    ESP_LOGI(TAG, "Gyro %.2f %.2f %.2f rad/s", event.vector.x, event.vector.y, event.vector.z);
    break;

  case BNO085Sensor::StepCounter:
    ESP_LOGI(TAG, "Steps: %lu", event.stepCount);
    break;

  case BNO085Sensor::TapDetector:
    if (event.detected) {
      ESP_LOGI(TAG, event.tap.doubleTap ? "Double tap!" : "Tap!");
    }
    break;

  default:
    break;
  }
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "BNO08x Full Features Example");
  ESP_LOGI(TAG, "=============================");

  // Configure I2C transport
  Esp32Bno08xBus::I2CConfig config;
  config.sda_pin = GPIO_NUM_21;
  config.scl_pin = GPIO_NUM_22;
  config.frequency = 400000;
  config.device_address = 0x4A;

  auto transport = std::make_unique<Esp32Bno08xBus>(config);

  if (!transport->Open()) {
    ESP_LOGE(TAG, "Failed to open I2C transport");
    return;
  }

  // Create IMU instance
  BNO085<Esp32Bno08xBus> imu(*transport);

  // Initialize IMU
  if (!imu.Begin()) {
    ESP_LOGE(TAG, "Failed to initialize BNO085");
    return;
  }

  ESP_LOGI(TAG, "BNO085 initialized successfully");

  // Register callback for event-driven sensors
  imu.SetCallback(event_callback);

  // Enable a broad set of sensors
  imu.EnableSensor(BNO085Sensor::RotationVector, 10);     // 100 Hz
  imu.EnableSensor(BNO085Sensor::LinearAcceleration, 20); // 50 Hz
  imu.EnableSensor(BNO085Sensor::Gyroscope, 20);          // 50 Hz
  imu.EnableSensor(BNO085Sensor::Gravity, 50);            // 20 Hz
  imu.EnableSensor(BNO085Sensor::StepCounter, 0);         // on change
  imu.EnableSensor(BNO085Sensor::TapDetector, 0);         // gesture events

  ESP_LOGI(TAG, "All sensors enabled. Starting main loop...");

  // Main loop - mixes callback and polling usage
  while (true) {
    imu.Update(); // Process events and update sensor data

    // Example of polling for data without callback
    if (imu.HasNewData(BNO085Sensor::Gravity)) {
      auto g = imu.getLatestData(BNO085Sensor::Gravity);
      ESP_LOGI(TAG, "Gravity %.2f %.2f %.2f m/s^2", g.vector.x, g.vector.y, g.vector.z);
    }

    vTaskDelay(pdMS_TO_TICKS(5)); // 5ms delay
  }
}
