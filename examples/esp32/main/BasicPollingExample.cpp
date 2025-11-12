/**
 * @file BasicPollingExample.cpp
 * @brief Basic polling mode example with orientation and linear acceleration
 *
 * This example demonstrates:
 * - Basic initialization
 * - Enabling sensors (Rotation Vector and Linear Acceleration)
 * - Polling for sensor data
 * - Calculating Euler angles from quaternion
 *
 * @author N3b3x
 * @date 2025
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include <memory>
#include <stdio.h>

#include "../../../inc/BNO085.hpp"
#include "Esp32Bno08xBus.hpp"

static const char* TAG = "BNO08x_BasicPolling";

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "BNO08x Basic Polling Example");
  ESP_LOGI(TAG, "============================");

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

  // Enable Rotation Vector at 50 Hz (20ms period)
  imu.enableSensor(BNO085Sensor::RotationVector, 20);

  // Enable Linear Acceleration at 50 Hz
  imu.enableSensor(BNO085Sensor::LinearAcceleration, 20);

  ESP_LOGI(TAG, "Sensors enabled. Starting polling loop...");

  // Main polling loop
  while (true) {
    imu.update(); // Poll for new sensor data

    // Check for rotation vector data
    if (imu.hasNewData(BNO085Sensor::RotationVector)) {
      auto rot = imu.getLatestData(BNO085Sensor::RotationVector);

      // Calculate Euler angles from quaternion
      float qw = rot.rotation.w, qx = rot.rotation.x;
      float qy = rot.rotation.y, qz = rot.rotation.z;
      float ysqr = qy * qy;

      // Yaw (Z axis rotation)
      float t3 = 2.0f * (qw * qz + qx * qy);
      float t4 = 1.0f - 2.0f * (ysqr + qz * qz);
      float yaw = atan2f(t3, t4);

      // Pitch (X axis rotation)
      float t2 = 2.0f * (qw * qx - qy * qz);
      t2 = t2 > 1.0f ? 1.0f : (t2 < -1.0f ? -1.0f : t2);
      float pitch = asinf(t2);

      // Roll (Y axis rotation)
      float t0 = 2.0f * (qw * qy + qz * qx);
      float t1 = 1.0f - 2.0f * (qx * qx + ysqr);
      float roll = atan2f(t0, t1);

      ESP_LOGI(TAG, "Orientation YPR (rad): Yaw=%.2f, Pitch=%.2f, Roll=%.2f", yaw, pitch, roll);
    }

    // Check for linear acceleration data
    if (imu.hasNewData(BNO085Sensor::LinearAcceleration)) {
      auto lin = imu.getLatestData(BNO085Sensor::LinearAcceleration);
      ESP_LOGI(TAG, "Linear Accel (m/s^2): X=%.2f, Y=%.2f, Z=%.2f", lin.vector.x, lin.vector.y,
               lin.vector.z);
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay
  }
}
