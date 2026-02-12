/**
 * @file basic_polling_example.cpp
 * @brief Basic polling mode example with orientation and linear acceleration
 *
 * This example demonstrates:
 * - Basic initialization using the factory function
 * - Enabling sensors (Rotation Vector and Linear Acceleration)
 * - Polling for sensor data
 * - Calculating Euler angles from quaternion
 *
 * @note This example runs independently on an I2C bus that only communicates
 *       with the BNO08x device. The I2C bus setup (GPIO4 SDA, GPIO5 SCL)
 *       mirrors the pcal95555/pca9685 examples for pin consistency.
 *
 * @author Nebiyu Tadesse
 * @date 2025
 * @copyright HardFOC
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include <memory>
#include <stdio.h>

#include "../../../inc/bno08x.hpp"
#include "esp32_bno08x_bus.hpp"

static const char* TAG = "BNO08x_BasicPolling";

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "BNO08x Basic Polling Example");
  ESP_LOGI(TAG, "============================");

  // Configure I2C transport (same bus pins as pcal95555/pca9685 examples)
  Esp32Bno08xI2cBus::I2CConfig config;
  config.sda_pin = GPIO_NUM_4;
  config.scl_pin = GPIO_NUM_5;
  config.frequency = 400000;
  config.rst_pin = GPIO_NUM_16; // Reset pin (RSTN) on GPIO16
  config.int_pin = GPIO_NUM_17; // Interrupt pin (INT) on GPIO17

  // BNO08x uses 7-bit I2C addresses: 0x4A (SA0=LOW) or 0x4B (SA0=HIGH)
  // Try both addresses automatically per datasheet specification
  // Try 0x4B first (SA0=HIGH) as it's the default on this board
  const uint8_t addresses[] = {0x4B, 0x4A};
  std::unique_ptr<Esp32Bno08xI2cBus> transport;
  BNO085<Esp32Bno08xI2cBus>* imu = nullptr;
  bool initialized = false;

  for (size_t i = 0; i < sizeof(addresses) / sizeof(addresses[0]); i++) {
    config.device_address = addresses[i];
    ESP_LOGI(TAG, "Trying BNO08x at I2C address 0x%02X (SA0=%s)...", config.device_address,
             (i == 0) ? "HIGH" : "LOW");

    // Create and initialize I2C transport
    transport = CreateEsp32Bno08xI2cBus(config);
    if (!transport) {
      ESP_LOGW(TAG, "Failed to create I2C transport for address 0x%02X", config.device_address);
      continue;
    }

    // Perform hardware reset BEFORE probing (BNO08x needs reset before communication)
    ESP_LOGI(TAG, "Performing hardware reset before probing...");
    transport->HardwareReset(2, 200); // 2ms reset pulse, 200ms boot delay

    // Probe I2C device to verify it's responding
    ESP_LOGI(TAG, "Probing I2C device at address 0x%02X...", config.device_address);
    if (!transport->Probe()) {
      ESP_LOGW(TAG, "I2C probe failed at address 0x%02X (device not responding)",
               config.device_address);
      transport.reset();
      continue;
    }
    ESP_LOGI(TAG, "I2C probe successful at address 0x%02X", config.device_address);

    // Create IMU instance
    imu = new BNO085<Esp32Bno08xI2cBus>(*transport);

    // Try to initialize
    ESP_LOGI(TAG, "Initializing BNO085 at address 0x%02X...", config.device_address);
    if (imu->Begin()) {
      // Verify initialization by checking if we can communicate
      // The SH-2 library might return success even if sensor isn't responding
      vTaskDelay(pdMS_TO_TICKS(50)); // Give sensor time to send reset notification

      // Try a simple operation to verify communication
      // If probe fails now, initialization didn't actually work
      if (transport->Probe()) {
        ESP_LOGI(TAG, "BNO085 initialized successfully at address 0x%02X", config.device_address);
        initialized = true;
        break;
      } else {
        ESP_LOGW(TAG, "Begin() returned success but device not responding at 0x%02X",
                 config.device_address);
      }
    }

    ESP_LOGW(TAG, "Failed to initialize BNO085 at address 0x%02X (error: %d)",
             config.device_address, imu->GetLastError());
    delete imu;
    imu = nullptr;
    transport.reset();
  }

  if (!initialized || !imu) {
    ESP_LOGE(TAG, "Failed to initialize BNO085 at any address");
    ESP_LOGE(TAG, "Tried addresses: 0x4B (SA0=HIGH) and 0x4A (SA0=LOW)");
    ESP_LOGE(TAG, "Check:");
    ESP_LOGE(TAG, "  1. I2C wiring (SDA/SCL)");
    ESP_LOGE(TAG, "  2. Power supply (3.3V)");
    ESP_LOGE(TAG, "  3. SA0 pin state (determines address: LOW=0x4A, HIGH=0x4B)");
    ESP_LOGE(TAG, "  4. Pull-up resistors (2.2k-4.7k on SDA/SCL)");
    ESP_LOGE(TAG, "  5. PS0/PS1 pins (both LOW for I2C mode)");
    return;
  }

  // Enable Rotation Vector at 50 Hz (20ms period)
  imu->EnableSensor(BNO085Sensor::RotationVector, 20);

  // Enable Linear Acceleration at 50 Hz
  imu->EnableSensor(BNO085Sensor::LinearAcceleration, 20);

  ESP_LOGI(TAG, "Sensors enabled. Starting polling loop...");

  // Main polling loop
  uint32_t loop_count = 0;
  while (true) {
    imu->Update(); // Poll for new sensor data (handles clock stretching automatically)
    loop_count++;

    // Periodically check for driver errors (e.g. I2C NAK, timeout)
    if ((loop_count % 100 == 0) && imu->GetLastError() != 0) {
      ESP_LOGW(TAG, "Driver error: %d (last error)", imu->GetLastError());
    }

    // Check for rotation vector data
    if (imu->HasNewData(BNO085Sensor::RotationVector)) {
      auto rot = imu->GetLatest(BNO085Sensor::RotationVector);

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
    if (imu->HasNewData(BNO085Sensor::LinearAcceleration)) {
      auto lin = imu->GetLatest(BNO085Sensor::LinearAcceleration);
      ESP_LOGI(TAG, "Linear Accel (m/s^2): X=%.2f, Y=%.2f, Z=%.2f", lin.vector.x, lin.vector.y,
               lin.vector.z);
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay
  }
}
