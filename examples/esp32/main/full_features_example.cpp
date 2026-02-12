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
#include <memory>
#include <stdio.h>

#include "../../../inc/bno08x.hpp"
#include "esp32_bno08x_bus.hpp"

static const char* TAG = "BNO08x_FullFeatures";

// Callback function for event-driven sensors
static void event_callback(const SensorEvent& event) {
  switch (event.sensor) {
  case BNO085Sensor::RotationVector:
    // Log quaternion (w,x,y,z); convert to Euler if needed for yaw/pitch/roll
    ESP_LOGI(TAG, "Quat w=%.2f x=%.2f y=%.2f z=%.2f (accuracy=%u)", event.rotation.w,
             event.rotation.x, event.rotation.y, event.rotation.z,
             static_cast<unsigned>(event.rotation.accuracy));
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
      if (event.tap.doubleTap) {
        ESP_LOGI(TAG, "Double tap!");
      } else {
        ESP_LOGI(TAG, "Tap!");
      }
    }
    break;

  default:
    break;
  }
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "BNO08x Full Features Example");
  ESP_LOGI(TAG, "=============================");

  // Configure I2C transport (same bus pins as pcal95555/pca9685 examples)
  Esp32Bno08xBus::I2CConfig config;
  config.sda_pin = GPIO_NUM_4;
  config.scl_pin = GPIO_NUM_5;
  config.frequency = 400000;
  config.rst_pin = GPIO_NUM_16; // Reset pin (RSTN) on GPIO16
  config.int_pin = GPIO_NUM_17; // Interrupt pin (INT) on GPIO17

  // BNO08x uses 7-bit I2C addresses: 0x4A (SA0=LOW) or 0x4B (SA0=HIGH)
  // Try both addresses automatically - try 0x4B first (SA0=HIGH) as it's the default on this board
  const uint8_t addresses[] = {0x4B, 0x4A};
  std::unique_ptr<Esp32Bno08xBus> transport;
  BNO085<Esp32Bno08xBus>* imu = nullptr;
  bool initialized = false;

  for (size_t i = 0; i < sizeof(addresses) / sizeof(addresses[0]); i++) {
    config.device_address = addresses[i];
    ESP_LOGI(TAG, "Trying BNO08x at I2C address 0x%02X (SA0=%s)...", config.device_address,
             (i == 0) ? "HIGH" : "LOW");

    // Create and initialize I2C transport
    transport = CreateEsp32Bno08xBus(config);
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
    imu = new BNO085<Esp32Bno08xBus>(*transport);

    // Try to initialize
    ESP_LOGI(TAG, "Initializing BNO085 at address 0x%02X...", config.device_address);
    if (imu->Begin()) {
      // Verify initialization by checking if we can communicate
      vTaskDelay(pdMS_TO_TICKS(50)); // Give sensor time to send reset notification

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

  // Register callback for event-driven sensors
  imu->SetCallback(event_callback);

  // Enable a broad set of sensors
  imu->EnableSensor(BNO085Sensor::RotationVector, 10);     // 100 Hz
  imu->EnableSensor(BNO085Sensor::LinearAcceleration, 20); // 50 Hz
  imu->EnableSensor(BNO085Sensor::Gyroscope, 20);          // 50 Hz
  imu->EnableSensor(BNO085Sensor::Gravity, 50);            // 20 Hz
  imu->EnableSensor(BNO085Sensor::StepCounter, 0);         // on change
  imu->EnableSensor(BNO085Sensor::TapDetector, 0);         // gesture events

  ESP_LOGI(TAG, "All sensors enabled. Starting main loop...");

  // Main loop - mixes callback and polling usage
  while (true) {
    imu->Update(); // Process events and update sensor data

    // Example of polling for data without callback
    if (imu->HasNewData(BNO085Sensor::Gravity)) {
      auto g = imu->GetLatest(BNO085Sensor::Gravity);
      ESP_LOGI(TAG, "Gravity %.2f %.2f %.2f m/s^2", g.vector.x, g.vector.y, g.vector.z);
    }

    vTaskDelay(pdMS_TO_TICKS(5)); // 5ms delay
  }
}
