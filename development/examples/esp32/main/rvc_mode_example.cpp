/**
 * @file rvc_mode_example.cpp
 * @brief RVC (Reduced Vector Computation) mode example
 *
 * This example demonstrates:
 * - RVC mode initialization
 * - Frame callback handling
 * - Yaw, pitch, roll reading
 *
 * Note: RVC mode requires a platform-specific HAL implementation (Esp32RvcHal)
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
#include "esp32_rvc_hal.hpp"

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
  // Note: Configure UART port and pins as needed for your hardware
  // Default: UART_NUM_1, TX=GPIO21, RX=GPIO20, RST=GPIO16, BOOT=GPIO10
  Esp32RvcHal hal;

  // Configure I2C transport (required for BNO085 constructor, even if not used for RVC)
  Esp32Bno08xBus::I2CConfig config;
  config.sda_pin = GPIO_NUM_4;
  config.scl_pin = GPIO_NUM_5;
  config.frequency = 400000;
  config.rst_pin = GPIO_NUM_16;  // Reset pin (RSTN) on GPIO16
  config.int_pin = GPIO_NUM_17;  // Interrupt pin (INT) on GPIO17
  
  // BNO08x uses 7-bit I2C addresses: 0x4A (SA0=LOW) or 0x4B (SA0=HIGH)
  // Try both addresses automatically - try 0x4B first (SA0=HIGH) as it's the default on this board
  const uint8_t addresses[] = {0x4B, 0x4A};
  std::unique_ptr<Esp32Bno08xBus> transport;
  BNO085<Esp32Bno08xBus>* imu = nullptr;
  bool initialized = false;

  for (size_t i = 0; i < sizeof(addresses) / sizeof(addresses[0]); i++) {
    config.device_address = addresses[i];
    ESP_LOGI(TAG, "Trying BNO08x at I2C address 0x%02X (SA0=%s)...", 
             config.device_address, (i == 0) ? "HIGH" : "LOW");

    // Create and initialize I2C transport
    transport = CreateEsp32Bno08xBus(config);
    if (!transport) {
      ESP_LOGW(TAG, "Failed to create I2C transport for address 0x%02X", config.device_address);
      continue;
    }

    // Perform hardware reset BEFORE probing (BNO08x needs reset before communication)
    ESP_LOGI(TAG, "Performing hardware reset before probing...");
    transport->HardwareReset(2, 200);  // 2ms reset pulse, 200ms boot delay

    // Probe I2C device to verify it's responding
    ESP_LOGI(TAG, "Probing I2C device at address 0x%02X...", config.device_address);
    if (!transport->Probe()) {
      ESP_LOGW(TAG, "I2C probe failed at address 0x%02X (device not responding)", config.device_address);
      transport.reset();
      continue;
    }
    ESP_LOGI(TAG, "I2C probe successful at address 0x%02X", config.device_address);

    // Create IMU instance (transport required by constructor, but RVC mode uses different HAL)
    imu = new BNO085<Esp32Bno08xBus>(*transport);

    // Set RVC callback
    imu->SetRvcCallback(on_rvc_frame);

    // Try to initialize in RVC mode
    ESP_LOGI(TAG, "Initializing BNO085 in RVC mode at address 0x%02X...", config.device_address);
    if (imu->BeginRvc(hal)) {
      ESP_LOGI(TAG, "BNO085 initialized successfully in RVC mode at address 0x%02X", config.device_address);
      initialized = true;
      break;
    } else {
      ESP_LOGW(TAG, "Failed to initialize BNO085 in RVC mode at address 0x%02X", config.device_address);
      delete imu;
      imu = nullptr;
      transport.reset();
    }
  }

  if (!initialized || !imu) {
    ESP_LOGE(TAG, "Failed to initialize BNO085 in RVC mode at any address");
    ESP_LOGE(TAG, "Tried addresses: 0x4B (SA0=HIGH) and 0x4A (SA0=LOW)");
    ESP_LOGE(TAG, "Check:");
    ESP_LOGE(TAG, "  1. I2C wiring (SDA/SCL)");
    ESP_LOGE(TAG, "  2. Power supply (3.3V)");
    ESP_LOGE(TAG, "  3. SA0 pin state (determines address: LOW=0x4A, HIGH=0x4B)");
    ESP_LOGE(TAG, "  4. Pull-up resistors (2.2k-4.7k on SDA/SCL)");
    ESP_LOGE(TAG, "  5. PS0/PS1 pins (both LOW for I2C mode)");
    return;
  }

  ESP_LOGI(TAG, "Starting RVC service loop...");

  // Main RVC service loop
  while (true) {
    imu->ServiceRvc();              // Service RVC mode
    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay
  }
}
