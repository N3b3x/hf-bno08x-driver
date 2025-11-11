/**
 * @file RvcModeExample.cpp
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

#include <stdio.h>
#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "../../../src/BNO085.hpp"
#include "../../../src/rvc/RvcHalEsp32C6.hpp"

static const char* TAG = "BNO08x_RvcMode";

// RVC frame callback
static void on_rvc_frame(const rvc_SensorValue_t& value) {
    ESP_LOGI(TAG, "RVC Frame: Yaw=%.2f°, Pitch=%.2f°, Roll=%.2f°",
             value.yaw_deg, value.pitch_deg, value.roll_deg);
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "BNO08x RVC Mode Example");
    ESP_LOGI(TAG, "========================");

    // Create RVC HAL instance
    // Note: Configure pins/port as needed for your hardware
    Esp32C6RvcHal hal;
    
    // Create IMU instance
    BNO085 imu;
    
    // Set RVC callback
    imu.setRvcCallback(on_rvc_frame);

    // Initialize in RVC mode
    if (!imu.beginRvc(&hal)) {
        ESP_LOGE(TAG, "Failed to initialize BNO085 in RVC mode");
        return;
    }

    ESP_LOGI(TAG, "BNO085 initialized in RVC mode successfully");
    ESP_LOGI(TAG, "Starting RVC service loop...");

    // Main RVC service loop
    while (true) {
        imu.serviceRvc();  // Service RVC mode
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay
    }
}

