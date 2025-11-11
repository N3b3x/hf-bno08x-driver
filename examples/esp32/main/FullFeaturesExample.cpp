/**
 * @file FullFeaturesExample.cpp
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

#include <stdio.h>
#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "../../../src/BNO085.hpp"
#include "Esp32Bno08xBus.hpp"

static const char* TAG = "BNO08x_FullFeatures";

// Callback function for event-driven sensors
static void event_callback(const BNO085::SensorEvent& event) {
    switch (event.sensor) {
        case BNO085Sensor::RotationVector:
            ESP_LOGI(TAG, "Yaw %.1f pitch %.1f roll %.1f",
                     event.rotation.z, event.rotation.y, event.rotation.x);
            break;
            
        case BNO085Sensor::LinearAcceleration:
            ESP_LOGI(TAG, "Linear accel %.2f %.2f %.2f m/s^2",
                     event.vector.x, event.vector.y, event.vector.z);
            break;
            
        case BNO085Sensor::Gyroscope:
            ESP_LOGI(TAG, "Gyro %.2f %.2f %.2f rad/s",
                     event.vector.x, event.vector.y, event.vector.z);
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

    // Register callback for event-driven sensors
    imu.setCallback(event_callback);

    // Enable a broad set of sensors
    imu.enableSensor(BNO085Sensor::RotationVector, 10);      // 100 Hz
    imu.enableSensor(BNO085Sensor::LinearAcceleration, 20);  // 50 Hz
    imu.enableSensor(BNO085Sensor::Gyroscope, 20);           // 50 Hz
    imu.enableSensor(BNO085Sensor::Gravity, 50);             // 20 Hz
    imu.enableSensor(BNO085Sensor::StepCounter, 0);         // on change
    imu.enableSensor(BNO085Sensor::TapDetector, 0);          // gesture events

    ESP_LOGI(TAG, "All sensors enabled. Starting main loop...");

    // Main loop - mixes callback and polling usage
    while (true) {
        imu.update();  // Process events and update sensor data

        // Example of polling for data without callback
        if (imu.hasNewData(BNO085Sensor::Gravity)) {
            auto g = imu.getLatestData(BNO085Sensor::Gravity);
            ESP_LOGI(TAG, "Gravity %.2f %.2f %.2f m/s^2",
                     g.vector.x, g.vector.y, g.vector.z);
        }

        vTaskDelay(pdMS_TO_TICKS(5));  // 5ms delay
    }
}

