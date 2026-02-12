/**
 * @file esp32_bno08x_test_config.hpp
 * @brief Hardware configuration for BNO08x driver on ESP32-S3
 *
 * This file contains the actual hardware configuration that is used by the HAL
 * and example applications. Modify these values to match your hardware setup.
 *
 * @copyright Copyright (c) 2024-2025 HardFOC. All rights reserved.
 */

#pragma once

#include <cstdint>

//==============================================================================
// COMPILE-TIME CONFIGURATION FLAGS
//==============================================================================

/**
 * @brief Enable detailed I2C transaction logging
 *
 * @details
 * When enabled (set to 1), the Esp32Bno08xI2cBus will log detailed
 * information about each I2C transaction including:
 * - TX/RX byte contents
 * - SH-2 packet header parsing
 * - Clock stretching events
 *
 * When disabled (set to 0), only basic error logging is performed.
 *
 * Default: 0 (disabled) - Set to 1 to enable for debugging
 */
#ifndef ESP32_BNO08X_ENABLE_DETAILED_I2C_LOGGING
#define ESP32_BNO08X_ENABLE_DETAILED_I2C_LOGGING 0
#endif

namespace BNO08x_TestConfig {

/**
 * @brief I2C Pin Configuration for ESP32-S3
 *
 * These pins are used for I2C communication with the BNO08x.
 * Ensure your hardware matches these pin assignments or modify accordingly.
 */
struct I2CPins {
    static constexpr uint8_t SDA = 4;           ///< GPIO4 - I2C SDA (data)
    static constexpr uint8_t SCL = 5;           ///< GPIO5 - I2C SCL (clock)
};

/**
 * @brief Control GPIO Pins for BNO08x
 *
 * These pins control device operation and status monitoring.
 * Set to -1 if not connected/configured.
 */
struct ControlPins {
    static constexpr int8_t RST  = 16;          ///< GPIO16 - Reset pin (RSTN, active low)
    static constexpr int8_t INT  = 17;          ///< GPIO17 - Interrupt pin (HINTN, active low)
    static constexpr int8_t BOOT = -1;          ///< BOOTN pin (not wired in this design)
    static constexpr int8_t WAKE = -1;          ///< WAKE pin (SPI mode only, not used for I2C)
};

/**
 * @brief I2C Communication Parameters
 *
 * The BNO08x supports I2C frequencies up to 400kHz (Fast Mode).
 *
 * I2C Addressing (per BNO08x datasheet):
 * - 0x4B when SA0 = HIGH (default)
 * - 0x4A when SA0 = LOW
 *
 * Clock stretching: BNO08x can hold SCL low when data is not ready.
 * scl_wait_us must be long enough to accommodate this (50ms recommended).
 */
struct I2CParams {
    static constexpr uint32_t FREQUENCY = 400000;     ///< 400kHz I2C frequency (Fast Mode)
    static constexpr uint8_t DEVICE_ADDRESS = 0x4B;   ///< 7-bit I2C address (SA0=HIGH)
    static constexpr uint8_t ALT_ADDRESS = 0x4A;      ///< Alternative address (SA0=LOW)
    static constexpr uint32_t SCL_WAIT_US = 50000;    ///< Clock stretching timeout (50ms)
    static constexpr bool PULLUP_ENABLE = true;        ///< Enable internal pullups
};

/**
 * @brief Sensor Specifications
 *
 * BNO08x is a 9-axis IMU with integrated sensor fusion.
 */
struct SensorSpecs {
    static constexpr uint16_t ACCEL_RANGE_G = 8;            ///< Accelerometer range (±g)
    static constexpr uint16_t GYRO_RANGE_DPS = 2000;        ///< Gyroscope range (±°/s)
    static constexpr float ROTATION_ACCURACY_DEG = 2.0f;    ///< Rotation vector accuracy (°)
    static constexpr uint16_t MAX_REPORT_RATE_HZ = 400;     ///< Maximum report rate (Hz)
};

/**
 * @brief Supply Voltage Specifications (volts)
 *
 * VDD: Power supply for BNO08x
 */
struct SupplyVoltage {
    static constexpr float VDD_MIN = 2.4f;     ///< Minimum VDD voltage (V)
    static constexpr float VDD_NOM = 3.3f;     ///< Nominal VDD voltage (V)
    static constexpr float VDD_MAX = 3.6f;     ///< Maximum VDD voltage (V)
};

/**
 * @brief Temperature Specifications (celsius)
 *
 * Operating temperature range from BNO08x datasheet.
 */
struct Temperature {
    static constexpr int16_t OPERATING_MIN = -40;    ///< Minimum operating temperature (°C)
    static constexpr int16_t OPERATING_MAX = 85;     ///< Maximum operating temperature (°C)
    static constexpr int16_t WARNING_THRESHOLD = 75; ///< Temperature warning threshold (°C)
};

/**
 * @brief Timing Parameters
 *
 * Timing requirements from the BNO08x datasheet.
 */
struct Timing {
    static constexpr uint16_t POWER_ON_DELAY_MS = 100;      ///< Power-on initialization delay (ms)
    static constexpr uint16_t RESET_LOW_MS = 2;             ///< Reset pulse low duration (ms)
    static constexpr uint16_t BOOT_DELAY_MS = 200;          ///< Boot delay after reset release (ms)
    static constexpr uint16_t SHTP_HEADER_TIMEOUT_MS = 100; ///< SHTP header read timeout (ms)
};

/**
 * @brief Diagnostic Thresholds
 *
 * Thresholds for sensor health monitoring and error detection.
 */
struct Diagnostics {
    static constexpr uint16_t POLL_INTERVAL_MS = 100;      ///< Diagnostic polling interval (ms)
    static constexpr uint8_t MAX_RETRY_COUNT = 3;          ///< Maximum communication retries
    static constexpr uint8_t SENSOR_RESET_LIMIT = 5;       ///< Max resets before failsafe
};

/**
 * @brief Test Configuration
 *
 * Default parameters for testing.
 */
struct TestConfig {
    static constexpr uint16_t REPORT_COUNT = 100;           ///< Number of reports per test
    static constexpr uint16_t SAMPLE_INTERVAL_MS = 10;      ///< Sampling interval (ms)
    static constexpr uint16_t TEST_DURATION_MS = 5000;      ///< Test duration (ms)
    static constexpr float QUATERNION_TOLERANCE = 0.01f;    ///< Quaternion norm tolerance
};

/**
 * @brief Application-specific Configuration
 *
 * Configuration values that can be adjusted per application.
 */
struct AppConfig {
    // Logging
    static constexpr bool ENABLE_DEBUG_LOGGING = true;     ///< Enable detailed debug logs
    static constexpr bool ENABLE_I2C_LOGGING = false;      ///< Enable I2C transaction logs

    // Performance
    static constexpr bool ENABLE_PERFORMANCE_MONITORING = true;  ///< Enable performance metrics
    static constexpr uint16_t STATS_REPORT_INTERVAL_MS = 10000;  ///< Statistics reporting interval

    // Error handling
    static constexpr bool ENABLE_AUTO_RECOVERY = true;     ///< Enable automatic error recovery
    static constexpr uint8_t MAX_ERROR_COUNT = 10;         ///< Maximum errors before failsafe
};

} // namespace BNO08x_TestConfig

/**
 * @brief Hardware configuration validation
 *
 * Compile-time checks to ensure configuration is valid.
 */
static_assert(BNO08x_TestConfig::I2CParams::FREQUENCY <= 400000,
              "I2C frequency exceeds BNO08x maximum of 400kHz");

static_assert(BNO08x_TestConfig::I2CParams::DEVICE_ADDRESS == 0x4B ||
              BNO08x_TestConfig::I2CParams::DEVICE_ADDRESS == 0x4A,
              "BNO08x I2C address must be 0x4A or 0x4B");

/**
 * @brief Helper macro for compile-time GPIO pin validation
 */
#define BNO08X_VALIDATE_GPIO(pin) \
    static_assert((pin) >= 0 && (pin) < 49, "Invalid GPIO pin number for ESP32-S3")
