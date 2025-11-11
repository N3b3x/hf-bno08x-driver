/**
 * @file TestFramework.h
 * @brief Testing framework for BNO08x driver examples on ESP32-C6
 * 
 * This file provides testing infrastructure including:
 * - Test result tracking
 * - Execution timing  
 * - FreeRTOS task-based test execution
 * - GPIO14 progress indicator
 * - Standardized test execution macros
 * 
 * Adapted from HardFOC Internal Interface Wrap test framework
 * 
 * @author N3b3x
 * @date 2025
 * @version 2.0.0
 */

#pragma once

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

//=============================================================================
// GPIO14 TEST PROGRESSION INDICATOR
//=============================================================================

static constexpr gpio_num_t TEST_PROGRESS_PIN = GPIO_NUM_14;
static bool g_gpio14_initialized = false;
static bool g_gpio14_state = false;

/**
 * @brief Initialize GPIO14 as test progression indicator
 */
inline bool init_test_progress_indicator() noexcept {
    if (g_gpio14_initialized) {
        return true;
    }
    
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << TEST_PROGRESS_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    
    if (gpio_config(&io_conf) != ESP_OK) {
        return false;
    }
    
    gpio_set_level(TEST_PROGRESS_PIN, 0);
    g_gpio14_state = false;
    g_gpio14_initialized = true;
    
    return true;
}

/**
 * @brief Toggle GPIO14 to indicate test progression
 */
inline void flip_test_progress_indicator() noexcept {
    if (!g_gpio14_initialized) {
        return;
    }
    
    g_gpio14_state = !g_gpio14_state;
    gpio_set_level(TEST_PROGRESS_PIN, g_gpio14_state ? 1 : 0);
    vTaskDelay(pdMS_TO_TICKS(50));
}

/**
 * @brief Blink GPIO14 to indicate section start/end
 */
inline void output_section_indicator(uint8_t blink_count = 5) noexcept {
    if (!g_gpio14_initialized) {
        return;
    }
    
    for (uint8_t i = 0; i < blink_count; ++i) {
        gpio_set_level(TEST_PROGRESS_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(TEST_PROGRESS_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    g_gpio14_state = false;
}

/**
 * @brief Cleanup GPIO14 indicator
 */
inline void cleanup_test_progress_indicator() noexcept {
    if (g_gpio14_initialized) {
        gpio_set_level(TEST_PROGRESS_PIN, 0);
        gpio_reset_pin(TEST_PROGRESS_PIN);
        g_gpio14_initialized = false;
        g_gpio14_state = false;
    }
}

/**
 * @brief Ensure GPIO14 is initialized
 */
inline void ensure_gpio14_initialized() noexcept {
    if (!g_gpio14_initialized) {
        init_test_progress_indicator();
    }
}

//=============================================================================
// TEST RESULT TRACKING
//=============================================================================

/**
 * @brief Test execution tracking and results accumulation
 */
struct TestResults {
    int total_tests = 0;
    int passed_tests = 0;
    int failed_tests = 0;
    uint64_t total_execution_time_us = 0;
    
    void add_result(bool passed, uint64_t execution_time) noexcept {
        total_tests++;
        total_execution_time_us += execution_time;
        if (passed) {
            passed_tests++;
        } else {
            failed_tests++;
        }
    }
    
    double get_success_percentage() const noexcept {
        return total_tests > 0 ? (static_cast<double>(passed_tests) / total_tests * 100.0) : 0.0;
    }
    
    double get_total_time_ms() const noexcept {
        return total_execution_time_us / 1000.0;
    }
    
    void print_summary(const char* tag) const noexcept {
        ESP_LOGI(tag, "");
        ESP_LOGI(tag, "╔══════════════════════════════════════════════════════════════════════════════╗");
        ESP_LOGI(tag, "║                           TEST RESULTS SUMMARY                               ║");
        ESP_LOGI(tag, "╠══════════════════════════════════════════════════════════════════════════════╣");
        ESP_LOGI(tag, "║  Total Tests:      %-56d  ║", total_tests);
        ESP_LOGI(tag, "║  Passed:           %-56d  ║", passed_tests);
        ESP_LOGI(tag, "║  Failed:           %-56d  ║", failed_tests);
        ESP_LOGI(tag, "║  Success Rate:     %-55.2f%%  ║", get_success_percentage());
        ESP_LOGI(tag, "║  Total Time:       %-52.2f  ms  ║", get_total_time_ms());
        ESP_LOGI(tag, "╚══════════════════════════════════════════════════════════════════════════════╝");
        ESP_LOGI(tag, "");
    }
};

// Global test results
static TestResults g_test_results;

//=============================================================================
// TEST EXECUTION MACROS
//=============================================================================

/**
 * @brief Run a test function inline (no separate task)
 */
#define RUN_TEST(test_func) \
do { \
    ensure_gpio14_initialized(); \
    ESP_LOGI(TAG, ""); \
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗"); \
    ESP_LOGI(TAG, "║ Running: " #test_func "                                                       "); \
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝"); \
    uint64_t start_time = esp_timer_get_time(); \
    bool result = test_func(); \
    uint64_t end_time = esp_timer_get_time(); \
    uint64_t execution_time = end_time - start_time; \
    g_test_results.add_result(result, execution_time); \
    if (result) { \
        ESP_LOGI(TAG, "[SUCCESS] PASSED: " #test_func " (%.2f ms)", execution_time / 1000.0); \
    } else { \
        ESP_LOGE(TAG, "[FAILED] FAILED: " #test_func " (%.2f ms)", execution_time / 1000.0); \
    } \
    flip_test_progress_indicator(); \
    vTaskDelay(pdMS_TO_TICKS(100)); \
} while (0)

/**
 * @brief Context for test task
 */
struct TestTaskContext {
    const char* test_name;
    bool (*test_func)() noexcept;
    TestResults* results;
    const char* tag;
    SemaphoreHandle_t completion_semaphore;
};

/**
 * @brief FreeRTOS task trampoline for test execution
 */
inline void test_task_trampoline(void* param) {
    TestTaskContext* ctx = static_cast<TestTaskContext*>(param);
    ESP_LOGI(ctx->tag, "");
    ESP_LOGI(ctx->tag, "╔══════════════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(ctx->tag, "║ Running (task): %-62s║", ctx->test_name);
    ESP_LOGI(ctx->tag, "╚══════════════════════════════════════════════════════════════════════════════╝");
    
    uint64_t start_time = esp_timer_get_time();
    bool result = ctx->test_func();
    uint64_t end_time = esp_timer_get_time();
    uint64_t execution_time = end_time - start_time;
    
    ctx->results->add_result(result, execution_time);
    
    if (result) {
        ESP_LOGI(ctx->tag, "[SUCCESS] PASSED (task): %s (%.2f ms)", 
                 ctx->test_name, execution_time / 1000.0);
    } else {
        ESP_LOGE(ctx->tag, "[FAILED] FAILED (task): %s (%.2f ms)", 
                 ctx->test_name, execution_time / 1000.0);
    }
    
    xSemaphoreGive(ctx->completion_semaphore);
    vTaskDelete(nullptr);
}

/**
 * @brief Run a test in its own FreeRTOS task
 */
#define RUN_TEST_IN_TASK(name, func, stack_size_bytes, priority) \
do { \
    ensure_gpio14_initialized(); \
    static TestTaskContext ctx; \
    ctx.test_name = name; \
    ctx.test_func = func; \
    ctx.results = &g_test_results; \
    ctx.tag = TAG; \
    ctx.completion_semaphore = xSemaphoreCreateBinary(); \
    if (ctx.completion_semaphore == nullptr) { \
        ESP_LOGE(TAG, "Failed to create semaphore for test: %s", name); \
        RUN_TEST(func); \
    } else { \
        BaseType_t created = xTaskCreate(test_task_trampoline, name, \
                                         (stack_size_bytes) / sizeof(StackType_t), \
                                         &ctx, (priority), nullptr); \
        if (created != pdPASS) { \
            ESP_LOGE(TAG, "Failed to create test task: %s", name); \
            vSemaphoreDelete(ctx.completion_semaphore); \
            RUN_TEST(func); \
        } else { \
            if (xSemaphoreTake(ctx.completion_semaphore, pdMS_TO_TICKS(30000)) == pdTRUE) { \
                ESP_LOGI(TAG, "Test task completed: %s", name); \
            } else { \
                ESP_LOGW(TAG, "Test task timeout: %s", name); \
            } \
            vSemaphoreDelete(ctx.completion_semaphore); \
            flip_test_progress_indicator(); \
            vTaskDelay(pdMS_TO_TICKS(100)); \
        } \
    } \
} while (0)

/**
 * @brief Run a test section with indicator pattern
 */
#define RUN_TEST_SECTION_IF_ENABLED(enabled, section_name, ...) \
do { \
    if (enabled) { \
        ESP_LOGI(TAG, ""); \
        ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗"); \
        ESP_LOGI(TAG, "║ SECTION: %-69s║", section_name); \
        ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝"); \
        output_section_indicator(5); \
        __VA_ARGS__ \
    } \
} while (0)

/**
 * @brief Run a test section with custom blink pattern
 */
#define RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(enabled, section_name, blink_count, ...) \
do { \
    if (enabled) { \
        ESP_LOGI(TAG, ""); \
        ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗"); \
        ESP_LOGI(TAG, "║ SECTION: %-69s║", section_name); \
        ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝"); \
        output_section_indicator(blink_count); \
        __VA_ARGS__ \
    } \
} while (0)

//=============================================================================
// TEST REPORTING
//=============================================================================

/**
 * @brief Print section configuration status
 */
inline void print_test_section_status(const char* tag, const char* module_name) {
    ESP_LOGI(tag, "");
    ESP_LOGI(tag, "╔══════════════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(tag, "║ %s TEST CONFIGURATION                                                    ", module_name);
    ESP_LOGI(tag, "╠══════════════════════════════════════════════════════════════════════════════╣");
    ESP_LOGI(tag, "║ Test sections will execute based on compile-time configuration               ║");
    ESP_LOGI(tag, "║ GPIO14 test progression indicator: ENABLED                                   ║");
    ESP_LOGI(tag, "╚══════════════════════════════════════════════════════════════════════════════╝");
    ESP_LOGI(tag, "");
}

