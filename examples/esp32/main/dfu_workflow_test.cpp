/**
 * @file dfu_workflow_test.cpp
 * @brief DFU workflow integration test for BNO08x on ESP32
 *
 * This suite validates class-aware DFU features on real transport hardware:
 * - Driver state transitions around DFU APIs
 * - Bootloader helper APIs (EnterBootloader/ExitBootloaderAndReboot)
 * - In-memory DFU APIs and validation behavior
 * - Full workflow helper (RunDfuFromMemory)
 *
 * Notes:
 * - Requires BNO08x hardware connected over I2C.
 * - BOOTN wiring is optional; when BOOTN is not wired, transfer attempts are
 *   expected to fail at protocol level but still exercise state/flow behavior.
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <array>
#include <memory>
#include <vector>

#include "../../../inc/bno08x.hpp"
#include "TestFramework.h"
#include "esp32_bno08x_bus.hpp"

static const char* TAG = "BNO08x_DfuWorkflowTest";

static std::unique_ptr<Esp32Bno08xBus> g_transport;
static std::unique_ptr<BNO085<Esp32Bno08xBus>> g_imu;
static TestResults g_test_results;

static constexpr bool ENABLE_INITIALIZATION_TESTS = true;
static constexpr bool ENABLE_STATE_CONTRACT_TESTS = true;
static constexpr bool ENABLE_BOOTLOADER_HELPER_TESTS = true;
static constexpr bool ENABLE_DFU_MEMORY_API_TESTS = true;
static constexpr bool ENABLE_DFU_WORKFLOW_TESTS = true;

#define CHECK_OR_RETURN(cond, fmt, ...)                                                            \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      ESP_LOGE(TAG, fmt, ##__VA_ARGS__);                                                          \
      return false;                                                                                \
    }                                                                                              \
  } while (0)

static bool create_test_transport() noexcept {
  Esp32Bno08xBus::I2CConfig config;
  config.sda_pin = GPIO_NUM_4;
  config.scl_pin = GPIO_NUM_5;
  config.frequency = 400000;
  config.rst_pin = GPIO_NUM_16;
  config.int_pin = GPIO_NUM_17;

  const std::array<uint8_t, 2> addresses = {0x4B, 0x4A};
  for (size_t i = 0; i < addresses.size(); ++i) {
    config.device_address = addresses[i];
    ESP_LOGI(TAG, "Trying BNO08x at I2C address 0x%02X (SA0=%s)...", config.device_address,
             (i == 0) ? "HIGH" : "LOW");

    g_transport = CreateEsp32Bno08xBus(config);
    if (!g_transport) {
      ESP_LOGW(TAG, "Failed to create I2C transport for 0x%02X", config.device_address);
      continue;
    }

    g_transport->HardwareReset(2, 200);
    if (!g_transport->Probe()) {
      ESP_LOGW(TAG, "I2C probe failed at 0x%02X", config.device_address);
      g_transport.reset();
      continue;
    }

    ESP_LOGI(TAG, "I2C probe succeeded at 0x%02X", config.device_address);
    return true;
  }

  ESP_LOGE(TAG, "Failed to discover BNO08x at 0x4B/0x4A");
  return false;
}

static bool create_test_imu() noexcept {
  CHECK_OR_RETURN(g_transport != nullptr, "Transport not initialized");

  g_imu = std::make_unique<BNO085<Esp32Bno08xBus>>(*g_transport);
  CHECK_OR_RETURN(g_imu != nullptr, "Failed to allocate IMU object");

  CHECK_OR_RETURN(g_imu->Begin(), "Begin() failed: %d", g_imu->GetLastError());
  CHECK_OR_RETURN(g_imu->GetState() == BNO085DriverState::Sh2Active,
                  "Expected Sh2Active after Begin()");
  return true;
}

static bool test_initialization_and_state() noexcept {
  CHECK_OR_RETURN(create_test_transport(), "Transport setup failed");
  CHECK_OR_RETURN(create_test_imu(), "IMU setup failed");
  CHECK_OR_RETURN(g_imu->GetState() == BNO085DriverState::Sh2Active,
                  "State mismatch after initialization");
  return true;
}

static bool test_state_contract_for_queries() noexcept {
  CHECK_OR_RETURN(g_imu != nullptr, "IMU not initialized");

  g_imu->Close();
  CHECK_OR_RETURN(g_imu->GetState() == BNO085DriverState::Closed, "Expected Closed after Close()");

  CHECK_OR_RETURN(!g_imu->HasNewData(BNO085Sensor::RotationVector),
                  "HasNewData must be false outside Sh2Active");

  SensorEvent e = g_imu->GetLatest(BNO085Sensor::RotationVector);
  CHECK_OR_RETURN(e.timestamp == 0, "GetLatest outside Sh2Active should return default event");

  CHECK_OR_RETURN(g_imu->Begin(), "Re-Begin failed: %d", g_imu->GetLastError());
  CHECK_OR_RETURN(g_imu->GetState() == BNO085DriverState::Sh2Active,
                  "Expected Sh2Active after re-Begin");
  return true;
}

static bool test_bootloader_helpers() noexcept {
  CHECK_OR_RETURN(g_imu != nullptr, "IMU not initialized");

  g_imu->Close();
  CHECK_OR_RETURN(g_imu->GetState() == BNO085DriverState::Closed, "Expected Closed before DFU");

  CHECK_OR_RETURN(g_imu->EnterBootloader(10, 20), "EnterBootloader failed: %d",
                  g_imu->GetLastError());
  CHECK_OR_RETURN(g_imu->GetState() == BNO085DriverState::Closed,
                  "EnterBootloader should keep state Closed");

  CHECK_OR_RETURN(g_imu->ExitBootloaderAndReboot(2, 20), "ExitBootloaderAndReboot failed: %d",
                  g_imu->GetLastError());
  CHECK_OR_RETURN(g_imu->GetState() == BNO085DriverState::Closed,
                  "ExitBootloaderAndReboot should keep state Closed");
  return true;
}

static bool test_memory_dfu_api_validation() noexcept {
  CHECK_OR_RETURN(g_imu != nullptr, "IMU not initialized");
  g_imu->Close();

  int status = g_imu->DfuFromMemory(nullptr, 0);
  CHECK_OR_RETURN(status == SH2_ERR_BAD_PARAM, "Expected SH2_ERR_BAD_PARAM for null image");
  CHECK_OR_RETURN(g_imu->GetState() == BNO085DriverState::Closed,
                  "State should remain Closed after validation failure");

  std::array<uint8_t, 64> short_fw{};
  status = g_imu->DfuFromMemory(short_fw.data(), static_cast<uint32_t>(short_fw.size()));
  CHECK_OR_RETURN(status == SH2_ERR_BAD_PARAM, "Expected SH2_ERR_BAD_PARAM for short image");
  CHECK_OR_RETURN(g_imu->GetState() == BNO085DriverState::Closed,
                  "State should remain Closed after short-image validation failure");
  return true;
}

static bool is_transfer_attempt_status_expected(int status) noexcept {
  return status == SH2_OK || status == SH2_ERR_TIMEOUT || status == SH2_ERR_HUB ||
         status == SH2_ERR_IO || status == SH2_ERR;
}

static bool test_dfu_transfer_attempt_and_workflow() noexcept {
  CHECK_OR_RETURN(g_imu != nullptr, "IMU not initialized");
  g_imu->Close();

  std::vector<uint8_t> fw(1024, 0xA5);
  DfuMemoryImage image{};
  image.data = fw.data();
  image.length = static_cast<uint32_t>(fw.size());
  image.format = "BNO_V1";
  image.partNumber = "1000-3608";
  image.preferredPacketLen = 16;

  uint32_t progress_calls = 0;
  DfuProgress last_progress{};
  DfuOptions options{};
  options.packetLenOverride = 16;
  options.progress = [&](const DfuProgress& p) {
    ++progress_calls;
    last_progress = p;
  };

  // Enter bootloader if BOOTN is wired; if not wired, transfer is expected to fail later.
  CHECK_OR_RETURN(g_imu->EnterBootloader(10, 20), "EnterBootloader failed: %d",
                  g_imu->GetLastError());
  int status = g_imu->DfuFromMemory(image, options);
  CHECK_OR_RETURN(status != SH2_ERR_BAD_PARAM,
                  "Valid in-memory image should not fail metadata/length validation");
  CHECK_OR_RETURN(g_imu->GetState() == BNO085DriverState::Closed,
                  "State should return to Closed after DfuFromMemory");

  if (status == SH2_OK) {
    CHECK_OR_RETURN(progress_calls >= 2, "Expected progress callbacks for successful transfer");
    CHECK_OR_RETURN(last_progress.bytesSent == image.length && last_progress.totalBytes == image.length,
                    "Progress counters should end at full image length");
  } else {
    CHECK_OR_RETURN(is_transfer_attempt_status_expected(status),
                    "Unexpected transfer status: %d", status);
    ESP_LOGW(TAG, "Transfer attempt returned %d (acceptable when BOOTN is unwired/dummy image)",
             status);
  }

  int workflow_status = g_imu->RunDfuFromMemory(image, options, 10, 20, 2, 20);
  CHECK_OR_RETURN(workflow_status != SH2_ERR_BAD_PARAM,
                  "RunDfuFromMemory should not fail on metadata/length for valid image");
  CHECK_OR_RETURN(g_imu->GetState() == BNO085DriverState::Closed,
                  "State should be Closed after RunDfuFromMemory");
  return true;
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "");
  ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║                  BNO08x DFU Workflow Integration Test                       ║");
  ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");
  ESP_LOGI(TAG, "");

  print_test_section_status(TAG, "BNO08x DFU Workflow");
  init_test_progress_indicator();

  RUN_TEST_SECTION_IF_ENABLED(
      ENABLE_INITIALIZATION_TESTS, "INITIALIZATION",
      RUN_TEST_IN_TASK("test_initialization_and_state", test_initialization_and_state, 16384, 5););

  RUN_TEST_SECTION_IF_ENABLED(
      ENABLE_STATE_CONTRACT_TESTS, "STATE CONTRACT",
      RUN_TEST_IN_TASK("test_state_contract_for_queries", test_state_contract_for_queries, 16384, 5););

  RUN_TEST_SECTION_IF_ENABLED(
      ENABLE_BOOTLOADER_HELPER_TESTS, "BOOTLOADER HELPERS",
      RUN_TEST_IN_TASK("test_bootloader_helpers", test_bootloader_helpers, 16384, 5););

  RUN_TEST_SECTION_IF_ENABLED(
      ENABLE_DFU_MEMORY_API_TESTS, "MEMORY DFU API VALIDATION",
      RUN_TEST_IN_TASK("test_memory_dfu_api_validation", test_memory_dfu_api_validation, 16384, 5););

  RUN_TEST_SECTION_IF_ENABLED(
      ENABLE_DFU_WORKFLOW_TESTS, "DFU TRANSFER ATTEMPT + WORKFLOW",
      RUN_TEST_IN_TASK("test_dfu_transfer_attempt_and_workflow", test_dfu_transfer_attempt_and_workflow,
                       24576, 5););

  print_test_summary(g_test_results, "BNO08x DFU Workflow", TAG);
  output_section_indicator(5);
  cleanup_test_progress_indicator();

  g_imu.reset();
  g_transport.reset();

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}
