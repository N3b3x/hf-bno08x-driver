#include "../inc/bno08x.hpp"
#include <cstdio>
#include <vector>

// Provide minimal link stubs for template-instantiated SH-2 close path.
extern "C" void sh2_close(void) {}

// Provide a local firmware symbol for default-argument linkage completeness.
static int stub_open(void) {
  return -1;
}
static int stub_close(void) {
  return 0;
}
static const char* stub_meta(const char*) {
  return nullptr;
}
static uint32_t stub_len(void) {
  return 0;
}
static uint32_t stub_pkt_len(void) {
  return 0;
}
static int stub_data(uint8_t*, uint32_t, uint32_t) {
  return -1;
}
const HcBin_t firmware = {stub_open, stub_close, stub_meta, stub_len, stub_pkt_len, stub_data};

class MockComm : public bno08x::CommInterface<MockComm> {
public:
  explicit MockComm(BNO085Interface iface = BNO085Interface::I2C) : iface_(iface) {}

  BNO085Interface GetInterfaceType() noexcept {
    return iface_;
  }

  bool Open() noexcept {
    ++open_calls;
    opened = true;
    return true;
  }

  void Close() noexcept {
    ++close_calls;
    opened = false;
  }

  int Write(const uint8_t*, uint32_t length) noexcept {
    if (!opened)
      return -1;
    ++write_calls;
    pending_ack_count++;
    bytes_written += length;
    return static_cast<int>(length);
  }

  int Read(uint8_t* data, uint32_t length) noexcept {
    if (!opened)
      return -1;
    if (length == 0)
      return 0;
    ++read_calls;
    if (pending_ack_count > 0) {
      data[0] = ack_byte;
      --pending_ack_count;
      return 1;
    }
    return 0;
  }

  bool DataAvailable() noexcept {
    return true;
  }

  void Delay(uint32_t ms) noexcept {
    time_us += ms * 1000U;
  }

  uint32_t GetTimeUs() noexcept {
    time_us += 100U;
    return time_us;
  }

  void SetReset(bool state) noexcept {
    if (state) {
      ++reset_assert_calls;
    } else {
      ++reset_release_calls;
    }
  }

  void SetBoot(bool state) noexcept {
    boot_history.push_back(state);
  }

  void SetWake(bool) noexcept {}
  void SetPS0(bool) noexcept {}
  void SetPS1(bool) noexcept {}

  BNO085Interface iface_;
  bool opened{false};
  uint8_t ack_byte{'s'};
  uint32_t time_us{0};
  int pending_ack_count{0};

  int open_calls{0};
  int close_calls{0};
  int write_calls{0};
  int read_calls{0};
  int reset_assert_calls{0};
  int reset_release_calls{0};
  uint32_t bytes_written{0};
  std::vector<bool> boot_history{};
};

static int g_failures = 0;

static void expect_true(bool cond, const char* msg) {
  if (!cond) {
    std::printf("FAIL: %s\n", msg);
    ++g_failures;
  }
}

int main() {
  std::vector<uint8_t> fw(1024, 0x5AU);
  DfuMemoryImage image{fw.data(), static_cast<uint32_t>(fw.size()), "BNO_V1", "1000-3608", 16};

  {
    MockComm comm;
    BNO085<MockComm> imu(comm);

    expect_true(imu.GetState() == BNO085DriverState::Closed, "initial state must be Closed");
    expect_true(!imu.HasNewData(BNO085Sensor::RotationVector),
                "HasNewData must be false outside Sh2Active");

    auto latest = imu.GetLatest(BNO085Sensor::RotationVector);
    expect_true(latest.timestamp == 0, "GetLatest should return default event outside Sh2Active");

    expect_true(imu.EnterBootloader(10, 1), "EnterBootloader should succeed on I2C transport");
    expect_true(!comm.boot_history.empty() && comm.boot_history.front(),
                "EnterBootloader should assert BOOTN low first");
    expect_true(comm.boot_history.size() >= 2 && !comm.boot_history.back(),
                "EnterBootloader should release BOOTN high");
    expect_true(comm.reset_assert_calls >= 1 && comm.reset_release_calls >= 1,
                "EnterBootloader should pulse reset");
    expect_true(imu.GetLastError() == SH2_OK, "EnterBootloader should set SH2_OK");

    uint32_t progress_calls = 0;
    DfuProgress last_progress{};
    DfuOptions options{};
    options.packetLenOverride = 32;
    options.progress = [&](const DfuProgress& p) {
      ++progress_calls;
      last_progress = p;
    };

    int status = imu.DfuFromMemory(image, options);
    expect_true(status == SH2_OK, "DfuFromMemory should succeed with valid image");
    expect_true(progress_calls >= 2, "Progress callback should be invoked at least twice");
    expect_true(last_progress.bytesSent == image.length && last_progress.totalBytes == image.length,
                "Progress callback should finish at full image length");
    expect_true(imu.GetState() == BNO085DriverState::Closed,
                "State should return to Closed after DFU");

    DfuMemoryImage bad_part = image;
    bad_part.partNumber = "9999-0000";
    int bad_status = imu.DfuFromMemory(bad_part);
    expect_true(bad_status == SH2_ERR_BAD_PARAM, "Unknown part should fail strict part validation");

    DfuMemoryImage custom_part = image;
    custom_part.partNumber = "CUSTOM-PART";
    DfuOptions custom_opts{};
    custom_opts.requiredPartNumber = "CUSTOM-PART";
    int custom_status = imu.DfuFromMemory(custom_part, custom_opts);
    expect_true(custom_status == SH2_OK, "Exact requiredPartNumber match should pass");

    int flow_status = imu.RunDfuFromMemory(image);
    expect_true(flow_status == SH2_OK, "RunDfuFromMemory workflow should succeed");
    expect_true(comm.reset_assert_calls >= 3,
                "Workflow should pulse reset for bootloader entry and reboot");

    expect_true(imu.ExitBootloaderAndReboot(2, 1),
                "ExitBootloaderAndReboot should succeed on I2C transport");
  }

  {
    MockComm rvc_comm(BNO085Interface::UARTRVC);
    BNO085<MockComm> imu(rvc_comm);
    expect_true(!imu.EnterBootloader(), "EnterBootloader should fail on UARTRVC interface");
    expect_true(imu.GetLastError() == SH2_ERR_BAD_PARAM,
                "UARTRVC EnterBootloader should set SH2_ERR_BAD_PARAM");
    int status = imu.DfuFromMemory(image);
    expect_true(status == SH2_ERR, "DFU should be rejected on UARTRVC interface");
  }

  if (g_failures == 0) {
    std::printf("All DFU workflow tests passed.\n");
    return 0;
  }

  std::printf("%d DFU workflow test(s) failed.\n", g_failures);
  return 1;
}
