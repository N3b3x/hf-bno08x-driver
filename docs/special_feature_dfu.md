---
layout: default
title: "📦 DFU"
description: "Device Firmware Update guide for BNO08x sensors"
nav_order: 9
parent: "📚 Documentation"
permalink: /docs/special_feature_dfu/
---

# DFU (Device Firmware Update)

The BNO08x driver includes support for updating sensor firmware via the Device Firmware Update (DFU) protocol. This allows you to update the sensor's firmware without removing it from your system.

## Overview

DFU is built into the BNO085 driver: use the same **CommInterface** (I²C, SPI, or UART SH-2) and call `imu.Dfu(firmware)`. The sensor must be in bootloader mode (BOOTN held low during reset). **DFU is not available when the transport reports `UARTRVC`** — use I²C, SPI, or UART SH-2.

DFU allows you to:
- Update sensor firmware in the field
- Fix firmware bugs without hardware replacement
- Upgrade to newer firmware versions

## Entering Bootloader Mode

The sensor must be in bootloader mode to perform a firmware update:

1. **Hold BOOTN pin low** (active low)
2. **Reset the sensor** (toggle RSTN pin or power cycle)
3. **Release BOOTN** after reset completes
4. Sensor is now in bootloader mode

**Hardware Method:**
```cpp
// If you have BOOTN and RSTN pins wired
imu.SetBootPin(true);   // BOOTN low
imu.HardwareReset(10);   // Reset for 10ms
imu.SetBootPin(false);   // Release BOOTN
```

**Manual Method:**
- Physically hold BOOTN pin low
- Power cycle or reset the sensor
- Release BOOTN

## Firmware Image Format

Firmware must be provided as an `HcBin_t` object. The driver includes a compile-time stub:

- **`src/dfu/firmware-bno.c`** / **`src/dfu/firmware.h`**: Stub firmware for BNO08x (symbol `firmware`). For production, obtain firmware from the sensor vendor.

For firmware loaded at runtime (e.g. from flash or network), use **`MemoryFirmware`** (see [Firmware Sources](#firmware-sources) below).

## Implementation

Use your existing **CommInterface** (I²C, SPI, or UART — not UARTRVC). Enter bootloader mode, then call `imu.Dfu(fw)`.

### Step 1: Enter bootloader mode

Hold BOOTN low, reset the sensor, then release BOOTN. If your transport implements `SetBoot()` and `SetReset()`, the driver’s `SetBootPin()` and `HardwareReset()` will use them.

### Step 2: Perform firmware update

```cpp
#include "bno08x.hpp"
#include "src/dfu/firmware.h"   // Provides default 'firmware' HcBin_t

// Use your normal transport (e.g. Esp32Bno08xBus)
BNO085<Esp32Bno08xBus> imu(*transport);

// Enter bootloader first: SetBootPin(true), HardwareReset(10), SetBootPin(false)
// Then close/reopen transport if required by your platform.

int result = imu.Dfu(firmware);   // or imu.Dfu(*custom_fw.getBin())
if (result == SH2_OK) {
    printf("Firmware update successful!\n");
} else {
    printf("Firmware update failed: %d\n", result);
}
```

### Step 3: Runtime firmware (MemoryFirmware)

If the image is in memory or flash, use `MemoryFirmware`:

```cpp
#include "src/dfu/MemoryFirmware.hpp"

const uint8_t* fw_data = /* pointer to firmware bytes */;
size_t fw_size = /* size in bytes */;
MemoryFirmware custom_fw(fw_data, fw_size);

int result = imu.Dfu(*custom_fw.getBin());
```

## Complete example (ESP32)

See `examples/esp32/main/dfu_example.cpp` for a full flow. Summary:

```cpp
auto transport = CreateEsp32Bno08xBus(config);
BNO085<Esp32Bno08xBus> imu(*transport);

// 1. Enter bootloader
imu.SetBootPin(true);
vTaskDelay(pdMS_TO_TICKS(10));
imu.HardwareReset(10);
vTaskDelay(pdMS_TO_TICKS(100));
imu.SetBootPin(false);

// 2. Reopen transport for bootloader communication
transport->Close();
vTaskDelay(pdMS_TO_TICKS(100));
transport->Open();

// 3. Run DFU
int result = imu.Dfu(firmware);   // or Dfu(*memory_fw.getBin())

// 4. Reset and reinit for normal operation
imu.HardwareReset(2);
transport->Close();
vTaskDelay(pdMS_TO_TICKS(100));
transport->Open();
imu.Begin();
```

## Timeout Handling

The DFU implementation includes built-in timeouts for slow communication links:
- **Default timeout**: Several minutes (varies by firmware size)
- **Automatic retries**: Built into DFU protocol
- **Progress indication**: Check return codes for status

## Error Codes

DFU returns SH-2 status codes:
- **`SH2_OK` (0)**: Update successful
- **`SH2_ERR` (non-zero)**: Update failed

Common error causes:
- Communication timeout
- Invalid firmware image
- Bootloader not responding
- Checksum mismatch

## Firmware Sources

### Compile-Time Firmware

```cpp
#include "src/dfu/firmware.h"

// Use default firmware object
imu.Dfu(firmware);
```

### Runtime Firmware

```cpp
#include "src/dfu/MemoryFirmware.hpp"

// Load from flash/memory
const uint8_t* fw_data = load_from_flash();
size_t fw_size = get_firmware_size();

MemoryFirmware fw(fw_data, fw_size);
imu.Dfu(*fw.getBin());
```

### Network/File Firmware

```cpp
// Load firmware from file or network
std::vector<uint8_t> fw_data = load_firmware_file("firmware.bin");
MemoryFirmware fw(fw_data.data(), fw_data.size());
imu.Dfu(*fw.getBin());
```

## Best Practices

1. **Backup Current Firmware**: If possible, read current firmware before updating
2. **Verify Firmware Source**: Ensure firmware is from trusted source
3. **Test First**: Test firmware update process in development before field deployment
4. **Handle Errors**: Always check return codes and handle failures gracefully
5. **Power Stability**: Ensure stable power during update (avoid brownouts)
6. **Don't Interrupt**: Never interrupt firmware update process

## Troubleshooting

### Update Fails Immediately

- **Check Bootloader Mode**: Verify sensor is actually in bootloader mode
- **Verify Communication**: Test communication interface works
- **Check Firmware**: Ensure firmware image is valid

### Update Times Out

- **Increase Timeout**: Modify timeout values in DFU source if needed
- **Check Communication Speed**: Try slower communication speed
- **Verify Connection**: Check all connections are secure

### Sensor Doesn't Respond After Update

- **Reset Sensor**: Perform hardware reset after update
- **Reinitialize**: Call `Begin()` again after reset
- **Check Firmware**: Verify firmware is compatible with your sensor model

## Next Steps

- Review [Examples](examples.md) for more usage examples
- Check [Troubleshooting](troubleshooting.md) for common issues
- See [RVC Mode](special_feature_rvc.md) for alternative operation mode

---

**Navigation**
⬅️ [RVC Mode](special_feature_rvc.md) | [Back to Index](index.md)

