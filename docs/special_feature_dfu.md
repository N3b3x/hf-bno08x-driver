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

DFU mode allows you to:
- Update sensor firmware in the field
- Fix firmware bugs without hardware replacement
- Upgrade to newer firmware versions
- Support multiple firmware formats (BNO08x and FSP200/201)

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

Firmware must be provided as an `HcBin_t` object. The driver includes stub firmware files:

- **`src/dfu/firmware-bno.c`**: Stub firmware for BNO08x (compile-time image)
- **`src/dfu/firmware-fsp.c`**: Stub firmware for FSP200/201 (compile-time image)

**Note**: These are minimal stub images. For production, obtain firmware from the sensor vendor.

## Implementation

### Step 1: Implement IDfuTransport Interface

```cpp
#include "src/dfu/IDfuTransport.hpp"

class MyDfuTransport : public IDfuTransport {
public:
    bool open() override {
        // Initialize communication interface for bootloader
        return true;
    }
    
    void close() override {
        // Close communication interface
    }
    
    int write(const uint8_t* data, size_t length) override {
        // Write data to sensor
        return /* your write implementation */;
    }
    
    int read(uint8_t* data, size_t length) override {
        // Read data from sensor
        return /* your read implementation */;
    }
    
    uint32_t getTimeUs() override {
        // Return current time in microseconds
        return /* your time implementation */;
    }
};
```

### Step 2: Use HalTransport Adapter (Alternative)

If you already have an `sh2_Hal_t` implementation, you can use the adapter:

```cpp
#include "src/dfu/HalTransport.hpp"

extern sh2_Hal_t my_hal;
HalTransport transport(&my_hal);

// Use transport with dfu()
```

### Step 3: Perform Firmware Update

```cpp
#include "bno08x.hpp"
#include "src/dfu/firmware.h"  // Provides 'firmware' HcBin_t object

MyComm comm;
bno08x::BNO085<MyComm> imu(comm);

// Enter bootloader mode first (see above)

// Method 1: Using BNO085::Dfu() convenience method
int result = imu.Dfu(firmware);
if (result == SH2_OK) {
    printf("Firmware update successful!\n");
} else {
    printf("Firmware update failed: %d\n", result);
}

// Method 2: Using dfu() function directly
MyDfuTransport transport;
result = dfu(transport, firmware);
```

### Step 4: Using Custom Firmware Source

If firmware is stored elsewhere (e.g., flash, network), use `MemoryFirmware`:

```cpp
#include "src/dfu/MemoryFirmware.hpp"

// Firmware stored in memory/flash
const uint8_t* firmware_data = /* your firmware data */;
size_t firmware_size = /* firmware size */;

MemoryFirmware custom_fw(firmware_data, firmware_size);
HcBin_t* fw_bin = custom_fw.getBin();

int result = imu.Dfu(*fw_bin);
```

## Complete Example

```cpp
#include "bno08x.hpp"
#include "src/dfu/firmware.h"
#include "src/dfu/HalTransport.hpp"

void update_firmware() {
    Esp32Bno08xBus comm(/* config */);
    bno08x::BNO085<Esp32Bno08xBus> imu(comm);
    
    // 1. Enter bootloader mode
    printf("Entering bootloader mode...\n");
    imu.SetBootPin(true);   // BOOTN low
    vTaskDelay(pdMS_TO_TICKS(10));
    imu.HardwareReset(10);  // Reset
    vTaskDelay(pdMS_TO_TICKS(100));
    imu.SetBootPin(false);  // Release BOOTN
    
    // 2. Reinitialize communication for bootloader
    comm->Close();
    vTaskDelay(pdMS_TO_TICKS(100));
    comm->Open();
    
    // 3. Perform firmware update
    printf("Starting firmware update...\n");
    int result = imu.Dfu(firmware);
    
    if (result == SH2_OK) {
        printf("Firmware update successful!\n");
    } else {
        printf("Firmware update failed: %d\n", result);
        return;
    }
    
    // 4. Reset sensor to normal mode
    printf("Resetting sensor to normal mode...\n");
    imu.HardwareReset(10);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 5. Reinitialize for normal operation
    comm->Close();
    vTaskDelay(pdMS_TO_TICKS(100));
    comm->Open();
    
    if (imu.Begin()) {
        printf("Sensor reinitialized successfully\n");
    }
}
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

