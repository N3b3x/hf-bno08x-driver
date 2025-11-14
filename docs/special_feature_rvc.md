# RVC Mode (Robot Vacuum Cleaner Mode)

RVC mode is a simplified UART streaming protocol that provides basic orientation and motion data without requiring the full SH-2 command interface. This mode is ideal for resource-constrained systems that only need yaw/pitch/roll and linear acceleration.

## Why Use RVC Mode?

- **No Command Parsing**: Sensor outputs data continuously once powered
- **Lower Overhead**: Simplified protocol reduces processing requirements
- **Resource Efficient**: Perfect for systems with limited CPU/memory
- **Simple Integration**: Fixed frame format, no configuration needed

## Frame Format

Each RVC frame is 19 bytes with the following structure:

| Offset | Bytes | Description |
|--------|-------|-------------|
| 0 | 2 | Sync bytes `0xAA 0xAA` |
| 2 | 1 | `index` sequence counter |
| 3 | 2 | `yaw` (little endian, 0.01°/LSB) |
| 5 | 2 | `pitch` (little endian, 0.01°/LSB) |
| 7 | 2 | `roll` (little endian, 0.01°/LSB) |
| 9 | 2 | `acc_x` (little endian, 0.001 g/LSB) |
| 11 | 2 | `acc_y` (little endian, 0.001 g/LSB) |
| 13 | 2 | `acc_z` (little endian, 0.001 g/LSB) |
| 15 | 1 | `mi` Motion Intent |
| 16 | 1 | `mr` Motion Request |
| 17 | 1 | Checksum of bytes 2-17 |

**Checksum**: Simple unsigned sum of bytes 2 through 17 modulo 256.

## Motion Intent and Motion Request

The `mi` (Motion Intent) and `mr` (Motion Request) fields indicate sensor state:

### Motion Intent Values

```cpp
MI_UNKNOWN                      0
MI_STATIONARY_WITHOUT_VIBRATION 1
MI_STATIONARY_WITH_VIBRATION    2
MI_IN_MOTION                    3
```

### Motion Request Values

```cpp
MR_NO_CONSTRAINT                0
MR_STAY_STATIONARY_REQUIRED     1
MR_NON_URGENT_STAY_STATIONARY   3
MR_URGENT_STATIONARY            4
MR_TIMER_STATIONARY             5
```

## Entering RVC Mode

RVC mode is selected at boot time via hardware pins:

- **PS0 = 1, PS1 = 0**: UART RVC mode
- Sensor must be reset after setting pins
- Once in RVC mode, sensor continuously streams frames at 115200 bps (8N1)

## Implementation

### Step 1: Implement IRvcHal Interface

```cpp
#include "src/rvc/RvcHal.hpp"

class MyRvcHal : public IRvcHal {
public:
    bool open() override {
        // Initialize UART at 115200 bps, 8N1
        return true;
    }
    
    void close() override {
        // Close UART
    }
    
    int read(uint8_t* data, size_t length) override {
        // Read bytes from UART
        return uart_read_bytes(uart_port, data, length, 0);
    }
    
    uint32_t getTimeUs() override {
        // Return current time in microseconds
        return esp_timer_get_time();
    }
};
```

### Step 2: Use BNO085 RVC Methods

```cpp
#include "bno08x.hpp"

MyComm comm;
bno08x::BNO085<MyComm> imu(comm);

MyRvcHal rvc_hal;

// Begin RVC mode
if (imu.BeginRvc(&rvc_hal)) {
    // Set callback for decoded frames
    imu.SetRvcCallback([](const rvc_SensorValue_t& val) {
        printf("Yaw: %.2f°, Pitch: %.2f°, Roll: %.2f°\n",
               val.yaw_deg, val.pitch_deg, val.roll_deg);
        printf("Accel: X=%.3f, Y=%.3f, Z=%.3f g\n",
               val.acc_x_g, val.acc_y_g, val.acc_z_g);
    });
    
    // Service loop
    while (true) {
        imu.ServiceRvc();  // Decode incoming frames
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    // Close when done
    imu.CloseRvc();
}
```

### Step 3: Alternative - Use Rvc Class Directly

```cpp
#include "src/rvc/Rvc.hpp"

MyRvcHal hal;
Rvc rvc(hal);

rvc.setCallback([](const rvc_SensorValue_t& val) {
    printf("Yaw: %.2f°\n", val.yaw_deg);
});

if (rvc.open()) {
    while (true) {
        rvc.service();  // Decode frames
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    rvc.close();
}
```

## Complete Example

```cpp
#include "bno08x.hpp"
#include "src/rvc/RvcHal.hpp"

class Esp32RvcHal : public IRvcHal {
private:
    uart_port_t uart_port_;
    
public:
    Esp32RvcHal(uart_port_t port) : uart_port_(port) {}
    
    bool open() override {
        uart_config_t config = {};
        config.baud_rate = 115200;
        config.data_bits = UART_DATA_8_BITS;
        config.parity = UART_PARITY_DISABLE;
        config.stop_bits = UART_STOP_BITS_1;
        config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        
        uart_param_config(uart_port_, &config);
        uart_set_pin(uart_port_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_driver_install(uart_port_, 1024, 0, 0, NULL, 0);
        return true;
    }
    
    void close() override {
        uart_driver_delete(uart_port_);
    }
    
    int read(uint8_t* data, size_t length) override {
        int len = uart_read_bytes(uart_port_, data, length, pdMS_TO_TICKS(100));
        return (len > 0) ? len : 0;
    }
    
    uint32_t getTimeUs() override {
        return esp_timer_get_time();
    }
};

void app_main() {
    Esp32RvcHal hal(UART_NUM_1);
    Esp32Bno08xBus comm(/* config */);
    bno08x::BNO085<Esp32Bno08xBus> imu(comm);
    
    if (imu.BeginRvc(&hal)) {
        imu.SetRvcCallback([](const rvc_SensorValue_t& val) {
            printf("Yaw: %.2f°, Pitch: %.2f°, Roll: %.2f°\n",
                   val.yaw_deg, val.pitch_deg, val.roll_deg);
        });
        
        while (true) {
            imu.ServiceRvc();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}
```

## UART Configuration

- **Baud Rate**: 115200 bps (typical, check datasheet for your device)
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Flow Control**: None

## Data Units

- **Yaw/Pitch/Roll**: Degrees (0.01° per LSB)
- **Acceleration**: g (0.001 g per LSB)
- **Time**: Microseconds

## Limitations

- **Fixed Data**: Only yaw/pitch/roll and linear acceleration
- **No Configuration**: Cannot change report rates or enable other sensors
- **UART Only**: Requires UART interface (not I²C or SPI)
- **Boot-Time Selection**: Must be selected at boot via pins

## When to Use RVC Mode

**Use RVC Mode When:**
- You only need basic orientation data
- System resources are limited
- Simple integration is preferred
- UART interface is available

**Use Full SH-2 Mode When:**
- You need multiple sensor types
- You need configurable report rates
- You need gesture detection or step counting
- You need I²C or SPI interface

## Next Steps

- See [DFU](special_feature_dfu.md) for firmware update guide
- Review [Examples](examples.md) for more usage examples
- Check [Troubleshooting](troubleshooting.md) for RVC mode issues

---

**Navigation**
⬅️ [Examples](examples.md) | [Next: DFU ➡️](special_feature_dfu.md) | [Back to Index](index.md)

