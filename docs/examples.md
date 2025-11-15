---
layout: default
title: "💡 Examples"
description: "Complete example walkthroughs for the BNO08x driver"
nav_order: 7
parent: "📚 Documentation"
permalink: /docs/examples/
---

# Examples

This guide provides complete, working examples demonstrating various use cases for the BNO08x driver.

## Example 1: Basic Polling Mode

This example shows the minimal setup required to read orientation and linear acceleration using polling mode.

```cpp
#include "bno08x.hpp"
#include "esp32_bno08x_bus.hpp"

void app_main() {
    // 1. Configure I2C transport
    Esp32Bno08xBus::I2CConfig config;
    config.sda_pin = GPIO_NUM_21;
    config.scl_pin = GPIO_NUM_22;
    config.frequency = 400000;
    config.device_address = 0x4A;
    
    auto transport = std::make_unique<Esp32Bno08xBus>(config);
    if (!transport->Open()) {
        printf("Failed to open I2C transport\n");
        return;
    }
    
    // 2. Create IMU instance
    bno08x::BNO085<Esp32Bno08xBus> imu(*transport);
    
    // 3. Initialize
    if (!imu.Begin()) {
        printf("Failed to initialize BNO085\n");
        return;
    }
    
    // 4. Enable sensors
    imu.EnableSensor(bno08x::BNO085Sensor::RotationVector, 20);      // 50 Hz
    imu.EnableSensor(bno08x::BNO085Sensor::LinearAcceleration, 20); // 50 Hz
    
    // 5. Polling loop
    while (true) {
        imu.Update();
        
        if (imu.HasNewData(bno08x::BNO085Sensor::RotationVector)) {
            auto rot = imu.getLatestData(bno08x::BNO085Sensor::RotationVector);
            auto euler = rot.toEuler();
            printf("Yaw: %.1f°, Pitch: %.1f°, Roll: %.1f°\n", 
                   euler.yaw, euler.pitch, euler.roll);
        }
        
        if (imu.HasNewData(bno08x::BNO085Sensor::LinearAcceleration)) {
            auto accel = imu.getLatestData(bno08x::BNO085Sensor::LinearAcceleration);
            printf("Linear Accel: %.2f %.2f %.2f m/s²\n", 
                   accel.vector.x, accel.vector.y, accel.vector.z);
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
```

### Explanation

1. **I2C Configuration**: Configure the I2C bus with appropriate pins and settings
2. **Transport Creation**: Create the communication transport instance
3. **IMU Initialization**: Initialize the BNO085 sensor
4. **Sensor Enablement**: Enable desired sensors with update rates (interval in milliseconds)
5. **Polling Loop**: Continuously call `Update()` and check for new data using `HasNewData()`

### Expected Output

```
Yaw: 45.2°, Pitch: -2.1°, Roll: 1.5°
Linear Accel: 0.12 0.05 -9.81 m/s²
Yaw: 45.3°, Pitch: -2.0°, Roll: 1.6°
Linear Accel: 0.11 0.06 -9.80 m/s²
```

---

## Example 2: Event-Driven Callback Mode

This example demonstrates using callbacks for event-driven sensor data processing.

```cpp
#include "bno08x.hpp"
#include "esp32_bno08x_bus.hpp"

// Callback function for sensor events
void sensor_callback(const bno08x::BNO085<Esp32Bno08xBus>::SensorEvent& event) {
    switch (event.sensor) {
        case bno08x::BNO085Sensor::RotationVector:
            {
                auto euler = event.toEuler();
                printf("Orientation - Yaw: %.1f°, Pitch: %.1f°, Roll: %.1f°\n",
                       euler.yaw, euler.pitch, euler.roll);
            }
            break;
            
        case bno08x::BNO085Sensor::StepCounter:
            printf("Step Count: %lu\n", event.stepCount);
            break;
            
        case bno08x::BNO085Sensor::TapDetector:
            if (event.detected) {
                printf("%s\n", event.tap.doubleTap ? "Double Tap!" : "Tap!");
            }
            break;
            
        case bno08x::BNO085Sensor::ShakeDetector:
            if (event.detected) {
                printf("Shake Detected!\n");
            }
            break;
            
        default:
            break;
    }
}

void app_main() {
    // ... I2C setup (same as Example 1)
    
    bno08x::BNO085<Esp32Bno08xBus> imu(*transport);
    if (!imu.Begin()) {
        printf("Initialization failed\n");
        return;
    }
    
    // Register callback
    imu.SetCallback(sensor_callback);
    
    // Enable event-driven sensors (interval = 0 for on-change)
    imu.EnableSensor(bno08x::BNO085Sensor::RotationVector, 10);  // 100 Hz
    imu.EnableSensor(bno08x::BNO085Sensor::StepCounter, 0);      // on-change
    imu.EnableSensor(bno08x::BNO085Sensor::TapDetector, 0);      // events
    imu.EnableSensor(bno08x::BNO085Sensor::ShakeDetector, 0);    // events
    
    // Main loop - just call Update() to process events
    while (true) {
        imu.Update();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
```

### Explanation

1. **Callback Function**: Define a callback function that handles different sensor events
2. **Callback Registration**: Register the callback using `SetCallback()`
3. **Event-Driven Sensors**: Enable sensors with interval `0` for on-change/event-driven behavior
4. **Update Loop**: Simply call `Update()` - callbacks are invoked automatically when data arrives

### Expected Output

```
Orientation - Yaw: 45.2°, Pitch: -2.1°, Roll: 1.5°
Orientation - Yaw: 45.3°, Pitch: -2.0°, Roll: 1.6°
Tap!
Step Count: 1
Double Tap!
Shake Detected!
```

---

## Example 3: Full Features - Multiple Sensors

This example demonstrates enabling multiple sensors simultaneously using both callback and polling methods.

```cpp
#include "bno08x.hpp"
#include "esp32_bno08x_bus.hpp"

void event_callback(const bno08x::BNO085<Esp32Bno08xBus>::SensorEvent& event) {
    switch (event.sensor) {
        case bno08x::BNO085Sensor::RotationVector:
            {
                auto euler = event.toEuler();
                printf("Yaw %.1f pitch %.1f roll %.1f\n", 
                       euler.yaw, euler.pitch, euler.roll);
            }
            break;
            
        case bno08x::BNO085Sensor::LinearAcceleration:
            printf("Linear accel %.2f %.2f %.2f m/s²\n", 
                   event.vector.x, event.vector.y, event.vector.z);
            break;
            
        case bno08x::BNO085Sensor::Gyroscope:
            printf("Gyro %.2f %.2f %.2f rad/s\n", 
                   event.vector.x, event.vector.y, event.vector.z);
            break;
            
        case bno08x::BNO085Sensor::StepCounter:
            printf("Steps: %lu\n", event.stepCount);
            break;
            
        case bno08x::BNO085Sensor::TapDetector:
            if (event.detected) {
                printf("%s\n", event.tap.doubleTap ? "Double tap!" : "Tap!");
            }
            break;
            
        default:
            break;
    }
}

void app_main() {
    // ... I2C setup
    
    bno08x::BNO085<Esp32Bno08xBus> imu(*transport);
    if (!imu.Begin()) {
        return;
    }
    
    imu.SetCallback(event_callback);
    
    // Enable multiple sensors with different rates
    imu.EnableSensor(bno08x::BNO085Sensor::RotationVector, 10);     // 100 Hz
    imu.EnableSensor(bno08x::BNO085Sensor::LinearAcceleration, 20); // 50 Hz
    imu.EnableSensor(bno08x::BNO085Sensor::Gyroscope, 20);          // 50 Hz
    imu.EnableSensor(bno08x::BNO085Sensor::Gravity, 50);             // 20 Hz
    imu.EnableSensor(bno08x::BNO085Sensor::StepCounter, 0);         // on-change
    imu.EnableSensor(bno08x::BNO085Sensor::TapDetector, 0);         // events
    
    while (true) {
        imu.Update();
        
        // Poll for gravity (not using callback)
        if (imu.HasNewData(bno08x::BNO085Sensor::Gravity)) {
            auto g = imu.getLatestData(bno08x::BNO085Sensor::Gravity);
            printf("Gravity %.2f %.2f %.2f m/s²\n", 
                   g.vector.x, g.vector.y, g.vector.z);
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
```

### Explanation

This example shows:
- **Mixed Usage**: Using both callbacks and polling in the same application
- **Multiple Sensors**: Enabling several sensors with different update rates
- **Flexible Data Access**: Some sensors use callbacks, others use polling

---

## Example 4: Error Handling

This example shows proper error handling and recovery.

```cpp
#include "bno08x.hpp"
#include "esp32_bno08x_bus.hpp"

void app_main() {
    // ... I2C setup
    
    bno08x::BNO085<Esp32Bno08xBus> imu(*transport);
    
    // Initialize with error checking
    if (!imu.Begin()) {
        printf("ERROR: Failed to initialize BNO085\n");
        printf("Check I2C connections and address (0x4A or 0x4B)\n");
        return;
    }
    
    printf("BNO085 initialized successfully\n");
    
    // Enable sensor with error checking
    if (!imu.EnableSensor(bno08x::BNO085Sensor::RotationVector, 10)) {
        printf("ERROR: Failed to enable Rotation Vector sensor\n");
        return;
    }
    
    printf("Rotation Vector enabled at 100 Hz\n");
    
    // Main loop with error recovery
    uint32_t error_count = 0;
    while (true) {
        imu.Update();
        
        // Check for sensor reset (auto-recovery)
        if (imu.WasReset()) {
            printf("WARNING: Sensor reset detected, re-enabling sensors...\n");
            imu.EnableSensor(bno08x::BNO085Sensor::RotationVector, 10);
            error_count++;
            
            if (error_count > 10) {
                printf("ERROR: Too many resets, stopping\n");
                break;
            }
        }
        
        if (imu.HasNewData(bno08x::BNO085Sensor::RotationVector)) {
            auto rot = imu.getLatestData(bno08x::BNO085Sensor::RotationVector);
            auto euler = rot.toEuler();
            printf("Yaw: %.1f°\n", euler.yaw);
            error_count = 0; // Reset error count on successful read
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

### Explanation

1. **Initialization Checks**: Verify `Begin()` returns success
2. **Sensor Enable Checks**: Verify sensor enablement succeeds
3. **Reset Detection**: Use `WasReset()` to detect sensor resets
4. **Auto-Recovery**: Re-enable sensors after reset
5. **Error Counting**: Track errors and stop if too many occur

---

## Running the Examples

### ESP32

```bash
cd examples/esp32
idf.py build flash monitor
```

### Other Platforms

For other platforms, implement the `CommInterface` for your platform (see [Platform Integration](platform_integration.md)) and compile with C++11 or newer:

```bash
g++ -std=c++11 -I inc/ your_code.cpp src/bno08x.cpp -o test
```

## Next Steps

- Review the [API Reference](api_reference.md) for method details
- Check [Troubleshooting](troubleshooting.md) if you encounter issues
- Explore the [examples directory](../examples/) for more examples
- Learn about [RVC Mode](special_feature_rvc.md) for simplified UART streaming
- Check [DFU](special_feature_dfu.md) for firmware update capabilities

---

**Navigation**
⬅️ [API Reference](api_reference.md) | [Next: RVC Mode ➡️](special_feature_rvc.md) | [Back to Index](index.md)
