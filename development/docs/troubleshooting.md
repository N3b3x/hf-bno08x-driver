# Troubleshooting

This guide helps you diagnose and resolve common issues when using the BNO08x driver.

## Common Error Messages

### Error: Initialization Failed

**Symptoms:**
- `Begin()` returns `false`
- Sensor not responding
- `GetLastError()` returns non-zero

**Causes:**
- Communication interface not properly initialized
- Hardware connections incorrect
- Power supply issues
- Wrong I²C address

**Solutions:**
1. **Verify Communication Interface**: Ensure interface is initialized before calling `Begin()`
2. **Check Connections**: Verify all I²C/SPI connections (SCL, SDA, CS, etc.)
3. **Verify Power**: Check power supply voltage (2.4V - 3.6V, typically 3.3V)
4. **Check I²C Address**: Verify address matches hardware (0x4A or 0x4B)
5. **Check Pull-ups**: Ensure I²C has proper pull-up resistors (4.7 kΩ)
6. **Verify Interface Selection**: Check PS0/PS1 pins match selected interface

---

### Error: No Sensor Data

**Symptoms:**
- `Begin()` succeeds but no data received
- `HasNewData()` always returns false
- Callbacks never called

**Causes:**
- Sensors not enabled
- `Update()` not called frequently enough
- Sensor not calibrated
- Communication errors

**Solutions:**
1. **Enable Sensors**: Call `EnableSensor()` for each sensor you need
2. **Call Update Frequently**: `Update()` must be called as often as possible (every 5-10ms)
3. **Check Sensor Status**: Wait for calibration (accuracy = 3)
4. **Verify Communication**: Check for communication errors with `GetLastError()`
5. **Check Interval**: Ensure report interval is reasonable (not too fast)

---

### Error: Communication Error

**Symptoms:**
- `GetLastError()` returns communication error codes
- Intermittent data loss
- Timeout errors

**Causes:**
- I²C/SPI configuration incorrect
- Signal integrity issues
- Bus speed too high
- Interference

**Solutions:**
1. **Check I²C Speed**: Try lower speed (e.g., 100 kHz instead of 400 kHz)
2. **Verify Pull-ups**: Ensure proper pull-up resistors on I²C bus
3. **Check Wiring**: Verify all connections are secure and wires are short
4. **Reduce Interference**: Keep sensor away from noise sources
5. **Check Bus Loading**: Ensure bus capacitance is within limits

---

### Error: Sensor Not Calibrating

**Symptoms:**
- `accuracy` field stays at 0 or 1
- Orientation data unreliable
- Heading drifts

**Causes:**
- Sensor needs calibration motion
- Magnetic interference
- Sensor not moving
- Calibration timeout

**Solutions:**
1. **Perform Calibration Motion**: Move sensor in figure-8 pattern
2. **Check Magnetic Environment**: Avoid strong magnetic fields
3. **Wait for Calibration**: Calibration can take 10-30 seconds
4. **Check Sensor Placement**: Ensure sensor is not near magnetic materials
5. **Verify Magnetometer**: Check if magnetometer is enabled and working

---

### Error: Wrong Orientation Data

**Symptoms:**
- Euler angles seem incorrect
- Quaternion values unexpected
- Coordinate system mismatch

**Causes:**
- Sensor mounting orientation
- Coordinate system convention
- Calibration issues

**Solutions:**
1. **Check Mounting**: Verify sensor orientation matches expected coordinate system
2. **Apply Transformations**: Rotate quaternion/Euler angles to match your coordinate system
3. **Verify Calibration**: Ensure sensor is fully calibrated (accuracy = 3)
4. **Check Datasheet**: Review sensor coordinate system in datasheet

---

### Error: RVC Mode Not Working

**Symptoms:**
- `BeginRvc()` fails
- No RVC frames received
- UART communication errors

**Causes:**
- Sensor not in RVC mode
- UART configuration incorrect
- Baud rate mismatch
- HAL implementation issues

**Solutions:**
1. **Verify RVC Mode**: Check PS0/PS1 pins are set for RVC mode (PS0=1, PS1=0)
2. **Check Baud Rate**: Ensure UART is configured for 115200 bps (8N1)
3. **Verify HAL**: Check `IRvcHal` implementation is correct
4. **Check Wiring**: Verify UART connections (TX/RX)
5. **Call ServiceRvc**: Ensure `ServiceRvc()` is called frequently

---

### Error: DFU Update Failed

**Symptoms:**
- `Dfu()` returns error code
- Firmware update doesn't complete
- Bootloader not responding

**Causes:**
- Sensor not in bootloader mode
- Firmware image invalid
- Communication timeout
- Transport implementation issues

**Solutions:**
1. **Enter Bootloader**: Hold BOOTN low while resetting sensor
2. **Verify Firmware**: Check firmware image is valid `HcBin_t` object
3. **Check Transport**: Verify `IDfuTransport` implementation is correct
4. **Increase Timeout**: DFU may take several minutes on slow links
5. **Check Communication**: Ensure communication interface works in bootloader mode

---

## Debugging Tips

### Enable Logging

Check your communication interface implementation for logging capabilities:

```cpp
// Example: Add logging to your CommInterface implementation
int Read(uint8_t* data, uint32_t length) {
    int result = /* your read implementation */;
    if (result < 0) {
        printf("Read error: %d\n", result);
    }
    return result;
}
```

### Check Last Error

```cpp
int error = imu.GetLastError();
if (error != 0) {
    printf("Last error: %d\n", error);
    // Error codes are from SH-2 library
}
```

### Verify Sensor Status

```cpp
// Check if sensor is responding
if (!imu.Begin()) {
    printf("Initialization failed\n");
    return;
}

// Check if sensors are enabled
imu.EnableSensor(bno08x::BNO085Sensor::RotationVector, 20);
// Wait a bit, then check for data
```

### Monitor Update Frequency

```cpp
uint32_t last_update = 0;
while (true) {
    uint32_t now = xTaskGetTickCount();
    if (now - last_update > 100) {  // More than 100ms
        printf("Warning: Update() not called frequently enough!\n");
    }
    last_update = now;
    imu.Update();
    vTaskDelay(pdMS_TO_TICKS(5));
}
```

## Common Configuration Mistakes

1. **Forgetting to Enable Sensors**: Must call `EnableSensor()` for each sensor
2. **Not Calling Update()**: `Update()` must be called frequently (every 5-10ms)
3. **Wrong I²C Address**: Check ADR/SA0 pin for correct address (0x4A or 0x4B)
4. **Interface Mismatch**: PS0/PS1 pins must match selected interface
5. **Too Fast Update Rate**: Some sensors have maximum rates (check datasheet)
6. **Not Waiting for Calibration**: Orientation data unreliable until accuracy = 3

## Next Steps

- Review [Hardware Setup](hardware_setup.md) for wiring verification
- Check [Platform Integration](platform_integration.md) for communication interface issues
- See [Examples](examples.md) for working code samples
- Review [API Reference](api_reference.md) for error codes

---

**Navigation**
⬅️ [Examples](examples.md) | [Back to Index](index.md)

