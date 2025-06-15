# RVC Mode

[⬅️ Docs Index](../../docs/index.md)

This folder contains a small helper library for **RVC mode** on BNO08x / FSP200 series sensors. RVC ("Robot Vacuum Cleaner") mode is a simplified UART protocol that streams orientation and motion data as fixed frames instead of using the SH-2 command interface.

For a full overview of the sensor see the [BNO085 Complete Guide](../../docs/BNO085_Complete_Guide.md). Firmware update instructions are available in the [DFU framework guide](../dfu/README.md).

## Why use RVC mode?
- No command parsing or feature configuration is required. The sensor outputs data continuously once powered.
- Useful for resource constrained systems that only need yaw/pitch/roll and linear acceleration.
- Compatible with both BNO08x IMUs and CEVA FSP200/FSP201 devices.

## Frame format
Each frame is 19 bytes:

| Offset | Bytes | Description |
|-------:|------:|-------------|
|0|2|Sync bytes `0xAA 0xAA`|
|2|1|`index` sequence counter|
|3|2|`yaw` (little endian, 0.01°/LSB)|
|5|2|`pitch` (little endian, 0.01°/LSB)|
|7|2|`roll` (little endian, 0.01°/LSB)|
|9|2|`acc_x` (little endian, 0.001 g/LSB)|
|11|2|`acc_y` (little endian, 0.001 g/LSB)|
|13|2|`acc_z` (little endian, 0.001 g/LSB)|
|15|1|`mi` Motion Intent|
|16|1|`mr` Motion Request|
|17|1|Checksum of bytes 2‒17|

`mi` and `mr` fields use the values defined in [`rvc.h`](./rvc.h). The checksum is a simple unsigned sum of bytes 2 through 17 modulo 256.

### Motion Intent and Motion Request

The library exposes convenience defines for the possible values of the `mi` and
`mr` fields:

```
MI_UNKNOWN                      0
MI_STATIONARY_WITHOUT_VIBRATION 1
MI_STATIONARY_WITH_VIBRATION    2
MI_IN_MOTION                    3

MR_NO_CONSTRAINT                0
MR_STAY_STATIONARY_REQUIRED     1
MR_NON_URGENT_STAY_STATIONARY   3
MR_URGENT_STATIONARY            4
MR_TIMER_STATIONARY             5
```

Use these to interpret how the sensor believes it is moving and any requested
constraints from the host.

## Using the library
1. Implement `IRvcHal` (or populate a `RvcHalC_t` struct) to read bytes from the
   UART connected to the sensor. An example for the ESP32‑C6 is provided in
   `RvcHalEsp32C6.hpp`.
2. Create a `Rvc` object with your HAL and register a callback with
   `Rvc::setCallback()`.
3. Call `Rvc::open()` and then periodically `Rvc::service()` to decode frames.
4. The UART runs 8N1 at 115200 bps on most devices. Adjust your HAL accordingly.

```c
rvc_SensorValue_t val;
void onFrame(void *, rvc_SensorEvent_t *e) {
    Rvc::decode(&val, e);
    printf("Yaw %.2f\n", val.yaw_deg);
}
```
See [`examples/RVC_Basic.cpp`](../../examples/RVC_Basic.cpp) for a minimal
application and [`src/app/demo_rvc.c`](../app/demo_rvc.c) for a more featureful
reference.  The top-level [README](../../README.md) also contains a short
overview of RVC mode.

## Entering RVC mode
RVC mode is selected at boot time on the sensor. Consult the device data sheet for the exact pin settings or commands. Once the sensor is in RVC mode it continually streams the frame described above at a fixed rate (typically 115200 bps).

See the [documentation index](../../docs/index.md) for links to all guides.

---
Released under the Apache 2.0 license (see [LICENSE](../../LICENSE)).
