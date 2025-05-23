# RVC Mode

This folder contains a small helper library for **RVC mode** on BNO08x / FSP200 series sensors. RVC ("Robot Vacuum Cleaner") mode is a simplified UART protocol that streams orientation and motion data as fixed frames instead of using the SH-2 command interface.

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

## Using the library
1. Implement `IRvcHal` to read bytes from the UART connected to the sensor. An example for the ESP32‑C6 is provided in `RvcHalEsp32C6.hpp`.
2. Call `rvc_init()` with your HAL instance and register a callback via `rvc_setCallback()`.
3. After `rvc_open()` the library will parse incoming frames. Call `rvc_service()` regularly to process data.

See [`src/app/demo_rvc.c`](../app/demo_rvc.c) for a reference application that prints sensor values.

## Entering RVC mode
RVC mode is selected at boot time on the sensor. Consult the device data sheet for the exact pin settings or commands. Once the sensor is in RVC mode it continually streams the frame described above at a fixed rate (typically 115200 bps).

---
Released under the Apache 2.0 license (see [LICENSE](../../LICENSE)).
