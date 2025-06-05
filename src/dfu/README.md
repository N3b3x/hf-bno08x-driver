#DFU Framework

This folder contains a hardware agnostic implementation of the firmware
update (DFU) routines.  Hardware access is abstracted through the
`IDfuTransport` C++ interface.  Applications must supply an implementation
of this interface for their platform and pass it to `dfu()`.

`HalTransport` is provided as an adapter for the vendor supplied
`sh2_Hal_t` structures so existing C HAL implementations can be used
without modification.

The files `dfu_bno.cpp` and `dfu_fsp200.cpp` contain the DFU logic for the
supported firmware formats.  Both now operate solely through the
`IDfuTransport` interface making them independent of the underlying bus
(I²C, SPI, UART, ...).

## Performing an update

1. Reset the sensor into bootloader mode. On most boards this is done by holding
   the **BOOTN** pin low while toggling **NRST**.
2. Create an `IDfuTransport` implementation for your platform or wrap an
   existing `sh2_Hal_t` using `HalTransport`.
3. Include or generate a firmware image (`firmware-bno.c` or `firmware-fsp.c`) and
   link it into your application. These files compile the binary image into the
   global `HcBin_t firmware` object consumed by `dfu()`. Alternatively create
   a custom `HcBin_t`—for example with `MemoryFirmware`—to stream data from
   another source.
4. Call `dfu(transport, firmware)` and wait for it to return `SH2_OK`.  The
   `BNO085` class exposes a convenience wrapper so you can simply call
   `imu.dfu(firmware)` when using the C++ API.

```cpp
HalTransport t(&myHal);
if (dfu(t) == SH2_OK) {
  printf("Update complete!\n");
}
```

Both DFU implementations time out after several minutes to cover slow serial
links. See the source files for details if you need to adjust the timeout.

## More information

This directory only contains minimal stub firmware images. Consult the sensor
vendor for production firmware files. The [top-level README](../../README.md)
also summarises DFU usage in the context of the full library.
