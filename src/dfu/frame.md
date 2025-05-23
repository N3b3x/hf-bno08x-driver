# DFU Framework

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
