# Firmware Update

The library includes DFU helpers to update the sensor firmware.

1. Hold **BOOTN** low and reset the sensor to enter bootloader mode.
2. Implement `IDfuTransport` for your bus (I²C, SPI, UART, etc.).
3. Provide a firmware image via `HcBin_t` (`firmware-bno.c` is a stub example).
4. Call `dfu(transport, firmware)` or `BNO085::dfu(firmware)`.

Timeouts are built in for slow links. See [`../src/dfu/README.md`](../src/dfu/README.md) for a complete explanation.

---

[⬅️ Previous: RVC Mode](RvcMode.md) | [Next: Advanced Notes ➡️](AdvancedNotes.md) | [Docs Hub 📚](README.md)
