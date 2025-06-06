# RVC Mode

Some sensors expose a simplified "Robot Vacuum Cleaner" mode. To use it:

1. Set the appropriate pins or command to boot into RVC mode (see the device data sheet).
2. Implement `IRvcHal` to read bytes from the UART at 115200 bps.
3. Create a `Rvc` instance or call `BNO085::beginRvc()`.
4. Register a callback with `setRvcCallback()`.
5. Call `serviceRvc()` in your loop to decode frames.

See [`src/rvc/README.md`](../src/rvc/README.md) and [`../examples/RVC_Basic.cpp`](../examples/RVC_Basic.cpp) for more information.

---

[⬅️ Previous: Usage Examples](Examples.md) | [Next: Firmware Update ➡️](FirmwareUpdate.md) | [Docs Hub 📚](README.md)
