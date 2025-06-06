# Porting Guide

Implement `IBNO085Transport` for your platform. Below are snippets for common targets.

## ESP32 (ESP-IDF)
```cpp
#include "driver/i2c.h"
class Esp32I2CTransport : public IBNO085Transport {
  // Provide open(), read(), write() and getTimeUs()
};
```

## Arduino
```cpp
#include <Wire.h>
class ArduinoTransport : public IBNO085Transport {
  bool open() override {
    Wire.begin();
    Wire.setClock(400000);
    delay(50);
    return true;
  }
  int read(uint8_t* b, size_t n) override {
    Wire.requestFrom(0x4A, n);
    size_t c = 0;
    while (Wire.available()) b[c++] = Wire.read();
    return c;
  }
  int write(const uint8_t* b, size_t n) override {
    Wire.beginTransmission(0x4A);
    Wire.write(b, n);
    return Wire.endTransmission() == 0 ? n : -1;
  }
  uint32_t getTimeUs() override { return micros(); }
};
```

Use `update()` in your loop or task and enable the sensors you need with `enableSensor()`.

---

[⬅️ Previous: Hardware Wiring](HardwareWiring.md) | [Next: Usage Examples ➡️](Examples.md) | [Docs Hub 📚](README.md)
