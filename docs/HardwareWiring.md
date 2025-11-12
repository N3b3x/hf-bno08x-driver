---
layout: default
title: "Hardware Wiring"
description: "Connect the BNO08x sensor to your MCU board"
nav_order: 2
parent: "Documentation Hub"
has_toc: true
permalink: /docs/hardware-wiring/
---

# Hardware Wiring

[⬅️ Previous: Getting Started](GettingStarted.md) | [Next: Porting Guide ➡️](PortingGuide.md) | [Docs Hub 📚](README.md)

Connect the sensor to your MCU as shown below.

```
MCU 3V3  ──── VIN   BNO085
MCU GND  ──── GND
MCU SCL  ──── SCL   (4.7 kΩ pull-up)
MCU SDA  ──── SDA   (4.7 kΩ pull-up)
MCU GPIO ──── INT   (optional IRQ)
MCU GPIO ──── NRST  (optional reset)
PS0 + PS1 → GND ➡️ selects I²C (tie high for SPI)
ADR/SA0  → GND ➡️ address 0x4A (0x4B if high)
```

Use the **INT** line to wake your application only when data is ready.

---

[⬅️ Previous: Getting Started](GettingStarted.md) | [Next: Porting Guide ➡️](PortingGuide.md) | [Docs Hub 📚](README.md)
