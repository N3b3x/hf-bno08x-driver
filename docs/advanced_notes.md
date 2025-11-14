---
layout: default
title: "Advanced Notes"
description: "Tare, power tips, pin helpers, and advanced usage patterns"
nav_order: 7
parent: "Documentation Hub"
has_toc: true
permalink: /docs/advanced-notes/
---

# Advanced Notes

[⬅️ Previous: Firmware Update](firmware_update.md) | [Docs Hub 📚](README.md)

- Call `tareNow()` to zero the current orientation.
- Check `event.accuracy` (0–3) before trusting heading data.
- Hold **BOOTN** low during reset to enter DFU mode.
- Disable unused reports to save power.
- Use pin helpers such as `hardwareReset()` when the relevant pins are wired.

---

[⬅️ Previous: Firmware Update](firmware_update.md) | [Docs Hub 📚](README.md)
