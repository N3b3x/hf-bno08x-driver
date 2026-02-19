---
layout: default
title: "⚙️ CMake Integration"
description: "How to integrate the BNO08x driver into your CMake project"
nav_order: 5
parent: "📚 Documentation"
permalink: /docs/cmake_integration/
---

# BNO08x — CMake Integration Guide

> **Build contract architecture and templates:**
> [CMake Build Contract](../../../../../docs/development/CMAKE_BUILD_CONTRACT.md).

## Quick Start (Generic CMake)

```cmake
add_subdirectory(external/hf-bno08x-driver)
target_link_libraries(my_app PRIVATE hf::bno08x)
```

## ESP-IDF Integration

Component wrapper: `examples/esp32/components/hf_bno08x/`

```cmake
idf_component_register(
    SRCS "app_main.cpp"
    INCLUDE_DIRS "."
    REQUIRES hf_bno08x driver freertos
)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_20)
```

## Build Variables

| Variable | Value |
|----------|-------|
| `HF_BNO08X_TARGET_NAME` | `hf_bno08x` |
| `HF_BNO08X_VERSION` | Current `MAJOR.MINOR.PATCH` |
| `HF_BNO08X_SOURCE_FILES` | Vendor C: sh2.c, shtp.c, firmware-bno.c, ... |
| `HF_BNO08X_IDF_REQUIRES` | `driver freertos` |

---

**Navigation**
⬅️ [Back to Documentation Index](../../DOCUMENTATION_INDEX.md) | [CMake Build Contract ↗](../../../../../docs/development/CMAKE_BUILD_CONTRACT.md)
