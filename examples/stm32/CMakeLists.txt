cmake_minimum_required(VERSION 3.16)
project(bno08x_example)
add_executable(bno08x
    main.cpp
    ../../src/BNO085.cpp
    ../../src/dfu/dfu_bno.cpp
    ../../src/dfu/dfu_fsp200.cpp
    ../../src/rvc/rvc.cpp
    ../../src/dfu/firmware-bno.c
    ../../src/dfu/firmware-fsp.c
    ../../src/sh2/euler.c
    ../../src/sh2/sh2.c
    ../../src/sh2/sh2_SensorValue.c
    ../../src/sh2/sh2_util.c
    ../../src/sh2/shtp.c
)
target_include_directories(bno08x PRIVATE ../../src ../../src/dfu ../../src/sh2 ../../src/rvc ../common)
