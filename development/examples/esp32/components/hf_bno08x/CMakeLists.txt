# =============================================================================
# BNO08x Driver Component - CMakeLists.txt
# =============================================================================
# This component provides the BNO08x driver library
# =============================================================================

cmake_minimum_required(VERSION 3.16)

# =============================================================================
# Path Configuration
# =============================================================================
# Set the source root path relative to this component directory
set(SRC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../src")
set(INC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../inc")

message(STATUS "BNO08x Driver - ESP32 Component Configuration")
message(STATUS "Source Root: ${SRC_ROOT}")

# =============================================================================
# Source File Discovery and Validation
# =============================================================================
# List all potential driver source files
# Note: bno08x.cpp is excluded - it's a template implementation included from bno08x.hpp
set(DRIVER_SRCS
    # bno08x.cpp is a template implementation included from bno08x.hpp (not compiled separately)
    # RVC and DFU logic are integrated directly into the BNO085 class
    "${SRC_ROOT}/dfu/firmware-bno.c"          # Compiled firmware stub image
    "${SRC_ROOT}/sh2/sh2.c"                   # Vendor SH-2 core API
    "${SRC_ROOT}/sh2/sh2_SensorValue.c"       # Vendor sensor value decoding
    "${SRC_ROOT}/sh2/sh2_util.c"              # Vendor byte utilities
    "${SRC_ROOT}/sh2/shtp.c"                  # Vendor transport protocol
    # Note: sh2/euler.c is kept in the vendor directory but not compiled.
)

# Check which source files actually exist and add them
set(EXISTING_SRCS "")
foreach(src_file ${DRIVER_SRCS})
    if(EXISTS "${src_file}")
        list(APPEND EXISTING_SRCS "${src_file}")
        message(STATUS "Found source: ${src_file}")
    else()
        message(WARNING "Source file not found: ${src_file}")
    endif()
endforeach()

# =============================================================================
# Include Directories Configuration
# =============================================================================
# Define include directories for the driver component
set(DRIVER_INCLUDE_DIRS
    "${SRC_ROOT}"                       # Main driver source directory
    "${SRC_ROOT}/dfu"                   # DFU firmware headers (HcBin.h, firmware.h)
    "${SRC_ROOT}/sh2"                   # Vendor SH2 headers
    "${INC_ROOT}"                       # Main driver headers (bno08x.hpp, interfaces)
)

# =============================================================================
# Component Dependencies
# =============================================================================
# Define required ESP-IDF components for the driver
set(DRIVER_REQUIRES
    driver              # ESP-IDF driver framework
    freertos            # FreeRTOS support
)

# =============================================================================
# Component Registration
# =============================================================================
# Register the driver component with ESP-IDF build system
idf_component_register(
    SRCS ${EXISTING_SRCS}                 # Source files (validated above)
    INCLUDE_DIRS ${DRIVER_INCLUDE_DIRS}   # Include directories defined above
    REQUIRES ${DRIVER_REQUIRES}           # Component dependencies defined above
)

# =============================================================================
# Compiler Configuration
# =============================================================================
# Set C++ standard
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)

# =============================================================================
# Compiler Flags
# =============================================================================
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall                    # Enable all warnings
    -Wextra                  # Enable extra warnings
    -O2                      # Optimize for performance
    -g                       # Include debug information
)
