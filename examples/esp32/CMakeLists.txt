cmake_minimum_required(VERSION 3.16)

# CRITICAL: Set variables BEFORE any other processing
# This ensures they are available during component configuration
if(DEFINED APP_TYPE)
    message(STATUS "APP_TYPE from command line: ${APP_TYPE}")
else()
    set(APP_TYPE "driver_integration_test")
    message(STATUS "APP_TYPE defaulting to: ${APP_TYPE}")
endif()

if(DEFINED BUILD_TYPE)
    message(STATUS "BUILD_TYPE from command line: ${BUILD_TYPE}")
else()
    set(BUILD_TYPE "Release")
    message(STATUS "BUILD_TYPE defaulting to: ${BUILD_TYPE}")
endif()

# Flexible build system for different app types
# Usage: idf.py build -DAPP_TYPE=<type> -DBUILD_TYPE=<type>
# 
# App types are dynamically read from app_config.yml (single source of truth)
# To see available app types, run: python3 scripts/get_app_info.py list
# To see app details, run: python3 scripts/get_app_info.py info <app_type>
#
# Build types: Debug, Release (default: Release)

# Validate app type by reading from app_config.yml (single source of truth)
execute_process(
    COMMAND python3 "${CMAKE_CURRENT_SOURCE_DIR}/scripts/get_app_info.py" list
    OUTPUT_VARIABLE VALID_APP_TYPES_STRING
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE APP_LIST_RESULT
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

if(NOT APP_LIST_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to read valid app types from app_config.yml")
endif()

# Convert space-separated string to CMake list
string(REPLACE " " ";" VALID_APP_TYPES "${VALID_APP_TYPES_STRING}")

if(NOT APP_TYPE IN_LIST VALID_APP_TYPES)
    message(FATAL_ERROR "Invalid APP_TYPE: ${APP_TYPE}. Valid types: ${VALID_APP_TYPES}")
endif()

# Validate build type
set(VALID_BUILD_TYPES "Debug;Release")
if(NOT BUILD_TYPE IN_LIST VALID_BUILD_TYPES)
    message(FATAL_ERROR "Invalid BUILD_TYPE: ${BUILD_TYPE}. Valid types: ${VALID_BUILD_TYPES}")
endif()

message(STATUS "Building BNO08x app type: ${APP_TYPE}")
message(STATUS "Build type: ${BUILD_TYPE}")
message(STATUS "CMake source dir: ${CMAKE_SOURCE_DIR}")
message(STATUS "CMake current source dir: ${CMAKE_CURRENT_SOURCE_DIR}")

# CRITICAL: Set these as global variables for all components
set(APP_TYPE "${APP_TYPE}" CACHE STRING "App type to build" FORCE)
set(BUILD_TYPE "${BUILD_TYPE}" CACHE STRING "Build type (Debug/Release)" FORCE)

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Set project name based on app type
project(esp32_bno08x_${APP_TYPE}_app)
