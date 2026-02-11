# Flexible main component for different BNO08x app types
# Determine source file based on APP_TYPE

# Get app type from parent
if(NOT DEFINED APP_TYPE)
    set(APP_TYPE "driver_integration_test")
endif()

# Debug information
message(STATUS "=== BNO08x CMake Debug Information ===")
message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "CMAKE_CURRENT_LIST_DIR: ${CMAKE_CURRENT_LIST_DIR}")
message(STATUS "CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR}")
message(STATUS "APP_TYPE: ${APP_TYPE}")
message(STATUS "==========================================")

# Get source file from centralized configuration
# Use CMAKE_CURRENT_LIST_DIR to get the directory containing this CMakeLists.txt
# Project root is one level up from main/
get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

execute_process(
    COMMAND python3 "${PROJECT_ROOT}/scripts/get_app_info.py" validate "${APP_TYPE}" --project-path "${PROJECT_ROOT}"
    RESULT_VARIABLE VALIDATE_RESULT
    OUTPUT_QUIET
    ERROR_QUIET
    WORKING_DIRECTORY "${PROJECT_ROOT}"
)

if(NOT VALIDATE_RESULT EQUAL 0)
    message(FATAL_ERROR "Unknown APP_TYPE: ${APP_TYPE}")
endif()

execute_process(
    COMMAND python3 "${PROJECT_ROOT}/scripts/get_app_info.py" source_file "${APP_TYPE}" --project-path "${PROJECT_ROOT}"
    OUTPUT_VARIABLE MAIN_SOURCE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE SOURCE_RESULT
    WORKING_DIRECTORY "${PROJECT_ROOT}"
)

if(NOT SOURCE_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to get source file for APP_TYPE: ${APP_TYPE}")
endif()

message(STATUS "Using source file from configuration: ${MAIN_SOURCE}")

# Check if source file exists
set(SOURCE_FILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${MAIN_SOURCE}")
message(STATUS "Looking for source file: ${SOURCE_FILE_PATH}")

# List files in current directory for debugging
file(GLOB DEBUG_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*")
message(STATUS "Files in source directory:")
foreach(DEBUG_FILE ${DEBUG_FILES})
    message(STATUS "  ${DEBUG_FILE}")
endforeach()

if(NOT EXISTS "${SOURCE_FILE_PATH}")
    # Try alternative paths for CI environment
    set(SOURCE_FILE_PATH "${CMAKE_CURRENT_LIST_DIR}/${MAIN_SOURCE}")
    message(STATUS "Trying alternative path: ${SOURCE_FILE_PATH}")
    if(NOT EXISTS "${SOURCE_FILE_PATH}")
        message(FATAL_ERROR "Source file not found: ${MAIN_SOURCE}\n"
                            "Tried paths:\n"
                            "  ${CMAKE_CURRENT_SOURCE_DIR}/${MAIN_SOURCE}\n"
                            "  ${CMAKE_CURRENT_LIST_DIR}/${MAIN_SOURCE}\n"
                            "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}\n"
                            "CMAKE_CURRENT_LIST_DIR: ${CMAKE_CURRENT_LIST_DIR}")
    endif()
endif()

message(STATUS "Using source file: ${SOURCE_FILE_PATH}")

# Determine include paths based on environment
# Use CMAKE_CURRENT_LIST_DIR (where this CMakeLists.txt is) or CMAKE_SOURCE_DIR (project root)
# In CI, src is copied to ../src, in development it's at ../../src or ../../../src
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../src")
    # CI environment - src is at project level
    set(SRC_BASE_PATH "${CMAKE_CURRENT_LIST_DIR}/../src")
elseif(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../../src")
    # Development environment - src is at examples level
    set(SRC_BASE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../src")
elseif(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../../../src")
    # Development environment - src is at driver root
    set(SRC_BASE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../../src")
elseif(EXISTS "${CMAKE_SOURCE_DIR}/src")
    # Try project root
    set(SRC_BASE_PATH "${CMAKE_SOURCE_DIR}/src")
else()
    message(FATAL_ERROR "Cannot find src directory. Checked:")
    message(FATAL_ERROR "  ${CMAKE_CURRENT_LIST_DIR}/../src")
    message(FATAL_ERROR "  ${CMAKE_CURRENT_LIST_DIR}/../../src")
    message(FATAL_ERROR "  ${CMAKE_CURRENT_LIST_DIR}/../../../src")
    message(FATAL_ERROR "  ${CMAKE_SOURCE_DIR}/src")
    message(FATAL_ERROR "CMAKE_CURRENT_LIST_DIR: ${CMAKE_CURRENT_LIST_DIR}")
    message(FATAL_ERROR "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
endif()

message(STATUS "Using source base path: ${SRC_BASE_PATH}")

# Determine required components based on app type
set(COMPONENT_REQUIRES_LIST 
    driver 
    esp_timer 
    freertos 
    nvs_flash 
    hf_bno08x
)

# Add I2C component for I2C transport (ESP-IDF v5.5 uses esp_driver_i2c)
if(APP_TYPE STREQUAL "driver_integration_test" OR 
   APP_TYPE STREQUAL "basic_polling" OR
   APP_TYPE STREQUAL "event_driven_callback" OR
   APP_TYPE STREQUAL "full_features")
    list(APPEND COMPONENT_REQUIRES_LIST esp_driver_i2c)
endif()

# Add SPI component if needed (for future SPI support)
# if(APP_TYPE STREQUAL "spi_example")
#     list(APPEND COMPONENT_REQUIRES_LIST spi_flash)
# endif()

# Bus implementation is header-only (esp32_bno08x_bus.hpp), no separate .cpp needed
idf_component_register(
    SRCS "${MAIN_SOURCE}"
    INCLUDE_DIRS "." "${SRC_BASE_PATH}" "${SRC_BASE_PATH}/dfu" "${SRC_BASE_PATH}/sh2"
    REQUIRES ${COMPONENT_REQUIRES_LIST}
)

# Set C++ standard (BNO08x requires C++17 minimum)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)

# Set compiler flags based on build type
if(BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${COMPONENT_LIB} PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -O0
        -g3
        -DDEBUG
        -DBNO08X_DEBUG
    )
else()
    target_compile_options(${COMPONENT_LIB} PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -O2
        -g
        -DNDEBUG
        -DBNO08X_RELEASE
    )
endif()

# Add compile definitions for each example type
target_compile_definitions(${COMPONENT_LIB} PRIVATE 
    "EXAMPLE_TYPE_${APP_TYPE}=1"
    "BNO08X_ESP32_S3=1"
)

# ESP32-S3 specific optimizations
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    "CONFIG_IDF_TARGET_ESP32S3=1"
)
