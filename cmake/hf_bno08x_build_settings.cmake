#===============================================================================
# BNO08x Driver - Build Settings
# Shared variables for target name, includes, sources, and dependencies.
# This file is the SINGLE SOURCE OF TRUTH for the driver version.
#===============================================================================

include_guard(GLOBAL)

set(HF_BNO08X_TARGET_NAME "hf_bno08x")

#===============================================================================
# Versioning (single source of truth)
#===============================================================================
set(HF_BNO08X_VERSION_MAJOR 1)
set(HF_BNO08X_VERSION_MINOR 0)
set(HF_BNO08X_VERSION_PATCH 0)
set(HF_BNO08X_VERSION "${HF_BNO08X_VERSION_MAJOR}.${HF_BNO08X_VERSION_MINOR}.${HF_BNO08X_VERSION_PATCH}")

#===============================================================================
# Generate version header from template (into build directory)
#===============================================================================
set(HF_BNO08X_VERSION_TEMPLATE "${CMAKE_CURRENT_LIST_DIR}/../inc/bno08x_version.h.in")
set(HF_BNO08X_VERSION_HEADER_DIR "${CMAKE_CURRENT_BINARY_DIR}/hf_bno08x_generated")
set(HF_BNO08X_VERSION_HEADER     "${HF_BNO08X_VERSION_HEADER_DIR}/bno08x_version.h")

file(MAKE_DIRECTORY "${HF_BNO08X_VERSION_HEADER_DIR}")

if(EXISTS "${HF_BNO08X_VERSION_TEMPLATE}")
    configure_file(
        "${HF_BNO08X_VERSION_TEMPLATE}"
        "${HF_BNO08X_VERSION_HEADER}"
        @ONLY
    )
    message(STATUS "BNO08x driver v${HF_BNO08X_VERSION} — generated bno08x_version.h in ${HF_BNO08X_VERSION_HEADER_DIR}")
else()
    message(WARNING "bno08x_version.h.in not found at ${HF_BNO08X_VERSION_TEMPLATE}")
endif()

#===============================================================================
# Public include directories
#===============================================================================
# Vendor C headers live under src/dfu/ and src/sh2/, so src/ subtrees are
# added here.  inc/ has the main C++ driver header.
set(HF_BNO08X_PUBLIC_INCLUDE_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../inc"
    "${CMAKE_CURRENT_LIST_DIR}/../src"
    "${CMAKE_CURRENT_LIST_DIR}/../src/dfu"
    "${CMAKE_CURRENT_LIST_DIR}/../src/sh2"
    "${HF_BNO08X_VERSION_HEADER_DIR}"
)

#===============================================================================
# Source files (compiled vendor C code)
#===============================================================================
set(HF_BNO08X_SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/../src/dfu/firmware-bno.c"
    "${CMAKE_CURRENT_LIST_DIR}/../src/sh2/sh2.c"
    "${CMAKE_CURRENT_LIST_DIR}/../src/sh2/sh2_SensorValue.c"
    "${CMAKE_CURRENT_LIST_DIR}/../src/sh2/sh2_util.c"
    "${CMAKE_CURRENT_LIST_DIR}/../src/sh2/shtp.c"
)

#===============================================================================
# ESP-IDF component dependencies
#===============================================================================
set(HF_BNO08X_IDF_REQUIRES driver freertos)
