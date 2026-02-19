###############################################################################
# HF BNO08x Driver - ESP-IDF Component Wrapper
###############################################################################
# Note:
#   cmake_minimum_required() is intentionally omitted here — ESP-IDF's
#   project.cmake already sets it and adding another can cause policy conflicts.
###############################################################################

set(HF_BNO08X_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." CACHE PATH
    "Path to root of hf-bno08x-driver (auto-detected from component location)")

get_filename_component(HF_BNO08X_ROOT "${HF_BNO08X_ROOT}" ABSOLUTE)
if(NOT EXISTS "${HF_BNO08X_ROOT}/cmake/hf_bno08x_build_settings.cmake")
    message(FATAL_ERROR
        "Cannot find BNO08x driver root.\n"
        "  Resolved path: ${HF_BNO08X_ROOT}\n"
        "  Expected file: ${HF_BNO08X_ROOT}/cmake/hf_bno08x_build_settings.cmake\n"
        "  Fix: set HF_BNO08X_ROOT to the correct driver checkout path.")
endif()

include("${HF_BNO08X_ROOT}/cmake/hf_bno08x_build_settings.cmake")

idf_component_register(
    SRCS ${HF_BNO08X_SOURCE_FILES}
    INCLUDE_DIRS ${HF_BNO08X_PUBLIC_INCLUDE_DIRS}
    REQUIRES ${HF_BNO08X_IDF_REQUIRES}
)

# Compiled driver — PRIVATE C++20
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_20)
