#pragma once
#include "rvc.h"

/**
 * @brief Convenience C++ wrapper around the C RVC API.
 *
 * The C API uses RvcHalC_t function pointers internally. The CRTP-based
 * RvcHalInterface bridge is handled at the BNO085 class level, which
 * creates a RvcHalC_t adapter from the user's CRTP-derived HAL type.
 */
class Rvc {
public:
  Rvc() = default;

  /// Initialize with a C style HAL.
  explicit Rvc(RvcHalC_t* hal) {
    rvc_init_c(hal);
  }

  /// Set or change the HAL using a C style implementation.
  void SetHal(RvcHalC_t* hal) {
    rvc_init_c(hal);
  }

  /// Register a callback for received frames.
  int SetCallback(rvc_Callback_t* cb, void* cookie = nullptr) {
    return rvc_setCallback(cb, cookie);
  }

  /// Begin reading frames using the HAL.
  int Open() {
    return rvc_open();
  }

  /// Stop reading frames.
  void Close() {
    rvc_close();
  }

  /// Poll the UART and dispatch any available frames.
  void Service() {
    rvc_service();
  }

  /// Helper to decode a raw event to floating point values.
  static void Decode(rvc_SensorValue_t* val, const rvc_SensorEvent_t* ev) {
    rvc_decode(val, ev);
  }
};
