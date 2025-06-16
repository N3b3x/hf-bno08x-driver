#pragma once
#include "rvc.h"

/**
 * @brief Convenience C++ wrapper around the C RVC API.
 *
 * Construct with an IRvcHal implementation and then call open(), service(),
 * and close() as needed. The class simply forwards to the underlying
 * rvc_* functions.
 */
class Rvc {
public:
  /// Construct the wrapper and optionally initialize with a HAL.
  explicit Rvc(IRvcHal *hal = nullptr) { rvc_init(hal); }

  /// Construct with a C style HAL.
  explicit Rvc(RvcHalC_t *hal) { rvc_init_c(hal); }

  /// Change the HAL after construction.
  void setHal(IRvcHal *hal) { rvc_init(hal); }
  /// Change the HAL using a C style implementation.
  void setHal(RvcHalC_t *hal) { rvc_init_c(hal); }

  /// Register a callback for received frames.
  int setCallback(rvc_Callback_t *cb, void *cookie = nullptr) {
    return rvc_setCallback(cb, cookie);
  }

  /// Begin reading frames using the HAL.
  int open() { return rvc_open(); }

  /// Stop reading frames.
  void close() { rvc_close(); }

  /// Poll the UART and dispatch any available frames.
  void service() { rvc_service(); }

  /// Helper to decode a raw event to floating point values.
  static void decode(rvc_SensorValue_t *val, const rvc_SensorEvent_t *ev) { rvc_decode(val, ev); }
};
