#pragma once
#include "rvc.h"
#include <cstdint>

/**
 * @brief Abstract interface for platform specific RVC hardware access.
 */
class IRvcHal {
public:
  virtual ~IRvcHal() = default;
  /**
   * @brief Open and configure the hardware interface.
   * @return RVC_OK on success or a negative error code.
   */
  virtual int open() = 0;
  /**
   * @brief Close the interface and release resources.
   */
  virtual void close() = 0;
  /**
   * @brief Read a sensor event from the device.
   *
   * Implementations should populate @p event with the next available
   * sensor data if one is ready. The method should return 1 when an
   * event was read, 0 when no event is available yet and a negative
   * value on error.
   */
  virtual int read(rvc_SensorEvent_t *event) = 0;
};
