/*
 * Copyright 2020-21 CEVA, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and
 * any applicable agreements you may have with CEVA, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>

#include "RvcHal.hpp"

#include "rvc.h"

/// Pointer to the active HAL implementation.
static IRvcHal *g_hal = nullptr;
/// Adapter used when a C style HAL is supplied.
struct CAdapterHal : IRvcHal {
  RvcHalC_t *c;
  explicit CAdapterHal(RvcHalC_t *h) : c(h) {}
  int open() override { return c->open ? c->open(c->ctx) : RVC_ERR; }
  void close() override {
    if (c->close)
      c->close(c->ctx);
  }
  int read(rvc_SensorEvent_t *e) override {
    return c->read ? c->read(c->ctx, e) : RVC_ERR;
  }
};
static CAdapterHal c_adapter(nullptr);

static rvc_Callback_t *pRvcCallback = nullptr;
void *rvcCookie = nullptr;

rvc_SensorEvent_t sensorEvent;

// initialize RVC subsystem
int rvc_init(IRvcHal *hal) {
  g_hal = hal;
  // Clear callback registration
  pRvcCallback = nullptr;
  rvcCookie = nullptr;

  return RVC_OK;
}

// initialize using a C style HAL
int rvc_init_c(RvcHalC_t *hal) {
  c_adapter.c = hal;
  if (hal)
    return rvc_init(&c_adapter);
  return rvc_init(nullptr);
}

// register a sensor callback function with the RVC subsystem
int rvc_setCallback(rvc_Callback_t *pCallback, void *cookie) {
  pRvcCallback = pCallback;
  rvcCookie = cookie;

  return RVC_OK;
}

// open the RVC interface (starts sensor events)
int rvc_open() {
  if (!g_hal)
    return RVC_ERR;
  return g_hal->open();
}

// close the RVC interface (ends sensor events)
void rvc_close() {
  if (g_hal)
    g_hal->close();
}

#define SKIP_COUNT (1000000)

// periodically service the RVC subsystem.
// must be called periodically to service the RVC UART, parse RVC messages
// and call the RVC callback on each sensor event.
void rvc_service() {
  rvc_SensorEvent_t event;

  bool done = false;
  while (!done) {
    if (!g_hal)
      return;
    int status = g_hal->read(&event);

    if (status > 0) {
      // we have a frame
      if (pRvcCallback != NULL) {
        // Deliver this event to the RVC client
        pRvcCallback(rvcCookie, &event);
      }
    } else {
      done = true;
    }
  }
}

// Convert from SensorEvent (integer, fixed-point representation)
// to SensorValue (float, degrees and g's)
void rvc_decode(rvc_SensorValue_t *value, const rvc_SensorEvent_t *event) {
  value->index = event->index;
  value->yaw_deg = 0.01 * event->yaw;
  value->pitch_deg = 0.01 * event->pitch;
  value->roll_deg = 0.01 * event->roll;
  value->acc_x_g = 0.001 * event->acc_x;
  value->acc_y_g = 0.001 * event->acc_y;
  value->acc_z_g = 0.001 * event->acc_z;
  value->mi = event->mi;
  value->mr = event->mr;

  value->timestamp_uS = event->timestamp_uS;
}
