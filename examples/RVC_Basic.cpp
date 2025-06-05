/**
 * @file RVC_Basic.cpp
 * @brief Minimal example showing how to read frames in RVC mode.
 */

#include "rvc/Rvc.hpp"
#include "rvc/RvcHalEsp32C6.hpp" // Replace with your platform HAL
#include <cstdio>

static rvc_SensorValue_t g_val;

static void onFrame(void *, rvc_SensorEvent_t *e) {
    Rvc::decode(&g_val, e);
    printf("Yaw %.2f Pitch %.2f Roll %.2f\n", g_val.yaw_deg, g_val.pitch_deg,
           g_val.roll_deg);
}

int main() {
    Esp32C6RvcHal hal; // Configure pins/port as needed
    Rvc rvc(&hal);
    rvc.setCallback(onFrame);
    if (rvc.open() != RVC_OK)
        return 1;

    while (true) {
        rvc.service();
        // Add small delay if running on a busy system
    }
    return 0;
}
