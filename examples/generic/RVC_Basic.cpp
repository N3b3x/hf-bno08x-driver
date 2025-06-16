/**
 * @file RVC_Basic.cpp
 * @brief Minimal example showing how to read frames in RVC mode.
 */

#include "BNO085.hpp"
#include "RvcHalEsp32C6.hpp" // Replace with your platform HAL
#include <cstdio>

static void onFrame(const rvc_SensorValue_t &v) {
    printf("Yaw %.2f Pitch %.2f Roll %.2f\n", v.yaw_deg, v.pitch_deg,
           v.roll_deg);
}

int main() {
    Esp32C6RvcHal hal; // Configure pins/port as needed
    BNO085 imu;
    imu.setRvcCallback(onFrame);
    if (!imu.beginRvc(&hal))
        return 1;

    while (true) {
        imu.serviceRvc();
        // Add small delay if running on a busy system
    }
    return 0;
}
