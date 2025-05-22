/**
 * @file BNO085_Event_Driven_Callback.cpp
 * @brief Example of event-driven usage of BNO085 sensor with callback handling.
 */
#include "BNO085.hpp"
#include "BNO085_Transport.hpp"
// (Assume I2CTransport is implemented as shown earlier for the specific platform)

I2C_HandleTypeDef hi2c1;  // instance from MCU HAL (for example)
I2CTransport imuTransport(&hi2c1, 0x4A<<1, GPIOB, GPIO_PIN_12); // using appropriate I2C address and INT pin
BNO085 imu;

int main() {
    // Initialize I2C peripheral (done elsewhere) and then:
    if (!imu.init(&imuTransport)) {
        printf("IMU init failed with error %d\n", imu.getLastError());
        while(true);
    }
    imu.enableSensor(BNO085Sensor::StepCounter, 0);     // enable step counter (0ms means default or on-change)
    imu.enableSensor(BNO085Sensor::TapDetector, 0);     // enable tap detector (event-driven)
    imu.setSensorCallback([&](const BNO085::SensorEvent& evt) {
        switch(evt.sensor) {
            case BNO085Sensor::StepCounter:
                printf("Steps counted: %lu\n", evt.stepCount);
                break;
            case BNO085Sensor::TapDetector:
                if (evt.detected) {
                    if (evt.tap.doubleTap) {
                        printf("Double tap detected!\n");
                    } else {
                        printf("Single tap detected on axis %d\n", evt.tap.direction);
                    }
                }
                break;
            default:
                // handle other events...
                break;
        }
    });
    // In main loop or thread:
    while (true) {
        // Wait for data ready interrupt, or poll at moderate rate
        if (intPinSignaled) {
            imu.update();
            intPinSignaled = false;
        }
        // Sleep or delay until next loop iteration (e.g., 10ms)
        HAL_Delay(10);
    }
}
