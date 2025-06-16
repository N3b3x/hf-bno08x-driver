/**
 * @file BNO085_Basic_Polling.cpp
 * @brief Example of using BNO085 in polling mode: orientation and linear acceleration.
 */

#include "BNO085.hpp"
#include "BNO085_Transport.hpp"
// (Assume I2CTransport is implemented as shown earlier for the specific platform)

I2C_HandleTypeDef hi2c1; // instance from MCU HAL (for example)
I2CTransport imuTransport(&hi2c1, 0x4A << 1, GPIOB,
                          GPIO_PIN_12); // using appropriate I2C address and INT pin
BNO085 imu;

/**
 * @brief Entry point demonstrating initialization, enabling sensors, and polling loop.
 * @return Program exit code (0 if successful, non-zero on error).
 */
int main() {
  // Initialize I2C peripheral (done elsewhere) and then:
  if (!imu.init(&imuTransport)) {
    printf("IMU init failed with error %d\n", imu.getLastError());
    while (true)
      ;
  }
  // Enable Rotation Vector at 50 Hz (20ms) and Linear Acceleration at 50 Hz
  imu.enableSensor(BNO085Sensor::RotationVector, 20);
  imu.enableSensor(BNO085Sensor::LinearAcceleration, 20);
  // Main loop
  while (true) {
    imu.update(); // poll for new sensor data
    if (imu.hasNewData(BNO085Sensor::RotationVector)) {
      auto rot = imu.getLatestData(BNO085Sensor::RotationVector);
      // Compute Euler angles (in radians) from quaternion:
      float qw = rot.rotation.w, qx = rot.rotation.x;
      float qy = rot.rotation.y, qz = rot.rotation.z;
      float ysqr = qy * qy;
      // yaw (Z axis rotation)
      float t3 = 2.0f * (qw * qz + qx * qy);
      float t4 = 1.0f - 2.0f * (ysqr + qz * qz);
      float yaw = atan2f(t3, t4);
      // pitch (X axis rotation)
      float t2 = 2.0f * (qw * qx - qy * qz);
      t2 = t2 > 1.0f ? 1.0f : (t2 < -1.0f ? -1.0f : t2);
      float pitch = asinf(t2);
      // roll (Y axis rotation)
      float t0 = 2.0f * (qw * qy + qz * qx);
      float t1 = 1.0f - 2.0f * (qx * qx + ysqr);
      float roll = atan2f(t0, t1);
      printf("Orientation YPR (rad): %.2f, %.2f, %.2f\n", yaw, pitch, roll);
    }
    if (imu.hasNewData(BNO085Sensor::LinearAcceleration)) {
      auto lin = imu.getLatestData(BNO085Sensor::LinearAcceleration);
      printf("Linear Accel (m/s^2): X=%.2f Y=%.2f Z=%.2f\n", lin.vector.x, lin.vector.y,
             lin.vector.z);
    }
    // Sleep or delay until next loop iteration (e.g., 10ms)
    HAL_Delay(10);
  }
}
