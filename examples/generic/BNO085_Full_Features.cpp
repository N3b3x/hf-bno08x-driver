/**
 * @file BNO085_Full_Features.cpp
 * @brief Comprehensive demonstration of most BNO085 driver capabilities.
 *
 * This example shows how to:
 *  - implement a transport layer (I2C)
 *  - enable a variety of sensor reports
 *  - use both event callbacks and polling methods
 *  - read orientation, acceleration and gesture data
 */

#include "BNO085.hpp"
#include "BNO085_Transport.hpp"

// Example I2C transport implementation (simplified pseudocode).
class I2CTransport : public IBNO085Transport {
public:
  I2CTransport(I2C_HandleTypeDef *bus, uint8_t address, GPIO_TypeDef *intPort = nullptr,
               uint16_t intPin = 0)
      : i2c(bus), addr(address), intPort(intPort), intPin(intPin) {}

  bool open() override {
    // Assume peripheral already initialised elsewhere.
    return true;
  }
  void close() override {}
  int write(const uint8_t *data, uint32_t len) override {
    return HAL_I2C_Master_Transmit(i2c, addr, (uint8_t *)data, len, HAL_MAX_DELAY) == HAL_OK ? len
                                                                                             : -1;
  }
  int read(uint8_t *data, uint32_t len) override {
    return HAL_I2C_Master_Receive(i2c, addr, data, len, 10) == HAL_OK ? len : -1;
  }
  bool dataAvailable() override {
    if (!intPort)
      return true;
    return HAL_GPIO_ReadPin(intPort, intPin) == GPIO_PIN_RESET;
  }
  void delay(uint32_t ms) override { HAL_Delay(ms); }
  uint32_t getTimeUs() override { return HAL_GetTick() * 1000; }

private:
  I2C_HandleTypeDef *i2c;
  uint8_t addr;
  GPIO_TypeDef *intPort;
  uint16_t intPin;
};

// Instantiate driver objects
I2C_HandleTypeDef hi2c1;                   // from platform HAL
I2CTransport transport(&hi2c1, 0x4A << 1); // using default address 0x4A
BNO085 imu;

// Callback demonstrating event-driven handling
static void handleEvent(const BNO085::SensorEvent &e) {
  switch (e.sensor) {
  case BNO085Sensor::RotationVector:
    printf("Yaw %.1f pitch %.1f roll %.1f\n", e.rotation.z, e.rotation.y, e.rotation.x);
    break;
  case BNO085Sensor::LinearAcceleration:
    printf("Linear accel %.2f %.2f %.2f m/s^2\n", e.vector.x, e.vector.y, e.vector.z);
    break;
  case BNO085Sensor::Gyroscope:
    printf("Gyro %.2f %.2f %.2f rad/s\n", e.vector.x, e.vector.y, e.vector.z);
    break;
  case BNO085Sensor::StepCounter:
    printf("Steps: %lu\n", e.stepCount);
    break;
  case BNO085Sensor::TapDetector:
    if (e.detected)
      printf(e.tap.doubleTap ? "Double tap!\n" : "Tap!\n");
    break;
  default:
    break;
  }
}

int main() {
  if (!imu.begin(&transport)) {
    printf("IMU init failed\n");
    return -1;
  }

  // Enable a broad set of sensors
  imu.enableSensor(BNO085Sensor::RotationVector, 10);     // 100 Hz
  imu.enableSensor(BNO085Sensor::LinearAcceleration, 20); // 50 Hz
  imu.enableSensor(BNO085Sensor::Gyroscope, 20);          // 50 Hz
  imu.enableSensor(BNO085Sensor::Gravity, 50);            // 20 Hz
  imu.enableSensor(BNO085Sensor::StepCounter, 0);         // on change
  imu.enableSensor(BNO085Sensor::TapDetector, 0);         // gesture events

  // Register callback for all events
  imu.setCallback(handleEvent);

  // Main loop mixes callback and polling usage
  while (true) {
    imu.update();

    // Example of polling for data without callback
    if (imu.hasNewData(BNO085Sensor::Gravity)) {
      auto g = imu.getLatest(BNO085Sensor::Gravity);
      printf("Gravity %.2f %.2f %.2f\n", g.vector.x, g.vector.y, g.vector.z);
    }

    transport.delay(5); // or sleep until INT
  }
}
