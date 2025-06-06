# Usage Examples

## Quick Start
```cpp
BNO085 imu(new ArduinoTransport());
if (!imu.begin()) {
  Serial.println("IMU not found!");
  while (1) ;
}
imu.enableSensor(BNO085Sensor::RotationVector, 10); // 100 Hz
imu.enableSensor(BNO085Sensor::StepCounter, 0);     // on-change
imu.setCallback([](const SensorEvent& e) {
  if (e.sensor == BNO085Sensor::RotationVector) {
    Serial.printf("Yaw %.1f\n", e.toEuler().yaw);
  }
});
```
Call `imu.update()` as often as possible or when the INT line triggers.

## Polling Loop
```cpp
while (true) {
  imu.update();
  if (imu.hasNewData(BNO085Sensor::TapDetector)) {
    auto tap = imu.getLatestData(BNO085Sensor::TapDetector);
    Serial.println(tap.doubleTap ? "Double Tap" : "Tap");
  }
  delay(5);
}
```

---

[Porting Guide](PortingGuide.md) | [RVC Mode](RvcMode.md) | [Back to Documentation Hub](README.md)
