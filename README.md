# SafeStep — ESP32-S3 Firmware

TECHIN 515 Final Group Project

## Hardware

| Component | Part | Interface |
|---|---|---|
| MCU | Seeed XIAO ESP32-S3 | — |
| Ultrasonic | HC-SR04 | GPIO (TRIG/ECHO) |
| LiDAR | VL53L1X | I2C |
| IMU | MPU-6050 | I2C |
| Haptic motor | ERM/LRA | GPIO PWM |
| Mic | INMP441 | I2S |

## Project structure

```
src/
  config.h       — pin assignments and tunable thresholds
  sensors.h/cpp  — HC-SR04 + VL53L1X + MPU-6050 read
  haptic.h/cpp   — PWM motor control + alert level logic
  main.cpp       — setup(), loop(), sensor fusion
```

## Next steps (TODOs in main.cpp)

- [ ] BLE notifications when alert level changes
- [ ] I2S audio cues ("obstacle ahead")
- [ ] IMU cane-down detection — suppress alerts when cane is lifted
- [ ] TFLite Micro obstacle classifier
