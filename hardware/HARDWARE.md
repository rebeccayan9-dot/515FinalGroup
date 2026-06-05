# SafeStep — Hardware Design Reference (PCB v1)

This document is the authoritative wiring/pinout reference, kept in sync with the
firmware (`src/main.cpp`). For the binary CAD/PCB design files, see
[`hardware/README.md`](README.md).

---

## Block diagram

```
                  ┌─────────────────────────┐
   VL53L1X LiDAR ─┤ I²C (SDA/SCL)            │
   MPU-6050 IMU  ─┤                          │
                  │   Seeed XIAO ESP32-S3    │── USB serial (115200) → host
   3× HC-SR04 ────┤ GPIO TRIG/ECHO           │
                  │                          │
                  │ I²C → PCA9548A mux (0x70)│
                  └────────────┬─────────────┘
                               │ ch0..ch3
                 ┌─────────────┼─────────────┬─────────────┐
              DRV2605L      DRV2605L      DRV2605L      DRV2605L   (all addr 0x5A)
               FRONT          LEFT          RIGHT         TOP
              + ERM          + ERM         + ERM         + ERM motor
```

---

## ESP32-S3 pin assignments

| Signal | GPIO | Notes |
|---|---|---|
| `TRIG_FRONT` | 1 | Front HC-SR04 trigger |
| `ECHO_FRONT` | 2 | Front HC-SR04 echo |
| `TRIG_LEFT` | 3 | Left HC-SR04 trigger |
| `ECHO_LEFT` | 4 | Left HC-SR04 echo |
| `TRIG_RIGHT` | 43 | Right HC-SR04 trigger |
| `ECHO_RIGHT` | 9 | Right HC-SR04 echo |
| I²C SDA | 5 (D4) | XIAO ESP32-S3 default `Wire` SDA |
| I²C SCL | 6 (D5) | XIAO ESP32-S3 default `Wire` SCL |

> These reflect the **PCB v1** mapping defined at the top of `src/main.cpp`. If you
> change the board, update both places.

## I²C address map

| Device | Address | Behind mux? |
|---|---|---|
| VL53L1X LiDAR | `0x29` (default) | no |
| MPU-6050 IMU | `0x68` (default) | no |
| PCA9548A mux | `0x70` | no |
| DRV2605L ×4 | `0x5A` (each) | yes — mux channels 0–3 |

I²C bus clock: **100 kHz** (`Wire.setClock(100000)`).

## Haptic channel map (PCA9548A)

| Mux channel | Location | Purpose |
|---|---|---|
| 0 | FRONT | Forward obstacle (LiDAR/front ultrasonic) |
| 1 | LEFT | Left obstacle |
| 2 | RIGHT | Right obstacle |
| 3 | TOP | Step-up cue / all-motor fall alert |

DRV2605 ROM effects used: strong buzz (14), gentle click (17), strong click (1),
long buzz (47), library 1 (ERM), internal-trigger mode.

---

## Bill of materials

| Qty | Part | Function |
|---|---|---|
| 1 | Seeed XIAO ESP32-S3 | MCU |
| 1 | VL53L1X ToF module | Forward LiDAR ranging |
| 1 | MPU-6050 module | IMU (accel + gyro) |
| 3 | HC-SR04 | Ultrasonic ranging (front/left/right) |
| 1 | PCA9548A | I²C multiplexer |
| 4 | DRV2605L | Haptic driver |
| 4 | ERM vibration motor | Directional haptic feedback |
| — | Li-Po battery + connectors, wiring, enclosure | Power & mounting |

---

## Tunable thresholds (firmware)

| Constant | Value | Meaning |
|---|---|---|
| `THRESH_LIDAR_CLOSE` | 60 cm | Urgent front buzz |
| `THRESH_LIDAR_MID` | 120 cm | Early front warning |
| `THRESH_FRONT` | 100 cm | Front ultrasonic fallback |
| `THRESH_SIDE` | 80 cm | Left/right alert |

---

## Change log

- **PCB v1** — single XIAO ESP32-S3 on J1/J2 sockets; ultrasonic pins moved to
  1/2/3/4/43/9; added PCA9548A + 4× DRV2605L haptics. (Supersedes the original
  breadboard wiring.)
