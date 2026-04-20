# SafeStep — ESP32-S3 Firmware

TECHIN 515 Final Group Project

---

## Hardware

| Component | Part | Interface |
|---|---|---|
| MCU | Seeed XIAO ESP32-S3 | — |
| Ultrasonic (×3) | HC-SR04 (front, left, right) | GPIO (TRIG/ECHO) |
| LiDAR | VL53L1X | I2C |
| IMU | MPU-6050 | I2C |
| Haptic motor | ERM/LRA | GPIO PWM |
| Mic | INMP441 | I2S |

---

## Milestone 1 — Sensor Fusion & Data Collection ✓

### What we built

- **3× HC-SR04 ultrasonic** (front / left / right) + **VL53L1X LiDAR** running concurrently on ESP32-S3
- Filtered distance readings (3-sample average per sensor per loop) output over Serial at ~5 Hz
- `serial_to_csv.py` — logs serial output to labeled CSV files for ML training

### Data format

Each session is saved to `data/{label}_{timestamp}.csv`:

```
timestamp_s, lidar_cm, front_cm, left_cm, right_cm, label
0.131, 31.0, 38.1, 16.3, 92.2, walking
0.355, 33.2, 44.6, 16.3, 92.2, walking
```

### Behavior labels collected

| Label | Description |
|---|---|
| `walking` | Straight-line walking, no obstacles |
| `left_turn` | Turning left to avoid obstacle |
| `right_turn` | Turning right to avoid obstacle |
| `stop` | Standing still |
| `step_up` | Stepping up a stair |
| `step_down` | Stepping down a stair / edge |
| `obstacle_avoid` | Generic obstacle avoidance maneuver |

### Project structure

```
src/
  main.cpp          — setup(), loop(), sensor fusion + serial output
serial_to_csv.py    — labeled data collection script
train_model.py      — Random Forest training + evaluation
data/               — labeled CSVs
```

---

## Milestone 2 — Behavior Recognition & Step Detection (TinyML)

### Phase 1 · Behavior Classification

**Goal:** classify the user's current motion state in real time.

**Classes:** walking · left_turn · right_turn · stop · step_up · step_down · obstacle_avoid

**Data collection:**
- Run `serial_to_csv.py`, select a label, perform the behavior for 1–3 min, repeat for each class
- Target: ≥ 3 min per class (≈ 900 samples at 5 Hz)

**Features (sliding window, 10 rows ≈ 2 s):**
- mean, std, min, max of each sensor → 16 features total

**Model:** Random Forest (100 trees) — trained with `train_model.py`, saved as `model.pkl`

---

### Phase 2 · Step & Edge Detection

**Goal:** binary classifier — distinguish "wall ahead" from "downward stair edge."

**Key feature:**
```
delta = lidar_cm - front_cm
```
- On flat ground: `delta` stays within a small range
- At a stair edge: LiDAR distance jumps sharply while ultrasonic still reads the edge → `delta` spikes

**Training:** collect "normal walking" vs "stair edge" sessions and train a binary classifier on `delta` + raw sensor values.

---

### Phase 3 · On-Device Deployment (TinyML)

1. **Data prep** — replace `OUT` / `-1.0` readings with `400.0` (max range)
2. **Edge Impulse** — upload CSVs, let it generate ESP32-S3-optimized C++ inference code (supports ESP-NN vector instructions)
3. **PlatformIO integration** — drop generated library into `lib/`, call `run_inference()` every 100 ms in `loop()`
4. **Haptic feedback** — map predicted class → alert level → PWM motor intensity

---

## TODOs

- [ ] Collect labeled data for all 7 behavior classes
- [ ] Tune window size and Random Forest hyperparameters
- [ ] Implement `delta` feature and stair-edge binary classifier
- [ ] Upload to Edge Impulse and generate ESP32-S3 C++ library
- [ ] Integrate inference into `main.cpp` loop
- [ ] BLE notifications on behavior change
- [ ] I2S audio cues ("obstacle ahead", "step down")
- [ ] IMU cane-down detection — suppress alerts when cane is lifted
