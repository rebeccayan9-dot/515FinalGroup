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

### Wiring

| Signal | ESP32-S3 GPIO |
|---|---|
| TRIG_FRONT | 2 |
| ECHO_FRONT | 44 |
| TRIG_LEFT | 8 |
| ECHO_LEFT | 9 |
| TRIG_RIGHT | 43 |
| ECHO_RIGHT | 4 |
| VL53L1X SDA | 5 (default I2C) |
| VL53L1X SCL | 6 (default I2C) |

---

## Requirements

### Firmware
- [PlatformIO](https://platformio.org/install) (VS Code extension or CLI)
- Board: **Seeed XIAO ESP32-S3**
- Library dependency (auto-installed by PlatformIO): `pololu/VL53L1X @ ^1.3.1`

### Python (data collection & training)
```bash
pip install pyserial scikit-learn pandas joblib
```

---

## How to Run

### 1. Flash the firmware

```bash
# Clone the repo
git clone https://github.com/rebeccayan9-dot/515FinalGroup.git
cd 515FinalGroup

# Build and upload with PlatformIO CLI
pio run --target upload

# Or open the folder in VS Code with PlatformIO extension and click Upload
```

The device will begin streaming sensor data over USB serial at **115200 baud** immediately after flashing.

### 2. Stream and record sensor data

Connect the ESP32-S3 via USB, then run:

```bash
python serial_to_csv.py
```

The script will:
1. Auto-detect the serial port
2. Prompt you to select a behavior label (walking, left_turn, stop, etc.)
3. Stream and display live readings
4. Save data to `data/{label}_{timestamp}.csv`

Press **Ctrl+C** to stop recording.

To specify a port manually:
```bash
python serial_to_csv.py /dev/tty.usbmodem101   # macOS/Linux
python serial_to_csv.py COM3                    # Windows
```

**Example output:**
```
Select behavior label:
  1. walking
  2. left_turn
  3. right_turn
  4. stop
  5. step_up
  6. step_down
  7. obstacle_avoid
  0. Custom (type your own)
Enter number or custom name: 1

Connecting to /dev/tty.usbmodem101 at 115200 baud...
Label: walking
Saving to data/walking_20260416_233414.csv

LiDAR:31.0 | F:38.1 L:16.3 R:92.2
LiDAR:33.2 | F:44.6 L:16.3 R:92.2
```

### 3. Train the behavior classifier

After collecting labeled data for at least 2 behavior classes:

```bash
python train_model.py
```

Outputs accuracy report, confusion matrix, and saves `model.pkl`.

---

## Data Format

Each CSV in `data/` has the following columns:

| Column | Unit | Description |
|---|---|---|
| `timestamp_s` | seconds | Time since recording started |
| `lidar_cm` | cm | VL53L1X LiDAR distance (400.0 = out of range) |
| `front_cm` | cm | Front HC-SR04 ultrasonic distance |
| `left_cm` | cm | Left HC-SR04 ultrasonic distance |
| `right_cm` | cm | Right HC-SR04 ultrasonic distance |
| `label` | — | Behavior label for this session |

---

## Milestone 1 — Sensor Fusion & Data Collection ✓

- 3× HC-SR04 ultrasonic (front / left / right) + VL53L1X LiDAR running concurrently on ESP32-S3
- Filtered distance readings (3-sample average per sensor) output over Serial at ~5 Hz
- `serial_to_csv.py` — labeled data collection with behavior menu
- `train_model.py` — sliding-window Random Forest classifier skeleton

---

## Milestone 2 — Behavior Recognition & Step Detection (TinyML)

### Phase 1 · Behavior Classification

**Goal:** classify the user's current motion state in real time.

**Classes:** walking · left_turn · right_turn · stop · step_up · step_down · obstacle_avoid

**Features (sliding window, 10 rows ≈ 2 s):** mean, std, min, max per sensor → 16 features total

**Model:** Random Forest (100 trees) — `train_model.py`

### Phase 2 · Step & Edge Detection

**Goal:** binary classifier — distinguish "wall ahead" from "downward stair edge."

**Key feature:**
```
delta = lidar_cm - front_cm
```
At a stair edge, LiDAR distance jumps sharply while ultrasonic still reads the edge → `delta` spikes.

### Phase 3 · On-Device Deployment

1. Upload labeled CSVs to [Edge Impulse](https://edgeimpulse.com)
2. Generate ESP32-S3-optimized C++ inference library (supports ESP-NN)
3. Drop library into `lib/`, call `run_inference()` every 100 ms in `loop()`
4. Map predicted class → haptic alert level

---

## TODOs

- [ ] Collect labeled data for all 7 behavior classes (≥ 3 min each)
- [ ] Tune window size and Random Forest hyperparameters
- [ ] Implement `delta` feature and stair-edge binary classifier
- [ ] Upload to Edge Impulse and generate ESP32-S3 C++ library
- [ ] Integrate inference into `main.cpp` loop
- [ ] BLE notifications on behavior change
- [ ] I2S audio cues ("obstacle ahead", "step down")
- [ ] IMU cane-down detection — suppress alerts when cane is lifted
