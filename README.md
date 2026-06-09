# SafeStep — ESP32-S3 Assistive Cane

TECHIN 515 Final Group Project

SafeStep is a wearable/cane-mounted obstacle- and terrain-awareness device on a
Seeed XIAO ESP32-S3. It classifies the user's motion in real time with an
on-device Random Forest — fed by **three ultrasonic rangefinders + an IMU** — and
delivers directional haptic alerts through vibration motors. A forward VL53L1X
LiDAR and the ultrasonic sensors also drive proximity warnings.

```
sensors ──► DSP (sliding window, mean/std/min/max) ──► Random Forest ──► haptic alert
3× ultrasonic + IMU              36 features (9ch × 4)     7 classes     L/R motors
```

> The LiDAR is still read and streamed (and drives obstacle haptics), but it was
> dropped from the **classifier** model — see [`train_model.py`](train_model.py).
> The PCB designs for **four** haptic motors; the current build drives **two**
> (left/right) — see [`hardware/HARDWARE.md`](hardware/HARDWARE.md).

---

## Repository layout

| Path | What it is |
|---|---|
| `src/main.cpp` | Firmware: sensor fusion, DSP, on-device inference, haptics |
| `src/classifier.h` | Auto-generated C++ Random Forest (from `train_model.py`) |
| `platformio.ini` | PlatformIO build config + library dependencies |
| `serial_to_csv.py` | **Stream sensor data** → labeled CSV (CLI) |
| `dashboard.py` | **Stream sensor data** → live web dashboard + recording |
| `train_model.py` | Train the Random Forest, export `model.pkl` + `classifier.h` |
| `e2e_eval.py` | End-to-end pipeline + cross-condition evaluation, writes `results/` |
| `data/` | Labeled recordings (one CSV per session) |
| `hardware/` | **Design files** — pinout, BOM, block diagram + Fusion 360 CAD links |
| `results/` | Evaluation report, confusion matrices, accuracy charts |
| `model.pkl` | Trained model bundle (used by dashboard + eval) |

---

## Hardware

| Component | Part | Interface |
|---|---|---|
| MCU | Seeed XIAO ESP32-S3 | — |
| LiDAR | VL53L1X | I²C |
| IMU | MPU-6050 | I²C |
| Ultrasonic ×3 | HC-SR04 (front / left / right) | GPIO TRIG/ECHO |
| I²C mux | PCA9548A | I²C (addr `0x70`) |
| Haptic driver ×4 | DRV2605L + ERM motor (front/left/right/top) | I²C via mux (addr `0x5A`) |

Full pin assignments, I²C address map, and bill of materials are in
[`hardware/HARDWARE.md`](hardware/HARDWARE.md).

---

## Requirements

### Firmware
- [PlatformIO](https://platformio.org/install) (VS Code extension or CLI)
- Board: **Seeed XIAO ESP32-S3** (`board = seeed_xiao_esp32s3`)
- Libraries (auto-installed by PlatformIO — see `platformio.ini`):
  `pololu/VL53L1X`, `electroniccats/MPU6050`, `adafruit/Adafruit DRV2605 Library`

### Python (data collection, training, evaluation)
```bash
pip install pyserial scikit-learn pandas numpy joblib matplotlib flask micromlgen
```
> Use Python 3.13 (the model/eval scripts are validated there).

---

## How to run

### 1. Flash the firmware
```bash
git clone https://github.com/rebeccayan9-dot/515FinalGroup.git
cd 515FinalGroup
pio run --target upload          # or use the PlatformIO "Upload" button in VS Code
```
On boot the device sleeps until picked up (IMU motion), then streams sensor data
over USB serial at **115200 baud** and fires haptics on detected hazards.

### 2. Stream sensor data

Two interchangeable tools read the same serial stream:

**A. CLI → CSV** (for data collection):
```bash
python serial_to_csv.py                       # auto-detect port
python serial_to_csv.py /dev/tty.usbmodem101  # macOS/Linux, explicit port
python serial_to_csv.py COM3                   # Windows
```
Prompts for a behavior label, records for 3 minutes (Ctrl+C to stop early), and
saves to `data/{label}_{timestamp}.csv`.

**B. Live web dashboard** (real-time charts + one-click recording):
```bash
python dashboard.py        # then open http://localhost:8050
```
Pick the serial port in the header, watch live distance/IMU plots and the current
prediction, and record labeled sessions from the footer controls.

### 3. Train the classifier
```bash
python train_model.py
```
Prints accuracy + confusion matrix, saves `model.pkl`, and exports the on-device
C++ classifier to `src/classifier.h`. Re-flash the firmware to deploy the new model.

### 4. Evaluate the end-to-end pipeline
```bash
python e2e_eval.py
```
Runs the full pipeline twice (reproducibility check) and a leave-one-session-out
cross-condition test, writing logs, confusion-matrix PNGs, and a consolidated
report to `results/` (see [`results/E2E_RESULTS.md`](results/E2E_RESULTS.md)).

---

## Data format

Each CSV in `data/` has these columns:

| Column | Unit | Description |
|---|---|---|
| `timestamp_s` | s | Time since recording started |
| `lidar_cm` | cm | VL53L1X distance (`OUT`/400 = out of range) |
| `front_cm` / `left_cm` / `right_cm` | cm | HC-SR04 distances (`-1` = no echo) |
| `ax_g` / `ay_g` / `az_g` | g | Accelerometer |
| `gx_dps` / `gy_dps` / `gz_dps` | °/s | Gyroscope |
| `label` | — | Behavior label for the session |

**Classes:** `walking` · `left_turn` · `right_turn` · `stop` · `step_up` ·
`step_down` · `obstacle_avoid`

---

## Milestone status

- **M1 — Sensor fusion & data collection** ✓ — concurrent LiDAR + 3× ultrasonic +
  IMU at ~5 Hz; labeled-data tooling (`serial_to_csv.py`, `dashboard.py`).
- **M2 — Behavior recognition (TinyML)** ✓ — sliding-window Random Forest trained
  (`train_model.py`), exported to C++ (`classifier.h`), running on-device with
  haptic mapping. End-to-end + cross-condition results in `results/`.

See [`PRESENTATION_READINESS.md`](PRESENTATION_READINESS.md) for a readiness summary.
