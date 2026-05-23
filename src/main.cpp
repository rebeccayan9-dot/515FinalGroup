#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <MPU6050.h>
#include <Adafruit_DRV2605.h>
#include "classifier.h"

// PCB v1 pin assignments (single XIAO ESP32-S3, J1/J2 sockets)
#define TRIG_FRONT  1
#define ECHO_FRONT  2
#define TRIG_LEFT   3
#define ECHO_LEFT   4
#define TRIG_RIGHT  43
#define ECHO_RIGHT  9

// PCA9548A I²C mux + 4× DRV2605L haptic drivers (all share addr 0x5A)
#define PCA_ADDR  0x70
#define CH_FRONT  0
#define CH_LEFT   1
#define CH_RIGHT  2
#define CH_TOP    3

// DRV2605 ROM effect IDs (library 1 = ERM)
#define FX_STRONG_BUZZ   14
#define FX_GENTLE_CLICK  17
#define FX_STRONG_CLICK   1
#define FX_LONG_BUZZ     47

// Obstacle distance thresholds (cm) — tune these during field testing
#define THRESH_LIDAR_CLOSE  60   // urgent strong buzz
#define THRESH_LIDAR_MID   120   // early gentle warning
#define THRESH_FRONT       100   // front ultrasonic (fallback when LiDAR failed)
#define THRESH_SIDE         80   // left / right ultrasonic

VL53L1X lidar;
MPU6050 mpu;
Adafruit_DRV2605 drv;
Eloquent::ML::Port::RandomForest clf;
bool lidar_ok = false;
bool mpu_ok   = false;
bool haptic_ok[4] = {false, false, false, false};

// Must match the label order produced by train_model.py (alphabetical)
// 0:left_turn  1:obstacle_avoid  2:right_turn
// 3:step_down  4:step_up         5:stop        6:walking
#define N_RAW   10
#define WINDOW  5
#define N_FEATS (N_RAW * 4)

const char* LABELS[] = {
  "left_turn", "obstacle_avoid", "right_turn",
  "step_down", "step_up", "stop", "walking"
};

float ring[WINDOW][N_RAW];
int   ring_count = 0;
int   ring_head  = 0;

void waitForPickup();

void tcaSelect(uint8_t ch) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void hapticPlay(uint8_t ch, uint8_t effect) {
  if (!haptic_ok[ch]) return;
  tcaSelect(ch);
  drv.setWaveform(0, effect);
  drv.setWaveform(1, 0);
  drv.go();
}

void hapticInit() {
  for (uint8_t ch = CH_FRONT; ch <= CH_TOP; ch++) {
    tcaSelect(ch);
    if (drv.begin()) {
      drv.selectLibrary(1);
      drv.setMode(DRV2605_MODE_INTTRIG);
      haptic_ok[ch] = true;
      Serial.printf("DRV2605 ch%u: OK\n", ch);
    } else {
      Serial.printf("DRV2605 ch%u: FAILED\n", ch);
    }
  }
}

// 4 independent motors on a handle (up/down/left/right). Fire in parallel —
// directional channels can buzz simultaneously when multiple obstacles exist.
void hapticAlert(int pred, float lc, float f, float l, float r) {
  // step_down: fall hazard — all 4 motors at once
  if (pred == 3) {
    hapticPlay(CH_FRONT, FX_STRONG_BUZZ);
    hapticPlay(CH_LEFT,  FX_STRONG_BUZZ);
    hapticPlay(CH_RIGHT, FX_STRONG_BUZZ);
    hapticPlay(CH_TOP,   FX_STRONG_BUZZ);
    return;
  }
  // Front: LiDAR close=urgent, LiDAR mid=early warning, ultrasonic fallback
  if (lidar_ok) {
    if      (lc < THRESH_LIDAR_CLOSE) hapticPlay(CH_FRONT, FX_STRONG_BUZZ);
    else if (lc < THRESH_LIDAR_MID)   hapticPlay(CH_FRONT, FX_GENTLE_CLICK);
  } else if (f < THRESH_FRONT) {
    hapticPlay(CH_FRONT, FX_STRONG_BUZZ);
  }
  if (l < THRESH_SIDE) hapticPlay(CH_LEFT,  FX_STRONG_CLICK);
  if (r < THRESH_SIDE) hapticPlay(CH_RIGHT, FX_STRONG_CLICK);
  // step_up: gentle upward cue on TOP
  if (pred == 4) hapticPlay(CH_TOP, FX_LONG_BUZZ);
}

float getFilteredDistance(int t, int e) {
  float d = 0; int count = 0;
  for (int i = 0; i < 3; i++) {
    digitalWrite(t, LOW); delayMicroseconds(2);
    digitalWrite(t, HIGH); delayMicroseconds(10);
    digitalWrite(t, LOW);
    long dur = pulseIn(e, HIGH, 25000);
    if (dur > 0) { d += (dur * 0.0343 / 2); count++; }
    delay(10);
  }
  return (count > 0) ? (d / count) : 400.0;
}

void computeFeatures(float* out) {
  for (int f = 0; f < N_RAW; f++) {
    float sum = 0, mn = ring[0][f], mx = ring[0][f];
    for (int i = 0; i < WINDOW; i++) {
      float v = ring[i][f];
      sum += v;
      if (v < mn) mn = v;
      if (v > mx) mx = v;
    }
    float mean = sum / WINDOW;
    float var = 0;
    for (int i = 0; i < WINDOW; i++) {
      float d = ring[i][f] - mean;
      var += d * d;
    }
    out[f * 4 + 0] = mean;
    out[f * 4 + 1] = sqrt(var / WINDOW);
    out[f * 4 + 2] = mn;
    out[f * 4 + 3] = mx;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Wire.setClock(100000);

  lidar.setTimeout(500);
  lidar_ok = lidar.init();
  if (lidar_ok) {
    lidar.setDistanceMode(VL53L1X::Long);
    lidar.setMeasurementTimingBudget(50000);
    lidar.startContinuous(50);
    Serial.println("VL53L1X: OK");
  } else {
    Serial.println("VL53L1X: FAILED");
  }

  mpu.initialize();
  mpu_ok = mpu.testConnection();
  Serial.println(mpu_ok ? "MPU6050: OK" : "MPU6050: FAILED");

  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  hapticInit();

  waitForPickup();
}

void waitForPickup() {
  if (!mpu_ok) return;
  Serial.println("--- Sleeping. Pick up to activate ---");
  ring_count = 0; ring_head = 0;
  int motion_count = 0;
  while (motion_count < 5) {
    int16_t ax16, ay16, az16, gx16, gy16, gz16;
    mpu.getMotion6(&ax16, &ay16, &az16, &gx16, &gy16, &gz16);
    float gx = gx16 / 131.0, gy = gy16 / 131.0, gz = gz16 / 131.0;
    if (sqrt(gx*gx + gy*gy + gz*gz) > 30.0) motion_count++;
    else motion_count = 0;
    delay(20);
  }
  Serial.println("--- Active ---\n");
}

void loop() {
  // --- Read sensors ---
  float lc = 400.0;
  if (lidar_ok) {
    uint16_t raw_mm = lidar.read();
    bool to = lidar.timeoutOccurred();
    float d = raw_mm / 10.0;
    if (!to && d < 400) lc = d;
    Serial.printf("[LiDAR dbg] raw=%u mm  timeout=%d  rangeStatus=%u\n",
                  raw_mm, to ? 1 : 0, lidar.ranging_data.range_status);
  } else {
    Serial.println("[LiDAR dbg] lidar_ok=false (init failed)");
  }

  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  if (mpu_ok) {
    int16_t ax16, ay16, az16, gx16, gy16, gz16;
    mpu.getMotion6(&ax16, &ay16, &az16, &gx16, &gy16, &gz16);
    ax = ax16 / 16384.0; ay = ay16 / 16384.0; az = az16 / 16384.0;
    gx = gx16 / 131.0;   gy = gy16 / 131.0;   gz = gz16 / 131.0;
  }

  float f = getFilteredDistance(TRIG_FRONT, ECHO_FRONT); delay(20);
  float l = getFilteredDistance(TRIG_LEFT,  ECHO_LEFT);  delay(20);
  float r = getFilteredDistance(TRIG_RIGHT, ECHO_RIGHT);

  // --- Update ring buffer ---
  ring[ring_head][0] = lc;
  ring[ring_head][1] = f;
  ring[ring_head][2] = l;
  ring[ring_head][3] = r;
  ring[ring_head][4] = ax;
  ring[ring_head][5] = ay;
  ring[ring_head][6] = az;
  ring[ring_head][7] = gx;
  ring[ring_head][8] = gy;
  ring[ring_head][9] = gz;
  ring_head = (ring_head + 1) % WINDOW;
  if (ring_count < WINDOW) ring_count++;

  // --- Inference + haptic ---
  int pred = -1;
  if (ring_count == WINDOW) {
    float feats[N_FEATS];
    computeFeatures(feats);
    pred = clf.predict(feats);
    hapticAlert(pred, lc, f, l, r);
  }

  // --- Auto-sleep: 30s of stillness ---
  if (mpu_ok) {
    static unsigned long still_since = 0;
    float gyro_mag = sqrt(gx*gx + gy*gy + gz*gz);
    if (gyro_mag > 5.0) {
      still_since = millis();
    } else if (millis() - still_since > 30000UL) {
      Serial.println("--- Still for 30s. Going to sleep ---");
      waitForPickup();
      still_since = millis();
    }
  }

  // --- Output ---
  const char* ml_label = (pred >= 0) ? LABELS[pred] : "...";
  const char* alert = "clear";
  if      (pred == 3)                            alert = "STEP_DOWN";
  else if (lidar_ok ? (lc < THRESH_LIDAR_CLOSE) : (f < THRESH_FRONT)) alert = "OBS_FRONT";
  else if (pred == 4)                            alert = "STEP_UP";
  else if (l < THRESH_SIDE)                      alert = "OBS_LEFT";
  else if (r < THRESH_SIDE)                      alert = "OBS_RIGHT";
  Serial.printf("LiDAR:%.1f | F:%.1f L:%.1f R:%.1f | AX:%.3f AY:%.3f AZ:%.3f GX:%.2f GY:%.2f GZ:%.2f | >> %s\n",
    lc, f, l, r, ax, ay, az, gx, gy, gz, alert);

  delay(50);
}
