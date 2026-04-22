#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <MPU6050.h>
#include "classifier.h"

#define TRIG_FRONT 2
#define ECHO_FRONT 44
#define TRIG_LEFT  8
#define ECHO_LEFT  9
#define TRIG_RIGHT 43
#define ECHO_RIGHT 4

VL53L1X lidar;
MPU6050 mpu;
Eloquent::ML::Port::RandomForest clf;
bool lidar_ok = false;
bool mpu_ok   = false;

// Feature order must match train_model.py
// [lidar_cm, front_cm, left_cm, right_cm, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps]
#define N_RAW    10
#define WINDOW   5
#define N_FEATS  (N_RAW * 4)  // mean, std, min, max per raw feature

float ring[WINDOW][N_RAW];
int   ring_count = 0;
int   ring_head  = 0;

const char* LABELS[] = {
  "left_turn", "obstacle_avoid", "right_turn",
  "step_down", "step_up", "stop", "walking"
};

void waitForPickup();

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
  return (count > 0) ? (d / count) : 400.0; // fallback 400cm
}

void computeFeatures(float* out) {
  // For each of N_RAW features: mean, std, min, max
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

  waitForPickup();
}

void waitForPickup() {
  if (!mpu_ok) return;
  Serial.println("--- Sleeping. Pick up to activate ---");
  ring_count = 0; ring_head = 0; // reset window
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
    float d = lidar.read() / 10.0;
    if (!lidar.timeoutOccurred() && d < 400) lc = d;
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

  // --- Inference (once window is full) ---
  const char* label = "...";
  if (ring_count == WINDOW) {
    float feats[N_FEATS];
    computeFeatures(feats);
    int pred = clf.predict(feats);
    label = LABELS[pred];
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
  Serial.printf("LiDAR:%.1f | F:%.1f L:%.1f R:%.1f | AX:%.3f AY:%.3f AZ:%.3f GX:%.2f GY:%.2f GZ:%.2f | >> %s\n",
    lc, f, l, r, ax, ay, az, gx, gy, gz, label);

  delay(50);
}
