#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <MPU6050.h>

#define TRIG_FRONT 2
#define ECHO_FRONT 44
#define TRIG_LEFT  8
#define ECHO_LEFT  9
#define TRIG_RIGHT 43
#define ECHO_RIGHT 4

VL53L1X lidar;
MPU6050 mpu;
bool lidar_ok = false;
bool mpu_ok   = false;

float getFilteredDistance(int t, int e) {
  float d = 0;
  int count = 0;
  for (int i = 0; i < 3; i++) {
    digitalWrite(t, LOW); delayMicroseconds(2);
    digitalWrite(t, HIGH); delayMicroseconds(10);
    digitalWrite(t, LOW);
    long dur = pulseIn(e, HIGH, 25000);
    if (dur > 0) { d += (dur * 0.0343 / 2); count++; }
    delay(10);
  }
  return (count > 0) ? (d / count) : -1.0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Wire.setClock(100000);

  // LiDAR
  lidar.setTimeout(500);
  lidar_ok = lidar.init();
  if (!lidar_ok) {
    Serial.println("VL53L1X: FAILED");
  } else {
    lidar.setDistanceMode(VL53L1X::Long);
    lidar.setMeasurementTimingBudget(50000);
    lidar.startContinuous(50);
    Serial.println("VL53L1X: OK");
  }

  // IMU
  mpu.initialize();
  mpu_ok = mpu.testConnection();
  Serial.println(mpu_ok ? "MPU6050: OK" : "MPU6050: FAILED");

  // Ultrasonic
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  Serial.println("--- SafeStep Ready ---\n");
}

void loop() {
  // LiDAR
  float l_dist = -1;
  if (lidar_ok) {
    float d = lidar.read() / 10.0;
    if (!lidar.timeoutOccurred() && d < 400) l_dist = d;
  }

  // IMU
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  if (mpu_ok) {
    int16_t ax16, ay16, az16, gx16, gy16, gz16;
    mpu.getMotion6(&ax16, &ay16, &az16, &gx16, &gy16, &gz16);
    ax = ax16 / 16384.0; ay = ay16 / 16384.0; az = az16 / 16384.0;
    gx = gx16 / 131.0;   gy = gy16 / 131.0;   gz = gz16 / 131.0;
  }

  // Ultrasonics
  float f = getFilteredDistance(TRIG_FRONT, ECHO_FRONT);
  delay(20);
  float l = getFilteredDistance(TRIG_LEFT,  ECHO_LEFT);
  delay(20);
  float r = getFilteredDistance(TRIG_RIGHT, ECHO_RIGHT);

  Serial.printf("LiDAR:%s | F:%.1f L:%.1f R:%.1f | AX:%.3f AY:%.3f AZ:%.3f GX:%.2f GY:%.2f GZ:%.2f\n",
    (l_dist < 0) ? "OUT" : String(l_dist, 1).c_str(),
    f, l, r, ax, ay, az, gx, gy, gz);

  delay(50);
}
