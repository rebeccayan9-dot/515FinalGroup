// Sensor connection self-check — continuously streams LiDAR/ultrasonic/IMU
// with an explicit OK/DROP flag and a rolling drop-rate, so you can walk &
// shake the device and see whether the LiDAR stays connected under motion.
// Upload:  pio run -e sensor_check --target upload
#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <MPU6050.h>

// PCB v1 ultrasonic pins
#define TRIG_FRONT 1
#define ECHO_FRONT 2
#define TRIG_LEFT  3
#define ECHO_LEFT  4
#define TRIG_RIGHT 43
#define ECHO_RIGHT 9

VL53L1X lidar;
MPU6050 mpu;
bool lidar_ok = false, mpu_ok = false;

#define WIN 50
bool   hist[WIN] = {false};   // last 50 drop flags
int    hpos = 0;
unsigned long total = 0, drops = 0;

float readUS(int t, int e) {
  digitalWrite(t, LOW); delayMicroseconds(2);
  digitalWrite(t, HIGH); delayMicroseconds(10);
  digitalWrite(t, LOW);
  long dur = pulseIn(e, HIGH, 25000);
  return dur > 0 ? (dur * 0.0343 / 2) : 400.0;
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
  }
  mpu.initialize();
  mpu_ok = mpu.testConnection();

  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  Serial.println("\n=== SENSOR CHECK ===");
  Serial.printf("LiDAR init: %s   MPU init: %s\n",
                lidar_ok ? "OK" : "FAILED", mpu_ok ? "OK" : "FAILED");
  Serial.println("Walk & shake the device. Watch LiDAR OK/DROP and drop-rate.\n");
}

void loop() {
  // --- LiDAR ---
  uint16_t raw = 0; bool to = true; uint8_t st = 0;
  if (lidar_ok) {
    raw = lidar.read();
    to  = lidar.timeoutOccurred();
    st  = lidar.ranging_data.range_status;
  }
  bool drop = !lidar_ok || to;
  total++; if (drop) drops++;
  // rolling last-50 drop count
  hist[hpos] = drop;
  hpos = (hpos + 1) % WIN;
  int last50 = 0;
  for (int i = 0; i < WIN; i++) if (hist[i]) last50++;

  // --- ultrasonic ---
  float f = readUS(TRIG_FRONT, ECHO_FRONT);
  float l = readUS(TRIG_LEFT,  ECHO_LEFT);
  float r = readUS(TRIG_RIGHT, ECHO_RIGHT);

  // --- IMU magnitude (just to show motion) ---
  float gmag = 0;
  if (mpu_ok) {
    int16_t ax,ay,az,gx,gy,gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    float GX=gx/131.0, GY=gy/131.0, GZ=gz/131.0;
    gmag = sqrt(GX*GX+GY*GY+GZ*GZ);
  }

  // front sanity flag (your FRONT ultrasonic was reading ~1-2cm)
  const char* fflag = (f < 5.0) ? " <FRONT?" : "";

  Serial.printf("LiDAR %-4s raw=%4u to=%d st=%u | drop %lu/%lu  last50=%d/50 (%d%%) | F:%.0f L:%.0f R:%.0f%s | |gyro|=%.0f\n",
    drop ? "DROP" : "OK", raw, to ? 1 : 0, st,
    drops, total, last50, last50 * 2,
    f, l, r, fflag, gmag);

  delay(80);
}
