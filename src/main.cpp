#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

// 重新核对这些 D-Number 是否和你面包板接线一致
#define TRIG_FRONT 2
#define ECHO_FRONT 44
#define TRIG_LEFT  8
#define ECHO_LEFT  9
#define TRIG_RIGHT 43
#define ECHO_RIGHT 4

VL53L1X lidar;

// 简单的滤波逻辑示例                                                            
float smoothF = 0;                                                               
float alpha = 0.3; // 滤波系数，越小越平滑，但延迟越大                           
                                
float getFilteredDistance(int t, int e) {
  float d = 0;
  int count = 0;
  for(int i=0; i<3; i++) {
    digitalWrite(t, LOW); delayMicroseconds(2);
    digitalWrite(t, HIGH); delayMicroseconds(10);
    digitalWrite(t, LOW);
    long dur = pulseIn(e, HIGH, 25000);
    if(dur > 0) { d += (dur * 0.0343 / 2); count++; }
    delay(10);
  }
  return (count > 0) ? (d / count) : -1.0;
}

void setup() {
  Serial.begin(115200);
  while(!Serial);

  // 1. 初始化 I2C 和 LiDAR
  Wire.begin(); // 默认使用 GPIO 5(SDA) 和 6(SCL)
  lidar.setTimeout(500);
  if (!lidar.init()) {
    Serial.println("Failed to detect VL53L1X! Check SDA(D4) and SCL(D5)");
  } else {
    lidar.setDistanceMode(VL53L1X::Long);
    lidar.setMeasurementTimingBudget(50000);
    lidar.startContinuous(50);
  }

  // 2. 初始化超声波
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  Serial.println("\n--- SafeStep: 3x Ultrasonic + 1x LiDAR Ready ---");
}

void loop() {
  // 读取激光雷达
  // 稍微修改下 loop 里的打印逻辑
  float l_dist = lidar.read() / 10.0;
  bool lidarValid = !lidar.timeoutOccurred() && (l_dist < 400); // 400cm 是其有效量程

  if (lidarValid) {
    Serial.printf("LiDAR:%.1f | ", l_dist);
  } else {
    Serial.print("LiDAR:OUT | ");
  }
  
  // 读取超声波
  float f = getFilteredDistance(TRIG_FRONT, ECHO_FRONT);
  delay(20);
  float l = getFilteredDistance(TRIG_LEFT,  ECHO_LEFT);
  delay(20);
  float r = getFilteredDistance(TRIG_RIGHT, ECHO_RIGHT);

  // 综合打印
  Serial.printf("LiDAR:%.1f | F:%.1f L:%.1f R:%.1f\n", l_dist, f, l, r);
  
  delay(50);
}