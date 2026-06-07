// I2C bus scanner — lists every device on the main bus, then probes each
// PCA9548A mux channel for the 4 DRV2605 haptics.
// Upload:  pio run -e i2c_scan --target upload
#include <Arduino.h>
#include <Wire.h>

#define PCA_ADDR 0x70

void scanBus(const char* tag) {
  int found = 0;
  Serial.printf("[%s] scanning 0x03..0x77: ", tag);
  for (uint8_t a = 0x03; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("0x%02X ", a);
      found++;
    }
  }
  Serial.printf(" -> %d device(s)\n", found);
}

void tcaSelect(uint8_t ch) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();
  Wire.setClock(100000);

  Serial.println("\n=== I2C SCAN ===");
  Serial.println("expected: 0x29 LiDAR, 0x68 IMU, 0x70 mux, 0x5A DRV (behind mux)\n");
  scanBus("main bus");

  // Is the mux present?
  Wire.beginTransmission(PCA_ADDR);
  bool mux = (Wire.endTransmission() == 0);
  Serial.printf("\nPCA9548A mux @0x70: %s\n", mux ? "PRESENT" : "MISSING");

  if (mux) {
    Serial.println("probing each mux channel for DRV2605 @0x5A:");
    for (uint8_t ch = 0; ch < 4; ch++) {
      tcaSelect(ch);
      Wire.beginTransmission(0x5A);
      bool drv = (Wire.endTransmission() == 0);
      Serial.printf("  ch%u: DRV2605 %s\n", ch, drv ? "PRESENT" : "missing");
    }
    tcaSelect(0xFF & 0); // deselect-ish
  } else {
    Serial.println("=> mux not on the bus: check haptic board ribbon/power.");
  }
  Serial.println("=== scan done ===");
}

void loop() { delay(1000); }
