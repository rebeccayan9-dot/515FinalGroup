// Standalone haptic bring-up test — fires each DRV2605 motor one at a time.
// Robust version: re-checks each channel every pass via raw I2C ACK (like the
// scanner) and fires regardless of begin()'s return, so an intermittently
// connected motor still buzzes whenever it's electrically present.
// Build/upload only this with:  pio run -e haptic_test --target upload
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_DRV2605.h>

#define PCA_ADDR 0x70
#define DRV_ADDR 0x5A
const uint8_t CH[4]   = {0, 1, 2, 3};
const char*   NAME[4] = {"FRONT", "LEFT", "RIGHT", "TOP"};
#define FX_STRONG_BUZZ 14

Adafruit_DRV2605 drv;

void tcaSelect(uint8_t ch) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

// raw ACK probe of the DRV on the currently-selected mux channel
bool drvPresent() {
  Wire.beginTransmission(DRV_ADDR);
  return Wire.endTransmission() == 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();
  Wire.setClock(100000);

  Wire.beginTransmission(PCA_ADDR);
  bool mux = (Wire.endTransmission() == 0);
  Serial.println("\n=== HAPTIC TEST (robust, ACK-based) ===");
  Serial.printf("PCA9548A mux @0x70: %s\n", mux ? "PRESENT" : "MISSING -> check ribbon/power!");
  Serial.println("Firing FRONT -> LEFT -> RIGHT -> TOP, looping. Feel each one.\n");
}

void loop() {
  for (int i = 0; i < 4; i++) {
    tcaSelect(CH[i]);
    delay(5);
    if (!drvPresent()) {
      Serial.printf(">> ch%u %-6s : not detected this pass\n", CH[i], NAME[i]);
      delay(500);
      continue;
    }
    drv.begin();                 // configure (ignore return — ACK already confirmed)
    drv.selectLibrary(1);
    drv.setMode(DRV2605_MODE_INTTRIG);
    Serial.printf(">> ch%u %-6s : PRESENT -> BUZZING\n", CH[i], NAME[i]);
    for (int p = 0; p < 3; p++) {
      drv.setWaveform(0, FX_STRONG_BUZZ);
      drv.setWaveform(1, 0);
      drv.go();
      delay(350);
    }
    delay(900);                  // gap before next motor
  }
  Serial.println("--- sweep complete, repeating ---\n");
  delay(1000);
}
