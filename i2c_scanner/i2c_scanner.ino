#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(200);
  Wire.begin();             // ESP32 default: SDA=GPIO21, SCL=GPIO22
  delay(800);               // give BNO055 time to boot
  Serial.println();
  Serial.println("Scanning I2C bus...");
}

void loop() {
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      if (a < 16) Serial.print('0');
      Serial.println(a, HEX);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.print("Total devices: ");
    Serial.println(found);
  }
  Serial.println("---");
  delay(2000);
}
