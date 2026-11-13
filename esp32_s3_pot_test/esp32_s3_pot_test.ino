#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#define I2C_SDA 11
#define I2C_SCL 12

Adafruit_ADS1115 ads;

int16_t minRaw[4] = {32767, 32767, 32767, 32767};
int16_t maxRaw[4] = {-32768, -32768, -32768, -32768};

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!ads.begin(0x48)) {
    Serial.println("ADS1115 init FAILED (check wiring, ADDR pin to GND)");
    while (true) delay(1000);
  }

  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS);
  Serial.println("ADS1115 OK. Sweep each pot fully, min/max are tracked.");
}

void loop() {
  static uint32_t lastPrint = 0;
  int16_t raw[4];

  for (int ch = 0; ch < 4; ch++) {
    raw[ch] = ads.readADC_SingleEnded(ch);
    if (raw[ch] < minRaw[ch]) minRaw[ch] = raw[ch];
    if (raw[ch] > maxRaw[ch]) maxRaw[ch] = raw[ch];
  }

  if (millis() - lastPrint >= 200) {
    lastPrint = millis();

    for (int ch = 0; ch < 4; ch++) {
      float norm = constrain(raw[ch] / 26400.0f, 0.0f, 1.0f);
      Serial.printf("P%d raw %5d (%.3f) [%5d..%5d]   ",
                    ch, raw[ch], norm, minRaw[ch], maxRaw[ch]);
    }
    Serial.println();
  }
}
