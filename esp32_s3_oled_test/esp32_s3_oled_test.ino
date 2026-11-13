#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define I2C_SDA 11
#define I2C_SCL 12
#define OLED_ADDRESS 0x3C

Adafruit_SSD1306 display(128, 64, &Wire, -1, 400000UL, 400000UL);

void scanI2C() {
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  device at 0x%02X\n", addr);
      found++;
    }
  }
  if (!found) Serial.println("  nothing found, check SDA/SCL wiring and pull-ups");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  scanI2C();

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED init FAILED");
    while (true) delay(1000);
  }
  Serial.println("OLED init OK");

  // Static test: border, corner pixels, text
  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 10);
  display.print("OLED TEST");
  display.setCursor(10, 22);
  display.print("128x64 @ 0x3C");
  display.display();
  delay(2000);
}

void loop() {
  static uint32_t frames = 0;
  static uint32_t lastFpsMs = 0;
  static int x = 0, dx = 2;
  static uint32_t counter = 0;

  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);

  // Step-grid style cells, same look as the sequencer
  for (int i = 0; i < 32; i++) {
    int cx = 2 + (i % 8) * 16;
    int cy = 11 + (i / 8) * 11;
    if (i % 3 == 0) display.fillRect(cx, cy, 7, 7, SSD1306_WHITE);
    else display.drawPixel(cx + 3, cy + 3, SSD1306_WHITE);
  }

  // Moving marker shows whether refresh is smooth
  x += dx;
  if (x <= 0 || x >= 120) dx = -dx;
  display.fillRect(x, 56, 8, 5, SSD1306_WHITE);

  display.setCursor(4, 2);
  display.print(counter++);

  display.display();
  frames++;

  if (millis() - lastFpsMs >= 1000) {
    Serial.printf("%lu fps\n", (unsigned long)frames);
    frames = 0;
    lastFpsMs = millis();
  }
}
