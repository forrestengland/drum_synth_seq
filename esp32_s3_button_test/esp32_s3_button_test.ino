#include <Arduino.h>

struct Input {
  const char* name;
  uint8_t pin;
  bool stable;
  bool lastReading;
  uint32_t lastChange;
};

Input inputs[] = {
  { "SYNC switch",    8,  HIGH, HIGH, 0 },
  { "WAVE switch",    4,  HIGH, HIGH, 0 },
  { "ACCENT button",  1,  HIGH, HIGH, 0 },
  { "START button",   2,  HIGH, HIGH, 0 },
  { "PATTERN button", 47, HIGH, HIGH, 0 },
};

constexpr int NUM_INPUTS = sizeof(inputs) / sizeof(inputs[0]);
constexpr uint32_t DEBOUNCE_MS = 20;

void setup() {
  Serial.begin(115200);
  delay(500);

  for (int i = 0; i < NUM_INPUTS; i++) {
    pinMode(inputs[i].pin, INPUT_PULLUP);
    inputs[i].stable = digitalRead(inputs[i].pin);
    inputs[i].lastReading = inputs[i].stable;
  }

  Serial.println("Switch test. Each input should read LOW when closed to ground.");
  for (int i = 0; i < NUM_INPUTS; i++) {
    Serial.printf("  %-15s GPIO %-2d  %s\n",
                  inputs[i].name, inputs[i].pin,
                  inputs[i].stable == LOW ? "LOW (closed)" : "HIGH (open)");
  }
}

void loop() {
  for (int i = 0; i < NUM_INPUTS; i++) {
    Input& in = inputs[i];
    bool reading = digitalRead(in.pin);

    if (reading != in.lastReading) {
      in.lastReading = reading;
      in.lastChange = millis();
    }

    if ((millis() - in.lastChange) >= DEBOUNCE_MS && reading != in.stable) {
      in.stable = reading;
      Serial.printf("%-15s (GPIO %d) -> %s\n",
                    in.name, in.pin,
                    reading == LOW ? "LOW / closed" : "HIGH / open");
    }
  }
}
