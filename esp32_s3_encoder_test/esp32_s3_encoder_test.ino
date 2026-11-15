
#include <Arduino.h>

#define ENCODER_CLK 13
#define ENCODER_DT  14
#define ENCODER_SW  15

#define ENCODER2_CLK 17
#define ENCODER2_DT  18
#define ENCODER2_SW  21

#define ENCODER_TRANSITIONS_PER_DETENT 4   // try 2 if one click counts twice

struct Encoder {
  uint8_t clkPin;
  uint8_t dtPin;
  volatile uint8_t state;
  volatile int8_t accumulator;
  volatile int32_t detents;
  volatile uint32_t badTransitions;   // invalid jumps, a sign of bounce
};

Encoder enc1 = { ENCODER_CLK,  ENCODER_DT,  0, 0, 0, 0 };
Encoder enc2 = { ENCODER2_CLK, ENCODER2_DT, 0, 0, 0, 0 };

static const DRAM_ATTR int8_t encoderTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void IRAM_ATTR encoderISR(void* arg) {
  Encoder* e = (Encoder*)arg;

  uint8_t current =
      (gpio_get_level((gpio_num_t)e->clkPin) << 1) |
       gpio_get_level((gpio_num_t)e->dtPin);

  uint8_t index = (e->state << 2) | current;
  int8_t movement = encoderTable[index];

  if (movement == 0 && current != e->state) {
    e->badTransitions++;
  }
  e->state = current;

  e->accumulator += movement;

  if (e->accumulator >= ENCODER_TRANSITIONS_PER_DETENT) {
    e->accumulator = 0;
    e->detents++;
  } else if (e->accumulator <= -ENCODER_TRANSITIONS_PER_DETENT) {
    e->accumulator = 0;
    e->detents--;
  }
}

void setupEncoder(Encoder& e) {
  pinMode(e.clkPin, INPUT_PULLUP);
  pinMode(e.dtPin, INPUT_PULLUP);
  e.state = (digitalRead(e.clkPin) << 1) | digitalRead(e.dtPin);
  attachInterruptArg(digitalPinToInterrupt(e.clkPin), encoderISR, &e, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(e.dtPin),  encoderISR, &e, CHANGE);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  setupEncoder(enc1);
  setupEncoder(enc2);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  pinMode(ENCODER2_SW, INPUT_PULLUP);

  Serial.println("Encoder test. Turn clockwise: count goes UP. One click = one count.");
}

void loop() {
  static int32_t pos1 = 0, pos2 = 0;
  static bool lastSw1 = HIGH, lastSw2 = HIGH;

  // Take and clear the counts
  noInterrupts();
  int32_t d1 = enc1.detents; enc1.detents = 0;
  int32_t d2 = enc2.detents; enc2.detents = 0;
  uint32_t bad1 = enc1.badTransitions;
  uint32_t bad2 = enc2.badTransitions;
  interrupts();

  if (d1) {
    pos1 += d1;
    Serial.printf("ENC1 %+ld  pos=%ld  (bad transitions: %lu)\n",
                  (long)d1, (long)pos1, (unsigned long)bad1);
  }
  if (d2) {
    pos2 += d2;
    Serial.printf("ENC2 %+ld  pos=%ld  (bad transitions: %lu)\n",
                  (long)d2, (long)pos2, (unsigned long)bad2);
  }

  bool sw1 = digitalRead(ENCODER_SW);
  bool sw2 = digitalRead(ENCODER2_SW);
  if (sw1 != lastSw1) {
    lastSw1 = sw1;
    Serial.printf("ENC1 button %s\n", sw1 == LOW ? "PRESSED" : "released");
  }
  if (sw2 != lastSw2) {
    lastSw2 = sw2;
    Serial.printf("ENC2 button %s\n", sw2 == LOW ? "PRESSED" : "released");
  }

  delay(2);
}
