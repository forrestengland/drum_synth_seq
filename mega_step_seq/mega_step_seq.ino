// Arduino Mega 2560 MIDI step sequencer
// 16 step buttons + 16 LEDs (4 banks x 16 = 64 steps), 2 shift buttons, play/stop, tempo + swing pots
// 8 "channels" = 8 notes on MIDI channel 0. Per-channel loop length (polyrhythm).
// MIDI out: Serial1 TX1 (pin 18) @31250 -> DIN-5 (5V-220R-pin4, TX1-220R-pin5, pin2 GND)
//
// Controls (step buttons 1-16 = index 0-15):
//   none      : toggle step in viewed bank (current channel)
//   Shift A   : 1-8 channel, 9-12 bank 1-4, 13-16 spare
//   Shift B   : 1-8 pattern, 9-15 spare, 16 (hold 1 s) = save to EEPROM
//   A + B     : set last step of current channel = bank*16 + button

#include <Arduino.h>
#include <EEPROM.h>

// ---------- Pins ----------
const uint8_t STEP_BTN[16] = {22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37};
const uint8_t STEP_LED[16] = {38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 51, 50, 52, 53};  // 50/51 swapped to match board
const uint8_t SHIFT_A_PIN = 7;
const uint8_t SHIFT_B_PIN = 6;
const uint8_t PLAY_PIN = 5;
const uint8_t TEMPO_POT = A0;
const uint8_t SWING_POT = A1;

// ---------- MIDI config ----------
const uint8_t MIDI_CH = 0;                       // channel 1 on the wire
const uint8_t NOTES[8] = {30, 31, 32, 33, 34, 35, 36, 37};
const uint8_t VELOCITY = 100;

// ---------- Data ----------
uint16_t steps[8][8][4];     // [pattern][channel][bank] -> 16 step bits
uint8_t lastStep[8][8];      // [pattern][channel] -> 0..63 (loop length - 1)

uint8_t curPat = 0, curCh = 0, viewBank = 0;

// ---------- Transport ----------
bool running = false;
uint32_t stepCounter = 0;
uint32_t gridTime = 0, fireTime = 0;             // micros
uint32_t stepLenUs = 125000;
uint8_t swingPct = 0;

bool noteOn[8];
uint32_t offTime[8];

// ---------- Buttons ----------
struct Btn {
  uint8_t pin;
  bool down;
  uint32_t t;
};
Btn stepB[16], shiftA, shiftB, playB;

void initBtn(Btn &b, uint8_t pin) {
  b.pin = pin; b.down = false; b.t = 0;
  pinMode(pin, INPUT_PULLUP);
}

// updates debounced state; returns true on press edge (5 ms debounce)
bool pressed(Btn &b) {
  bool raw = !digitalRead(b.pin);
  uint32_t now = millis();
  if (raw != b.down && now - b.t >= 5) {
    b.down = raw;
    b.t = now;
    return raw;
  }
  return false;
}

// ---------- MIDI ----------
inline void midiOn(uint8_t note)  { Serial1.write(0x90 | MIDI_CH); Serial1.write(note); Serial1.write(VELOCITY); }
inline void midiOff(uint8_t note) { Serial1.write(0x80 | MIDI_CH); Serial1.write(note); Serial1.write((uint8_t)0); }

void allNotesOff() {
  for (uint8_t c = 0; c < 8; c++) {
    if (noteOn[c]) { midiOff(NOTES[c]); noteOn[c] = false; }
  }
}

// ---------- Pots ----------
uint16_t tempoF = 0, swingF = 0, tempoVal = 512, swingVal = 0;

void readPots() {
  tempoF += ((int16_t)analogRead(TEMPO_POT) - (int16_t)tempoF) / 4;
  swingF += ((int16_t)analogRead(SWING_POT) - (int16_t)swingF) / 4;
  if (abs((int16_t)tempoF - (int16_t)tempoVal) > 3) tempoVal = tempoF;   // hysteresis
  if (abs((int16_t)swingF - (int16_t)swingVal) > 3) swingVal = swingF;
  uint16_t bpm = map(tempoVal, 0, 1023, 40, 240);
  stepLenUs = 15000000UL / bpm;                 // 16th notes
  swingPct = map(swingVal, 0, 1023, 0, 66);
}

// ---------- Sequencer ----------
inline uint8_t chLen(uint8_t pat, uint8_t ch) { return lastStep[pat][ch] + 1; }

void computeFireTime() {
  uint32_t off = 0;
  if (stepCounter & 1) off = (stepLenUs * swingPct) / 100;
  fireTime = gridTime + off;
}

void playStep() {
  uint32_t now = micros();
  uint32_t gate = stepLenUs / 2;
  for (uint8_t c = 0; c < 8; c++) {
    uint8_t p = stepCounter % chLen(curPat, c);
    if (steps[curPat][c][p >> 4] & (1u << (p & 15))) {
      if (noteOn[c]) midiOff(NOTES[c]);
      midiOn(NOTES[c]);
      noteOn[c] = true;
      offTime[c] = now + gate;
    }
  }
}

void startSeq() {
  running = true;
  stepCounter = 0;
  gridTime = micros();
  fireTime = gridTime;
}

void stopSeq() {
  running = false;
  allNotesOff();
}

void runClock() {
  uint32_t now = micros();
  for (uint8_t c = 0; c < 8; c++) {
    if (noteOn[c] && (int32_t)(now - offTime[c]) >= 0) {
      midiOff(NOTES[c]);
      noteOn[c] = false;
    }
  }
  if (!running) return;
  if ((int32_t)(now - fireTime) >= 0) {
    playStep();
    stepCounter++;
    gridTime += stepLenUs;
    computeFireTime();
  }
}

// ---------- Save / load (EEPROM) ----------
// Layout: magic(4) | steps(512) | lastStep(64) | checksum(2) = 582 bytes
const uint32_t SAVE_MAGIC = 0x53455132UL;       // change this to invalidate old saves
const int EE_MAGIC = 0;
const int EE_STEPS = EE_MAGIC + sizeof(SAVE_MAGIC);
const int EE_LAST  = EE_STEPS + sizeof(steps);
const int EE_SUM   = EE_LAST + sizeof(lastStep);

uint32_t flashUntil = 0;                        // all LEDs on until this time (save confirmation)

uint16_t dataChecksum() {
  uint16_t sum = 0;
  const uint8_t *p = (const uint8_t *)steps;
  for (size_t i = 0; i < sizeof(steps); i++) sum = ((sum << 1) | (sum >> 15)) ^ p[i];
  p = (const uint8_t *)lastStep;
  for (size_t i = 0; i < sizeof(lastStep); i++) sum = ((sum << 1) | (sum >> 15)) ^ p[i];
  return sum;
}

void initDefaults() {
  memset(steps, 0, sizeof(steps));
  memset(lastStep, 15, sizeof(lastStep));       // default 16-step loops
}

bool loadState() {
  uint32_t magic;
  uint16_t stored;
  EEPROM.get(EE_MAGIC, magic);
  if (magic != SAVE_MAGIC) return false;
  EEPROM.get(EE_STEPS, steps);
  EEPROM.get(EE_LAST, lastStep);
  EEPROM.get(EE_SUM, stored);
  if (stored != dataChecksum()) {               // corrupt or partial save
    initDefaults();
    return false;
  }
  return true;
}

void saveState() {
  allNotesOff();                                // writes block for a moment
  EEPROM.put(EE_STEPS, steps);                  // put() only writes bytes that changed
  EEPROM.put(EE_LAST, lastStep);
  EEPROM.put(EE_SUM, dataChecksum());
  EEPROM.put(EE_MAGIC, SAVE_MAGIC);             // magic last, so a partial save is never valid
  if (running) {                                // resync clock after the blocking write
    gridTime = micros();
    computeFireTime();
  }
  flashUntil = millis() + 400;
}

// Shift B + step 16 held for 1 second = save
void checkSaveCombo() {
  static uint32_t holdStart = 0;
  static bool done = false;
  if (shiftB.down && !shiftA.down && stepB[15].down) {
    if (holdStart == 0) holdStart = millis();
    else if (!done && millis() - holdStart >= 1000) {
      saveState();
      done = true;
    }
  } else {
    holdStart = 0;
    done = false;
  }
}

// ---------- UI ----------
void handleButtons() {
  pressed(shiftA);                              // keep shift state current
  pressed(shiftB);
  bool a = shiftA.down, b = shiftB.down;

  for (uint8_t i = 0; i < 16; i++) {
    if (!pressed(stepB[i])) continue;
    if (a && b) {                               // last step of current channel
      lastStep[curPat][curCh] = viewBank * 16 + i;
    } else if (a) {                             // channel / bank
      if (i < 8) curCh = i;
      else if (i < 12) viewBank = i - 8;
    } else if (b) {                             // pattern
      if (i < 8) curPat = i;
    } else {                                    // edit
      steps[curPat][curCh][viewBank] ^= (1u << i);
    }
  }

  if (pressed(playB)) {
    if (running) stopSeq(); else startSeq();
  }
}

void updateLeds() {
  if ((int32_t)(millis() - flashUntil) < 0) {   // save confirmation: all on
    for (uint8_t i = 0; i < 16; i++) digitalWrite(STEP_LED[i], HIGH);
    return;
  }
  bool a = shiftA.down, b = shiftB.down;
  uint8_t last = lastStep[curPat][curCh];

  // step most recently played on the edited channel
  int16_t playPos = -1;
  if (running && stepCounter > 0) playPos = (stepCounter - 1) % chLen(curPat, curCh);

  for (uint8_t i = 0; i < 16; i++) {
    bool on = false;
    if (a && b) {
      on = ((last >> 4) == viewBank) && ((last & 15) == i);
    } else if (a) {
      if (i < 8) on = (i == curCh);
      else if (i < 12) on = ((i - 8) == viewBank);
    } else if (b) {
      if (i < 8) on = (i == curPat);
    } else {
      on = steps[curPat][curCh][viewBank] & (1u << i);
      if (playPos >= 0 && (playPos >> 4) == viewBank && (playPos & 15) == i) on = !on;
    }
    digitalWrite(STEP_LED[i], on);
  }
}

// ---------- Setup / loop ----------
void setup() {
  Serial1.begin(31250);
  for (uint8_t i = 0; i < 16; i++) {
    initBtn(stepB[i], STEP_BTN[i]);
    pinMode(STEP_LED[i], OUTPUT);
  }
  initBtn(shiftA, SHIFT_A_PIN);
  initBtn(shiftB, SHIFT_B_PIN);
  initBtn(playB, PLAY_PIN);

  initDefaults();
  loadState();                                  // restore last save if present

  tempoF = tempoVal = analogRead(TEMPO_POT);
  swingF = swingVal = analogRead(SWING_POT);
  readPots();

  startSeq();
}

void loop() {
  static uint32_t potT = 0;
  runClock();                                   // timing first
  handleButtons();
  checkSaveCombo();
  if (millis() - potT >= 5) { potT = millis(); readPots(); }
  updateLeds();
}
