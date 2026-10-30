// Arduino Mega 2560 MIDI step sequencer
// 16 step buttons + 16 LEDs (4 banks x 16 = 64 steps), 2 shift buttons, play/stop, tempo + swing pots
// 8 "channels" = 8 notes on MIDI channel 0. Per-channel loop length (polyrhythm).
// MIDI out (notes): Serial1 TX1 (pin 18) @31250 -> DIN-5 (5V-220R-pin4, TX1-220R-pin5, pin2 GND)
// MIDI clock out:   Serial2 TX2 (pin 16) @31250 -> second DIN-5, same circuit (24 PPQN + start/stop)
//
// Controls (step buttons 1-16 = index 0-15):
//   none      : toggle step in viewed bank (current channel)
//   Shift A   : 1-8 channel, 9-12 bank 1-4, 13-16 spare
//   Shift B   : 1-8 pattern, 9-15 spare, 16 (hold 1 s) = save to EEPROM
//   Accent held (no shift): step buttons toggle accents for the viewed bank. One accent
//             track per pattern, shared by all channels (indexed by each lane's own step).
//   A + B     : set last step of current channel = bank*16 + button

#include <Arduino.h>
#include <EEPROM.h>

// ---------- Pins ----------
const uint8_t STEP_BTN[16] = {22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37};
const uint8_t STEP_LED[16] = {38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 51, 50, 52, 53};  // 50/51 swapped to match board
const uint8_t SHIFT_A_PIN = 7;
const uint8_t SHIFT_B_PIN = 6;
const uint8_t PLAY_PIN = 5;
const uint8_t ACCENT_PIN = 4;
const uint8_t TEMPO_POT = A0;
const uint8_t SWING_POT = A1;

// ---------- MIDI config ----------
const uint8_t MIDI_CH = 0;                       // channel 1 on the wire
const uint8_t NOTES[8] = {30, 31, 32, 33, 34, 35, 36, 37};
const uint8_t VELOCITY_NORMAL = 60;
const uint8_t VELOCITY_ACCENT = 127;
const uint32_t CLOCK_LEAD_US = 30000;             // clock leads the notes by this much (compensates slaved gear's latency; tune by ear)

// ---------- Data ----------
uint16_t steps[8][8][4];     // [pattern][channel][bank] -> 16 step bits
uint16_t accents[8][4];      // [pattern][bank] -> 16 accent bits, shared by all channels
uint8_t lastStep[8][8];      // [pattern][channel] -> 0..63 (loop length - 1)

uint8_t curPat = 0, curCh = 0, viewBank = 0;

// ---------- Transport ----------
bool running = false;
uint32_t stepCounter = 0;
uint32_t tickTime = 0, fireTime = 0;             // micros: next MIDI clock tick, next (swung) step
uint8_t tickIdx = 0;                             // 0..5: clock tick within a 16th step (24 PPQN)
bool stepPending = false;                        // step scheduled, waiting for its swung fire time
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
Btn stepB[16], shiftA, shiftB, playB, accentB;

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
inline void midiOn(uint8_t note, uint8_t vel) { Serial1.write(0x90 | MIDI_CH); Serial1.write(note); Serial1.write(vel); }
inline void midiOff(uint8_t note) { Serial1.write(0x80 | MIDI_CH); Serial1.write(note); Serial1.write((uint8_t)0); }

void allNotesOff() {
  for (uint8_t c = 0; c < 8; c++) {
    if (noteOn[c]) { midiOff(NOTES[c]); noteOn[c] = false; }
  }
}

// ---------- Pots ----------
// One ADC read per call (non-blocking), alternating pots. The first read after switching ADC
// channel is discarded (mux settling), readings are smoothed (EMA), and a deadband stops the
// final value flipping on noise. Tempo is whole BPM, swing is whole percent.
int32_t tempoAcc = 0, swingAcc = 0;             // smoothed reading x16
uint16_t bpm = 120;
bool potTurn = false;

float potToUnit(int32_t acc) {                  // 0.0 .. 1.0, dead ends trimmed so extremes are reachable
  float v = (acc / 16.0f - 8.0f) / 1007.0f;
  return constrain(v, 0.0f, 1.0f);
}

void applyPots() {
  float bpmF = 40.0f + potToUnit(tempoAcc) * 200.0f;
  if (fabs(bpmF - bpm) > 0.75f) bpm = (uint16_t)(bpmF + 0.5f);       // deadband
  float swF = potToUnit(swingAcc) * 66.0f;
  if (fabs(swF - swingPct) > 0.75f) swingPct = (uint8_t)(swF + 0.5f);
  stepLenUs = 15000000UL / bpm;                 // 16th notes
}

void readPots() {                               // call every ~2 ms
  if (potTurn) {
    analogRead(SWING_POT);                      // discard
    swingAcc += ((int32_t)analogRead(SWING_POT) * 16 - swingAcc) >> 3;
  } else {
    analogRead(TEMPO_POT);                      // discard
    tempoAcc += ((int32_t)analogRead(TEMPO_POT) * 16 - tempoAcc) >> 3;
  }
  potTurn = !potTurn;
  applyPots();
}

void initPots() {
  analogRead(TEMPO_POT);
  tempoAcc = (int32_t)analogRead(TEMPO_POT) * 16;
  analogRead(SWING_POT);
  swingAcc = (int32_t)analogRead(SWING_POT) * 16;
  bpm = 40 + (uint16_t)(potToUnit(tempoAcc) * 200.0f + 0.5f);
  swingPct = (uint8_t)(potToUnit(swingAcc) * 66.0f + 0.5f);
  stepLenUs = 15000000UL / bpm;
}

// ---------- Sequencer ----------
inline uint8_t chLen(uint8_t pat, uint8_t ch) { return lastStep[pat][ch] + 1; }

void playStep() {
  uint32_t now = micros();
  uint32_t gate = stepLenUs / 2;
  for (uint8_t c = 0; c < 8; c++) {
    uint8_t p = stepCounter % chLen(curPat, c);
    if (steps[curPat][c][p >> 4] & (1u << (p & 15))) {
      if (noteOn[c]) midiOff(NOTES[c]);
      midiOn(NOTES[c], (accents[curPat][p >> 4] & (1u << (p & 15))) ? VELOCITY_ACCENT : VELOCITY_NORMAL);
      noteOn[c] = true;
      offTime[c] = now + gate;
    }
  }
}

void startSeq() {
  running = true;
  stepCounter = 0;
  tickIdx = 0;
  stepPending = false;
  Serial2.write(0xFA);                          // MIDI start
  tickTime = micros() + 1000;                   // >= 1 ms between start and first clock
}

void stopSeq() {
  running = false;
  stepPending = false;
  Serial2.write(0xFC);                          // MIDI stop
  allNotesOff();
}

// The clock tick is the master timeline: 6 ticks per 16th step (24 PPQN).
// Notes fire on step boundaries, delayed by CLOCK_LEAD_US plus swing on odd steps. The clock itself never swings.
void runClock() {
  uint32_t now = micros();
  for (uint8_t c = 0; c < 8; c++) {
    if (noteOn[c] && (int32_t)(now - offTime[c]) >= 0) {
      midiOff(NOTES[c]);
      noteOn[c] = false;
    }
  }
  if (!running) return;

  if ((int32_t)(now - tickTime) >= 0) {
    Serial2.write(0xF8);                        // MIDI clock
    if (tickIdx == 0) {                         // step boundary
      uint32_t d = CLOCK_LEAD_US + ((stepCounter & 1) ? (stepLenUs * swingPct) / 100 : 0);
      uint32_t maxD = stepLenUs - stepLenUs / 8;   // must fire before the next step boundary
      if (d > maxD) d = maxD;
      fireTime = tickTime + d;
      stepPending = true;
    }
    if (++tickIdx >= 6) tickIdx = 0;
    tickTime += stepLenUs / 6;
  }

  if (stepPending && (int32_t)(now - fireTime) >= 0) {
    playStep();
    stepCounter++;
    stepPending = false;
  }
}

// ---------- Save / load (EEPROM) ----------
// Layout: magic(4) | steps(512) | lastStep(64) | accents(64) | checksum(2) = 646 bytes
const uint32_t SAVE_MAGIC = 0x53455133UL;       // change this to invalidate old saves
const int EE_MAGIC = 0;
const int EE_STEPS = EE_MAGIC + sizeof(SAVE_MAGIC);
const int EE_LAST  = EE_STEPS + sizeof(steps);
const int EE_ACC   = EE_LAST + sizeof(lastStep);
const int EE_SUM   = EE_ACC + sizeof(accents);

uint32_t flashUntil = 0;                        // all LEDs on until this time (save confirmation)

uint16_t dataChecksum() {
  uint16_t sum = 0;
  const uint8_t *p = (const uint8_t *)steps;
  for (size_t i = 0; i < sizeof(steps); i++) sum = ((sum << 1) | (sum >> 15)) ^ p[i];
  p = (const uint8_t *)lastStep;
  for (size_t i = 0; i < sizeof(lastStep); i++) sum = ((sum << 1) | (sum >> 15)) ^ p[i];
  p = (const uint8_t *)accents;
  for (size_t i = 0; i < sizeof(accents); i++) sum = ((sum << 1) | (sum >> 15)) ^ p[i];
  return sum;
}

void initDefaults() {
  memset(steps, 0, sizeof(steps));
  memset(accents, 0, sizeof(accents));
  memset(lastStep, 15, sizeof(lastStep));       // default 16-step loops
}

bool loadState() {
  uint32_t magic;
  uint16_t stored;
  EEPROM.get(EE_MAGIC, magic);
  if (magic != SAVE_MAGIC) return false;
  EEPROM.get(EE_STEPS, steps);
  EEPROM.get(EE_LAST, lastStep);
  EEPROM.get(EE_ACC, accents);
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
  EEPROM.put(EE_ACC, accents);
  EEPROM.put(EE_SUM, dataChecksum());
  EEPROM.put(EE_MAGIC, SAVE_MAGIC);             // magic last, so a partial save is never valid
  if (running) tickTime = micros();             // resync clock after the blocking write
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
  pressed(accentB);
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
    } else if (accentB.down) {                  // accent edit (shared track)
      accents[curPat][viewBank] ^= (1u << i);
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
      on = (accentB.down ? accents[curPat][viewBank] : steps[curPat][curCh][viewBank]) & (1u << i);
      if (playPos >= 0 && (playPos >> 4) == viewBank && (playPos & 15) == i) on = !on;
    }
    digitalWrite(STEP_LED[i], on);
  }
}

// ---------- Setup / loop ----------
void setup() {
  Serial1.begin(31250);                         // MIDI notes out
  Serial2.begin(31250);                         // MIDI clock out
  for (uint8_t i = 0; i < 16; i++) {
    initBtn(stepB[i], STEP_BTN[i]);
    pinMode(STEP_LED[i], OUTPUT);
  }
  initBtn(shiftA, SHIFT_A_PIN);
  initBtn(shiftB, SHIFT_B_PIN);
  initBtn(playB, PLAY_PIN);
  initBtn(accentB, ACCENT_PIN);

  initDefaults();
  loadState();                                  // restore last save if present

  initPots();
}

void loop() {
  static uint32_t potT = 0;
  runClock();                                   // timing first
  handleButtons();
  checkSaveCombo();
  if (millis() - potT >= 2) { potT = millis(); readPots(); }
  updateLeds();
}
