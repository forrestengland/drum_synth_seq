
#include <Arduino.h>
#include <math.h>

#include <ESP32_Host_MIDI.h>
#include <USBConnection.h>

#include "driver/i2s_std.h"

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "esp_timer.h"

#include <Preferences.h>

#define NUM_PATTERNS 16
#define PATTERN_BUTTON_PIN 47

float targetPhaseIncrement = 0.0f;
bool glideActive = false;

constexpr float GLIDE_TIME_MS = 25.0f;   // time constant; ~60-75 ms to settle

enum RetriggerMode {
    RETRIG_ON_PITCH,   // old behavior: only when the pitch changes
    RETRIG_ALWAYS,     // new gated note
    RETRIG_NEVER       // slide: keep envelope and gate running
};

// Gate closes after this many of the 6 pulses in a step.
// 3 = 50%, 4 = 67%, 5 = 83%
constexpr uint8_t GATE_PULSES = 4;

constexpr float ACCENT_ENV_MULT   = 1.8f;   // filter envelope depth multiplier
constexpr float ACCENT_DECAY_MULT = 0.5f;   // envelope decay time multiplier
constexpr float ACCENT_RES_BOOST  = 0.15f;  // added to resonance

// Sync mode switch
#define SYNC_SWITCH_PIN 8

enum SyncMode {
    SYNC_EXTERNAL,
    SYNC_INTERNAL
};

SyncMode syncMode = SYNC_EXTERNAL;

// encoders
#define ENCODER_CLK 13
#define ENCODER_DT  14
#define ENCODER_SW  15

#define ENCODER2_CLK 17
#define ENCODER2_DT  18
#define ENCODER2_SW  21

// Gray-code encoder state

int encoderPosition = 0;
int encoder2Position = 0;

#define ACCENT_BUTTON_PIN 1
#define START_BUTTON_PIN  2

enum ButtonEvent { BTN_NONE, BTN_PRESSED, BTN_RELEASED };

struct Button {
    uint8_t pin;
    bool lastReading;
    bool stableState;
    uint32_t lastChange;

    Button(uint8_t p) : pin(p), lastReading(HIGH), stableState(HIGH), lastChange(0) {}
};

Button encoder1Button(ENCODER_SW);       // defined after the encoder pin #defines
Button encoder2Button(ENCODER2_SW);
Button accentButton(ACCENT_BUTTON_PIN);
Button startButton(START_BUTTON_PIN);
Button patternButton(PATTERN_BUTTON_PIN);

bool encoder2Turned = false;

// Internal tempo
float internalBpm = 120.0f;
constexpr float MIN_BPM = 40.0f;
constexpr float MAX_BPM = 240.0f;

esp_timer_handle_t internalClockTimer = nullptr;

// encoder button debounce
#define ENCODER_DEBOUNCE_MS 50

// clock
// MIDI clock / sequencer state

// OLED refresh state
uint32_t lastDisplayUpdate = 0;

// MIDI clock / sequencer state (shared between tasks)
volatile uint8_t clockCount = 0;
volatile bool midiRunning = false;
volatile uint8_t currentStep = 0;
volatile bool displayDirty = true;

volatile uint32_t lastClockMicros = 0;
volatile float externalBpm = 0.0f;

// Tempo is measured over one quarter note (24 pulses).
constexpr int CLOCK_WINDOW = 24;
uint32_t clockTimes[CLOCK_WINDOW];
uint8_t clockTimeIndex = 0;
uint8_t clockTimeCount = 0;

// sequence
#define NUM_STEPS 16

struct Step {
    uint8_t note = 60;  // Middle C
    bool active = false;
    bool accent = false;
    bool slide = false;
};

Step patternBank[NUM_PATTERNS][NUM_STEPS];
Step* pattern = patternBank[0];          // the pattern currently playing/editing
uint8_t loopLengths[NUM_PATTERNS];       // loopLength is stored per pattern
uint8_t currentPattern = 0;
uint8_t pendingPattern = 0;
bool patternSelectMode = false;

uint8_t selectedStep = 0;

bool sequencerAccent = false;   // set per step
bool accentActive = false;      // read by the audio task

// ---------- Autosave to flash ----------
constexpr uint32_t SAVE_DELAY_MS = 3000;
constexpr uint8_t  SAVE_VERSION  = 2;   // bump when SavedState or Step changes

struct SavedState {
	uint8_t version;
	uint8_t currentPattern;
	uint8_t loopLengths[NUM_PATTERNS];
	float   internalBpm;
	Step    patterns[NUM_PATTERNS][NUM_STEPS];
};

// Old single-pattern format, for migration.
struct SavedStateV1 {
    uint8_t version;
    uint8_t loopLength;
    float   internalBpm;
    Step    pattern[NUM_STEPS];
};

Preferences prefs;
SavedState lastSeenState;    // what we saw on the previous loop
SavedState lastSavedState;   // what is in flash
bool saveDirty = false;
uint32_t lastChangeMs = 0;

volatile uint8_t loopLength = NUM_STEPS;   // steps 0..loopLength-1 play
bool chordUsed = false;

// 5 pin midi in
#define MIDI_RX_PIN 16
#define MIDI_TX_PIN -1

// pots
#define I2C_SDA 11
#define I2C_SCL 12

Adafruit_ADS1115 ads;

// -------- I2S wiring --------
constexpr int I2S_BCLK = 5;
constexpr int I2S_LRCK = 6;
constexpr int I2S_DOUT = 7;

// -------- Audio settings --------
constexpr int SAMPLE_RATE = 44100;
constexpr int BLOCK_FRAMES = 128;
constexpr float MAX_AMPLITUDE = 0.25f;

// wavetable osc
enum Waveform {
    WAVE_SAW,
    WAVE_SQUARE
};

Waveform currentWaveform = WAVE_SAW;

// -------- Wavetable settings --------
#define WAVETABLE_SIZE 1024

float sawTable[WAVETABLE_SIZE];
float squareTable[WAVETABLE_SIZE];

// Oscillator phase, measured in wavetable samples.
//float phase = 0.0f;
//float phaseIncrement = 0.0f;

// -------- MIDI --------
USBConnection usbHost;
static uint64_t lastEventIndex = 0;

// Protect shared audio parameters between tasks
portMUX_TYPE synthMux = portMUX_INITIALIZER_UNLOCKED;

int activeNote = -1;
float targetAmplitude = 0.0f;
float currentAmplitude = 0.0f;
float phase = 0.0f;
float phaseIncrement = 0.0f;

int manualNote = -1;
int manualVelocity = 0;

int sequencerNote = -1;
int sequencerVelocity = 0;

// waveform switch
#define WAVE_SWITCH_PIN 4

// -------- Synth controls --------
float filterCutoff = 1000.0f;
float filterResonance = 0.0f;
float filterEnvDepth = 0.0f;
float envelopeDecay = 500.0f;

// -------- Resonant low-pass filter --------
float filterStage[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// -------- Filter envelope --------
bool filterEnvelopeTrigger = false;

// -------- I2S driver --------
i2s_chan_handle_t txHandle = nullptr;

// display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

Adafruit_SSD1306 display(
    SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET,
    400000UL, 400000UL   // clkDuring, clkAfter
);

// ---------- Interrupt-driven encoders ----------
#define ENCODER_TRANSITIONS_PER_DETENT 4   // try 2 if your encoder gives 2 counts per click

struct Encoder {
    uint8_t clkPin;
    uint8_t dtPin;
    volatile uint8_t state;
    volatile int8_t accumulator;
    volatile int32_t detents;
};

Encoder enc1 = { ENCODER_CLK,  ENCODER_DT,  0, 0, 0 };
Encoder enc2 = { ENCODER2_CLK, ENCODER2_DT, 0, 0, 0 };

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

    int8_t movement = encoderTable[(e->state << 2) | current];
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

SavedState captureState() {

	SavedState s;
    memset(&s, 0, sizeof(s));
    s.version = SAVE_VERSION;
    s.currentPattern = currentPattern;
    memcpy(s.loopLengths, loopLengths, sizeof(loopLengths));
    s.loopLengths[currentPattern] = loopLength;   // include unsaved live value
    s.internalBpm = internalBpm;
    memcpy(s.patterns, patternBank, sizeof(patternBank));
    return s;
}

void loadState() {

	for (int p = 0; p < NUM_PATTERNS; p++) loopLengths[p] = NUM_STEPS;

    prefs.begin("tb303", false);
    size_t len = prefs.getBytesLength("state");

    SavedState s;
    SavedStateV1 old;

    if (len == sizeof(s) &&
        prefs.getBytes("state", &s, sizeof(s)) == sizeof(s) &&
        s.version == SAVE_VERSION &&
        s.currentPattern < NUM_PATTERNS &&
        s.internalBpm >= MIN_BPM && s.internalBpm <= MAX_BPM) {

        for (int p = 0; p < NUM_PATTERNS; p++) {
            loopLengths[p] = (s.loopLengths[p] >= 1 && s.loopLengths[p] <= NUM_STEPS)
                                 ? s.loopLengths[p] : NUM_STEPS;
        }
        memcpy(patternBank, s.patterns, sizeof(patternBank));
        internalBpm = s.internalBpm;
        currentPattern = s.currentPattern;
        Serial.println("Patterns loaded from flash.");

    } else if (len == sizeof(old) &&
               prefs.getBytes("state", &old, sizeof(old)) == sizeof(old) &&
               old.version == 1 &&
               old.loopLength >= 1 && old.loopLength <= NUM_STEPS &&
               old.internalBpm >= MIN_BPM && old.internalBpm <= MAX_BPM) {

        // Keep the existing pattern as pattern 1.
        memcpy(patternBank[0], old.pattern, sizeof(old.pattern));
        loopLengths[0] = old.loopLength;
        internalBpm = old.internalBpm;
        currentPattern = 0;
        Serial.println("Migrated v1 pattern into slot 1.");
    }

    pattern = patternBank[currentPattern];
    loopLength = loopLengths[currentPattern];

    lastSeenState = captureState();
    lastSavedState = lastSeenState;

    // Write the new format immediately after a migration.
    if (len == sizeof(old)) {
        prefs.putBytes("state", &lastSavedState, sizeof(lastSavedState));
    }
}

void updateAutosave() {

	SavedState now = captureState();

    if (memcmp(&now, &lastSeenState, sizeof(now)) != 0) {
        lastSeenState = now;
        lastChangeMs = millis();
        saveDirty = true;
    }

    if (saveDirty && (millis() - lastChangeMs) >= SAVE_DELAY_MS) {
        saveDirty = false;

        // Skip the write if the state ended up back where it was saved.
        if (memcmp(&now, &lastSavedState, sizeof(now)) != 0) {
            prefs.putBytes("state", &now, sizeof(now));
            lastSavedState = now;
            Serial.println("Pattern saved.");
        }
    }
}

// Returns detents since last call and clears the counter.
int32_t takeEncoderDetents(Encoder& e) {
    portENTER_CRITICAL(&synthMux);
    int32_t d = e.detents;
    e.detents = 0;
    portEXIT_CRITICAL(&synthMux);
    return d;
}

void updateEncoder() {
    updateButtons();

    int32_t d1 = takeEncoderDetents(enc1);
    int32_t d2 = takeEncoderDetents(enc2);

    if (patternSelectMode) {
        if (d1 != 0) {
            pendingPattern =
                (pendingPattern + NUM_PATTERNS + (d1 % NUM_PATTERNS)) % NUM_PATTERNS;
            displayDirty = true;
        }
        return;   // encoder 2 is ignored while selecting
    }

    // Encoder 1: select step
    if (d1 != 0) {
        selectedStep = (selectedStep + NUM_STEPS + (d1 % NUM_STEPS)) % NUM_STEPS;
        displayDirty = true;
    }

    // Encoder 2: note, or tempo while its button is held
    if (d2 != 0) {
        bool buttonHeld = (encoder2Button.stableState == LOW);

        if (buttonHeld) {
            encoder2Turned = true;

            if (syncMode == SYNC_INTERNAL) {
                internalBpm = constrain(internalBpm + d2, MIN_BPM, MAX_BPM);
                if (midiRunning) restartInternalClock();
                displayDirty = true;
            }
        } else {
            int n = pattern[selectedStep].note + d2;
            pattern[selectedStep].note = constrain(n, 24, 96);
            displayDirty = true;
        }
    }
}

void setupEncoder() {
	
    pinMode(ENCODER_CLK,  INPUT_PULLUP);
    pinMode(ENCODER_DT,   INPUT_PULLUP);
    pinMode(ENCODER_SW,   INPUT_PULLUP);
    pinMode(ENCODER2_CLK, INPUT_PULLUP);
    pinMode(ENCODER2_DT,  INPUT_PULLUP);
    pinMode(ENCODER2_SW,  INPUT_PULLUP);

		pinMode(ACCENT_BUTTON_PIN, INPUT_PULLUP);
		pinMode(START_BUTTON_PIN,  INPUT_PULLUP);		
		pinMode(PATTERN_BUTTON_PIN, INPUT_PULLUP);
		
    enc1.state = (digitalRead(ENCODER_CLK) << 1) | digitalRead(ENCODER_DT);
    enc2.state = (digitalRead(ENCODER2_CLK) << 1) | digitalRead(ENCODER2_DT);

    attachInterruptArg(digitalPinToInterrupt(ENCODER_CLK),  encoderISR, &enc1, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(ENCODER_DT),   encoderISR, &enc1, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(ENCODER2_CLK), encoderISR, &enc2, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(ENCODER2_DT),  encoderISR, &enc2, CHANGE);
}

void internalClockCallback(void* arg) {
    if (syncMode != SYNC_INTERNAL || !midiRunning) {
        return;
    }
    sequencerPulse();
}

void restartInternalClock() {
    if (esp_timer_is_active(internalClockTimer)) {
        esp_timer_stop(internalClockTimer);
    }

    uint64_t interval =
        (uint64_t)(60000000.0f / (internalBpm * 24.0f));

    esp_timer_start_periodic(internalClockTimer, interval);
}

ButtonEvent updateButton(Button& b) {
    bool reading = digitalRead(b.pin);

    if (reading != b.lastReading) {
        b.lastChange = millis();
        b.lastReading = reading;
    }

    if ((millis() - b.lastChange) >= ENCODER_DEBOUNCE_MS &&
        reading != b.stableState) {
        b.stableState = reading;
        return reading == LOW ? BTN_PRESSED : BTN_RELEASED;
    }

    return BTN_NONE;
}

void toggleInternalRun();   // defined below

void switchPattern(uint8_t n) {
	
    loopLengths[currentPattern] = loopLength;   // remember the old pattern's loop
    currentPattern = n;
    pattern = patternBank[n];
    loopLength = loopLengths[n];
}

void confirmPatternSelect() {
	
    patternSelectMode = false;

    // Stop the internal clock first so its task can't race the switch.
    if (syncMode == SYNC_INTERNAL && esp_timer_is_active(internalClockTimer)) {
        esp_timer_stop(internalClockTimer);
    }

    stopSequencerNote();              // release the old pattern's note
    switchPattern(pendingPattern);

    if (syncMode == SYNC_INTERNAL) {
        // Start the new pattern from step 1, right now.
        clockCount = 0;
        currentStep = 0;
        midiRunning = true;
        triggerSequencerStep();
        restartInternalClock();
    } else if (currentStep >= loopLength) {
        // External: stay locked to the incoming clock, wrap if needed.
        currentStep = 0;
    }

    displayDirty = true;
}

void updateButtons() {

    // Pattern button: fires on press, works in both modes.
    if (updateButton(patternButton) == BTN_PRESSED) {
        if (!patternSelectMode) {
            patternSelectMode = true;
            pendingPattern = currentPattern;
            displayDirty = true;
        } else {
            confirmPatternSelect();
        }
        return;
    }

    // While selecting, ignore the other buttons so nothing is edited by accident.
    if (patternSelectMode) {
        updateButton(encoder1Button);
        updateButton(encoder2Button);
        updateButton(accentButton);
        updateButton(startButton);
        return;
    }	
	
    // Encoder 1 button: toggle step active
    if (updateButton(encoder1Button) == BTN_PRESSED) {
        pattern[selectedStep].active = !pattern[selectedStep].active;
        displayDirty = true;
    }

    // Encoder 2 button: slide on release, unless it was used for tempo
    ButtonEvent e2 = updateButton(encoder2Button);
    if (e2 == BTN_PRESSED) {
        encoder2Turned = false;
    } else if (e2 == BTN_RELEASED && !encoder2Turned) {
        pattern[selectedStep].slide = !pattern[selectedStep].slide;
        displayDirty = true;
    }

    // Accent + Start chord: set loop end to the current step.
    ButtonEvent accentEvent = updateButton(accentButton);
    ButtonEvent startEvent  = updateButton(startButton);

    bool accentDown = (accentButton.stableState == LOW);
    bool startDown  = (startButton.stableState == LOW);

    if (accentDown && startDown && !chordUsed) {
        chordUsed = true;

        // While running use the playing step, otherwise the selected step.
        uint8_t endStep = selectedStep;
        loopLength = endStep + 1;
        displayDirty = true;
    }

    // Single-button actions fire on release, unless the chord happened.
    if (accentEvent == BTN_RELEASED && !chordUsed) {
        pattern[selectedStep].accent = !pattern[selectedStep].accent;
        displayDirty = true;
    }

    if (startEvent == BTN_RELEASED && !chordUsed) {
        toggleInternalRun();
    }

    if (!accentDown && !startDown) {
        chordUsed = false;
    }		

}

float processFilter(
    float input,
    float coefficient,
    float resonance
) {
    // Resonance feedback.
    float feedback = filterStage[3] * resonance * 3.0f;

    float x = input - feedback;

    // Four cascaded low-pass stages.
    for (int i = 0; i < 4; i++) {
        filterStage[i] += coefficient * (x - filterStage[i]);
        x = filterStage[i];
    }

    return x;
}

float getOscillatorSample() {
  int index = (int)phase;
  float fraction = phase - index;

  int nextIndex = index + 1;

  if (nextIndex >= WAVETABLE_SIZE) {
    nextIndex = 0;
  }

  float* table =
      (currentWaveform == WAVE_SAW)
      ? sawTable
      : squareTable;

  float sample = table[index] +
      (table[nextIndex] - table[index]) * fraction;

  phase += phaseIncrement;

  while (phase >= WAVETABLE_SIZE) {
    phase -= WAVETABLE_SIZE;
  }

  return sample;
}

void setupAudio() {
  i2s_chan_config_t chanConfig =
      I2S_CHANNEL_DEFAULT_CONFIG(
          I2S_NUM_AUTO,
          I2S_ROLE_MASTER
      );

  ESP_ERROR_CHECK(
      i2s_new_channel(&chanConfig, &txHandle, nullptr)
  );

  i2s_std_config_t stdConfig = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),

      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
          I2S_DATA_BIT_WIDTH_16BIT,
          I2S_SLOT_MODE_STEREO
      ),

      .gpio_cfg = {
          .mclk = I2S_GPIO_UNUSED,
          .bclk = (gpio_num_t)I2S_BCLK,
          .ws = (gpio_num_t)I2S_LRCK,
          .dout = (gpio_num_t)I2S_DOUT,
          .din = I2S_GPIO_UNUSED,
          .invert_flags = {
              .mclk_inv = false,
              .bclk_inv = false,
              .ws_inv = false
          }
      }
  };

  ESP_ERROR_CHECK(
      i2s_channel_init_std_mode(txHandle, &stdConfig)
  );

  ESP_ERROR_CHECK(i2s_channel_enable(txHandle));

	//  Serial.println("I2S audio started.");
}

void updateSynthVoice(RetriggerMode mode) {
    // Manual keyboard notes take priority.
    int note = manualNote >= 0 ? manualNote : sequencerNote;
    int velocity = manualNote >= 0
        ? manualVelocity
        : sequencerVelocity;

    // Slide and forced retrigger only apply to sequencer notes.
    if (manualNote >= 0) {
        mode = RETRIG_ON_PITCH;
    }

    if (note < 0) {
        portENTER_CRITICAL(&synthMux);
        activeNote = -1;
        targetAmplitude = 0.0f;
        portEXIT_CRITICAL(&synthMux);
        return;
    }

    float frequency = 440.0f * powf(2.0f, (note - 69) / 12.0f);
    float increment = (frequency * WAVETABLE_SIZE) / SAMPLE_RATE;
    float amplitude = MAX_AMPLITUDE * (velocity / 127.0f);

    portENTER_CRITICAL(&synthMux);

    if (mode == RETRIG_ALWAYS ||
        (mode == RETRIG_ON_PITCH && activeNote != note)) {
        filterEnvelopeTrigger = true;
    }

		accentActive = (manualNote < 0) && sequencerAccent;

    activeNote = note;

    targetPhaseIncrement = increment;
    glideActive = (mode == RETRIG_NEVER);
		
    targetAmplitude = amplitude;

    portEXIT_CRITICAL(&synthMux);
}

void setNoteOn(int note, int velocity) {
    if (velocity <= 0) {
        setNoteOff(note);
        return;
    }

    manualNote = note;
    manualVelocity = velocity;

    updateSynthVoice(RETRIG_ALWAYS);
}

void setNoteOff(int note) {
    if (manualNote == note) {
        manualNote = -1;
        manualVelocity = 0;

        updateSynthVoice(RETRIG_ALWAYS);
    }
}

void triggerSequencerStep() {
	
    Step& step = pattern[currentStep];

    if (!step.active) {
        // Rest: close the gate.
        sequencerNote = -1;
        sequencerVelocity = 0;
        updateSynthVoice(RETRIG_ALWAYS);
        return;
    }

    // A slide only ties to the previous note if that gate is still open.
    // The previous step's gate-off is skipped when the next step slides,
    // so sequencerNote is still >= 0 here in that case.
    bool legato = step.slide && sequencerNote >= 0;

    sequencerNote = step.note;
    sequencerVelocity = step.accent ? 127 : 100;
		sequencerAccent = step.accent;

    updateSynthVoice(legato ? RETRIG_NEVER : RETRIG_ALWAYS);
}

// Called partway through each step to end the note.
void gateOffCurrentStep() {
    if (sequencerNote < 0) {
        return;
    }

    // If the next step slides, hold the gate open into it.
    Step& next = pattern[(currentStep + 1 >= loopLength) ? 0 : currentStep + 1];		
    if (next.active && next.slide) {
        return;
    }

    sequencerNote = -1;
    sequencerVelocity = 0;
    updateSynthVoice(RETRIG_ALWAYS);
}

// One MIDI-clock-equivalent pulse (24 per quarter note).
void sequencerPulse() {
    clockCount++;

    if (clockCount >= 6) {
        clockCount = 0;
				currentStep = (currentStep + 1 >= loopLength) ? 0 : currentStep + 1;
        triggerSequencerStep();
        displayDirty = true;
    } else if (clockCount == GATE_PULSES) {
        gateOffCurrentStep();
    }
}

void stopSequencerNote() {
    sequencerNote = -1;
    sequencerVelocity = 0;

    updateSynthVoice(RETRIG_ALWAYS);
}

void processMidi() {
  midiHandler.task();

  const auto& queue = midiHandler.getQueue();

  for (const auto& ev : queue) {
    // Process each queued event only once.
    if (ev.index <= lastEventIndex) {
      continue;
    }

    lastEventIndex = ev.index;

    uint8_t status = ev.statusCode & 0xF0;

    // Ignore pitch bend and control change for now.
    if (status == 0xE0 || status == 0xB0) {
      continue;
    }

    // Accept only Note On and Note Off.
    if (status != 0x90 && status != 0x80) {
      continue;
    }

    int note = ev.noteNumber;
    int velocity = ev.velocity7;
    int channel = (ev.statusCode & 0x0F) + 1;

    if (note < 0 || note > 127) {
      continue;
    }

    if (status == 0x90 && velocity > 0) {
      setNoteOn(note, velocity);
    } else {
      // Includes Note On with velocity zero.
      setNoteOff(note);
			//      Serial.printf("MIDI channel: %d\n", channel);
    }
  }
}

void audioTask(void* parameter) {
	
  int16_t buffer[BLOCK_FRAMES * 2];

  // Approximately 5 ms amplitude ramp.
  constexpr float RAMP_STEP =
      1.0f / (SAMPLE_RATE * 0.005f);

	float filterEnvelope = 0.0f;

  // Fraction of the remaining pitch distance covered per block.
  const float glideBlockCoeff =
      1.0f - expf(-(float)BLOCK_FRAMES /
                  (GLIDE_TIME_MS * 0.001f * SAMPLE_RATE));	

	for (;;) {
    float blockTargetAmplitude;
    float blockCutoff;
    float blockResonance;
    float blockPhaseIncrement;
		float blockDecay;
		float blockEnvDepth;
		bool triggerEnvelope;
		float blockTargetInc;
		bool blockGlide;
		bool blockAccent;

    portENTER_CRITICAL(&synthMux);

    blockTargetAmplitude = targetAmplitude;
    blockCutoff = filterCutoff;
    blockResonance = filterResonance;
		blockEnvDepth = filterEnvDepth;
		blockDecay = envelopeDecay;
    blockTargetInc = targetPhaseIncrement;
    blockGlide = glideActive;
		blockAccent = accentActive;

		// consume the trigger
		triggerEnvelope = filterEnvelopeTrigger;
		filterEnvelopeTrigger = false;

    portEXIT_CRITICAL(&synthMux);

		if (blockAccent) {
			blockEnvDepth *= ACCENT_ENV_MULT;
			blockDecay = max(blockDecay * ACCENT_DECAY_MULT, 20.0f);
			blockResonance = min(blockResonance + ACCENT_RES_BOOST, 0.9f);
    }

		float incStart = phaseIncrement;
    float incEnd;

    if (!blockGlide || incStart <= 0.0f) {
			// Normal note: jump straight to the new pitch.
			incStart = blockTargetInc;
			incEnd = blockTargetInc;
    } else {
			// Glide in the pitch (ratio) domain so it sounds even across octaves.
			incEnd = incStart * powf(blockTargetInc / incStart, glideBlockCoeff);
    }

    const float incStep = (incEnd - incStart) / BLOCK_FRAMES;

		// Restart envelope when a new MIDI note arrives.
		if (triggerEnvelope) {
			filterEnvelope = 1.0f;
		}

		// Calculate exponential envelope decay.
		// blockDecay is expressed in milliseconds.
		float decayMultiplier = expf(
																 -1.0f / (blockDecay * 0.001f * SAMPLE_RATE)
																 );

		// Apply envelope depth to the base cutoff.
		float effectiveCutoff =
			blockCutoff + (blockEnvDepth * filterEnvelope);

		effectiveCutoff = constrain(
																effectiveCutoff,
																20.0f,
																SAMPLE_RATE * 0.45f
																);

		// Calculate filter coefficient.
		float filterCoefficient =
			1.0f - expf(
									-TWO_PI * effectiveCutoff / SAMPLE_RATE
									);		


    for (int i = 0; i < BLOCK_FRAMES; i++) {

			phaseIncrement = incStart + incStep * (i + 1);
			
      if (currentAmplitude < blockTargetAmplitude) {
        currentAmplitude += RAMP_STEP;

        if (currentAmplitude > blockTargetAmplitude) {
          currentAmplitude = blockTargetAmplitude;
        }
      } else if (currentAmplitude > blockTargetAmplitude) {
        currentAmplitude -= RAMP_STEP;

        if (currentAmplitude < blockTargetAmplitude) {
          currentAmplitude = blockTargetAmplitude;
        }
      }

      float sample = 0.0f;

      if (currentAmplitude > 0.00001f) {
				float oscillatorSample = getOscillatorSample();

				float filteredSample = processFilter(
																						 oscillatorSample,
																						 filterCoefficient,
																						 blockResonance
																						 );

				sample = filteredSample * currentAmplitude;
				//				sample = getOscillatorSample() * currentAmplitude;

      }

      int16_t pcm = (int16_t)(sample * 32767.0f);

      // Duplicate mono signal to left and right.
      buffer[i * 2] = pcm;
      buffer[i * 2 + 1] = pcm;

			// Advance filter envelope.
			filterEnvelope *= decayMultiplier;

			if (filterEnvelope < 0.0001f) {
				filterEnvelope = 0.0f;
			}
    }

    size_t bytesWritten = 0;

    esp_err_t result = i2s_channel_write(
        txHandle,
        buffer,
        sizeof(buffer),
        &bytesWritten,
        1000
    );

    if (result != ESP_OK) {
			//      Serial.printf("I2S write error: %d\n", result);
    }
  }
}

void initializeWavetables() {
  for (int i = 0; i < WAVETABLE_SIZE; i++) {
    float position = (float)i / WAVETABLE_SIZE;

    // Sawtooth: -1.0 to nearly +1.0
    sawTable[i] = (2.0f * position) - 1.0f;

    // Square: first half positive, second half negative
    squareTable[i] = (position < 0.5f) ? 1.0f : -1.0f;
  }

	//  Serial.println("Wavetables initialized.");
}

void updateWaveformSwitch() {
  static Waveform lastWaveform = WAVE_SAW;

  Waveform selectedWaveform =
      digitalRead(WAVE_SWITCH_PIN) == LOW
      ? WAVE_SQUARE
      : WAVE_SAW;

  if (selectedWaveform != lastWaveform) {
    portENTER_CRITICAL(&synthMux);
    currentWaveform = selectedWaveform;
    portEXIT_CRITICAL(&synthMux);

    lastWaveform = selectedWaveform;

		/*    Serial.println(
        currentWaveform == WAVE_SAW
        ? "Waveform: SAW"
        : "Waveform: SQUARE"
				); */
  }
}

float readCutoffPot() {
  int16_t raw = ads.readADC_SingleEnded(0);

  // GAIN_ONE uses a +/-4.096V measurement range.
  // Convert the reading to a 0.0 - 1.0 range.
  float normalized = raw / 26400.0f;

  return constrain(normalized, 0.0f, 1.0f);
}

void updateSynthControls() {

  static uint32_t lastRead = 0;

  if (millis() - lastRead < 20) {
    return;
  }

  lastRead = millis();

  float pot0 = constrain(
      ads.readADC_SingleEnded(0) / 26400.0f,
      0.0f, 1.0f
  );

  float pot1 = constrain(
      ads.readADC_SingleEnded(1) / 26400.0f,
      0.0f, 1.0f
  );

  float pot2 = constrain(
      ads.readADC_SingleEnded(2) / 26400.0f,
      0.0f, 1.0f
  );

  float pot3 = constrain(
      ads.readADC_SingleEnded(3) / 26400.0f,
      0.0f, 1.0f
  );

  // Convert normalized readings to synth parameters.
  float newCutoff = 80.0f * powf(150.0f, pot0);
  float newResonance = pot1 * 0.75f;
  float newEnvDepth = pot2 * 8000.0f;
  float newDecay = 50.0f + pot3 * 1450.0f;

  portENTER_CRITICAL(&synthMux);

  filterCutoff = newCutoff;
  filterResonance = newResonance;
  filterEnvDepth = newEnvDepth;
  envelopeDecay = newDecay;

  portEXIT_CRITICAL(&synthMux);
}

void printSynthControls() {
  static uint32_t lastPrint = 0;

  if (millis() - lastPrint < 500) {
    return;
  }

  lastPrint = millis();

  float cutoff, resonance, envDepth, decay;

  portENTER_CRITICAL(&synthMux);

  cutoff = filterCutoff;
  resonance = filterResonance;
  envDepth = filterEnvDepth;
  decay = envelopeDecay;

  portEXIT_CRITICAL(&synthMux);

	/*  Serial.printf(
    "Cutoff: %.0f Hz | Resonance: %.2f | Env: %.0f Hz | Decay: %.0f ms\n",
    cutoff, resonance, envDepth, decay
		); */
}

//uint8_t clockCount = 0;
//uint32_t stepCount = 0;

void getNoteName(uint8_t note, char* output, size_t outputSize) {
    static const char* noteNames[] = {
        "C", "C#", "D", "D#", "E", "F",
        "F#", "G", "G#", "A", "A#", "B"
    };

    int octave = (note / 12) - 1;

    snprintf(
        output,
        outputSize,
        "%s%d",
        noteNames[note % 12],
        octave
    );
}

void midiClockTask(void* parameter) {
    for (;;) {
        while (Serial2.available()) {
            uint8_t data = Serial2.read();
            uint32_t now = micros();   // timestamp as soon as the byte is read

            if (data >= 0xF8) {        // real-time messages only
                handleMidiClock(data, now);
            }
        }

        // If the clock stops arriving, drop the tempo readout.
        if (syncMode == SYNC_EXTERNAL &&
            externalBpm > 0.0f &&
            (micros() - lastClockMicros) > 500000) {
            externalBpm = 0.0f;
            displayDirty = true;
        }

        vTaskDelay(1);  // poll at ~1 kHz
    }
}

void resetClockTempo() {
    clockTimeIndex = 0;
    clockTimeCount = 0;
}

void handleMidiClock(uint8_t data, uint32_t now) {

    if (syncMode != SYNC_EXTERNAL) {
        return;
    }

    switch (data) {

        case 0xFA:  // MIDI Start
            clockCount = 0;
            currentStep = 0;
            midiRunning = true;
            lastClockMicros = 0;
            triggerSequencerStep();
            displayDirty = true;
            break;

        case 0xFB:  // MIDI Continue
            midiRunning = true;
            lastClockMicros = 0;
            displayDirty = true;
            break;

        case 0xFC:  // MIDI Stop
            midiRunning = false;
            lastClockMicros = 0;
            stopSequencerNote();
            displayDirty = true;
            break;

        case 0xF8:  // MIDI Clock
        {
            // First pulse after start/stop, or a long gap: restart measurement.
            if (lastClockMicros == 0 || (now - lastClockMicros) > 250000) {
                resetClockTempo();
            }
            lastClockMicros = now;

            // When the window is full, clockTimeIndex points at the oldest
            // timestamp: exactly 24 pulses (one quarter note) ago.
            if (clockTimeCount == CLOCK_WINDOW) {
                uint32_t span = now - clockTimes[clockTimeIndex];
                float bpm = 60000000.0f / (float)span;

                if (bpm >= 20.0f && bpm <= 400.0f) {
                    // Hysteresis: only change the shown value when the
                    // measurement has clearly moved to a new integer BPM.
                    if (externalBpm == 0.0f ||
                        fabsf(bpm - externalBpm) > 0.75f) {
                        externalBpm = roundf(bpm);
                        displayDirty = true;
                    }
                }
            }

            clockTimes[clockTimeIndex] = now;
            clockTimeIndex = (clockTimeIndex + 1) % CLOCK_WINDOW;
            if (clockTimeCount < CLOCK_WINDOW) {
                clockTimeCount++;
            }

						if (midiRunning) {
                sequencerPulse();
            }
        }
    }
}

bool patternHasNotes(uint8_t p) {
    for (int i = 0; i < NUM_STEPS; i++) {
        if (patternBank[p][i].active) return true;
    }
    return false;
}

void drawPatternSelect() {
    for (int i = 0; i < NUM_PATTERNS; i++) {
        int x = 2 + ((i % 8) * 16);
        int numY = (i < 8) ? 10 : 29;
        int y    = (i < 8) ? 19 : 38;

        display.setCursor(x, numY);
        display.print(i + 1);

        if (patternHasNotes(i)) {
            display.fillRect(x, y, 7, 7, SSD1306_WHITE);
        } else {
            display.drawPixel(x + 3, y + 3, SSD1306_WHITE);
        }

        if (i == currentPattern) {
            display.drawRect(x - 1, y - 1, 9, 9, SSD1306_WHITE);
        }

        if (i == pendingPattern) {
            display.drawFastHLine(x, y + 8, 7, SSD1306_WHITE);
        }
    }

    char text[24];
    snprintf(text, sizeof(text), "SELECT P%02d  NOW P%02d",
             pendingPattern + 1, currentPattern + 1);
    display.setCursor(0, 55);
    display.print(text);
}

void updateDisplay() {

	// Only refresh when something has changed.
    if (!displayDirty) {
        return;
    }

    // Limit refresh rate to 20 Hz.
    if (millis() - lastDisplayUpdate < 50) {
        return;
    }

    lastDisplayUpdate = millis();
    displayDirty = false;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // --------------------------------
    // Header
    // --------------------------------

    display.setCursor(0, 0);
    display.print("TB-303");

		display.setCursor(66, 0);

		if (syncMode == SYNC_INTERNAL) {
			display.print("INT");
		} else {
			display.print("EXT");
		}

		char tempoText[8];

		float displayedBpm =
			syncMode == SYNC_INTERNAL
			? internalBpm
			: externalBpm;

		if (displayedBpm > 0.0f) {
			snprintf(
							 tempoText,
							 sizeof(tempoText),
							 "%3.0f",
							 displayedBpm
							 );
		} else {
			snprintf(tempoText, sizeof(tempoText), "---");
		}

		display.setCursor(98, 0);
		display.print(tempoText);		

    // Small indicator: filled when running, outlined when stopped.
    if (midiRunning) {
        display.fillCircle(60, 4, 2, SSD1306_WHITE);
    } else {
        display.drawCircle(60, 4, 2, SSD1306_WHITE);
    }

		if (patternSelectMode) {
        drawPatternSelect();
        display.display();
        return;
    }

    // --------------------------------
    // First row: steps 1-8
    // --------------------------------

    for (int i = 0; i < 8; i++) {
        int x = 2 + (i * 16);
        int y = 19;

        // Step number.
        display.setCursor(x, 10);
        display.print(i + 1);

        // Active step.
        if (pattern[i].active) {
            display.fillRect(x, y, 7, 7, SSD1306_WHITE);
        } else {
            display.drawPixel(x + 3, y + 3, SSD1306_WHITE);
        }

        // Playback position.
        if (currentStep == i) {
            display.drawRect(x - 1, y - 1, 9, 9, SSD1306_WHITE);
        }

        // Editing selection.
        if (selectedStep == i) {
            display.drawFastHLine(x, 27, 7, SSD1306_WHITE);
        }

				if (loopLength < NUM_STEPS && i == loopLength - 1) {
            display.drawFastVLine(x + 10, y - 1, 9, SSD1306_WHITE);
        }
    }

    // --------------------------------
    // Second row: steps 9-16
    // --------------------------------

    for (int i = 8; i < 16; i++) {
        int x = 2 + ((i - 8) * 16);
        int y = 38;

        // Step number.
        display.setCursor(x, 29);
        display.print(i + 1);

        // Active step.
        if (pattern[i].active) {
            display.fillRect(x, y, 7, 7, SSD1306_WHITE);
        } else {
            display.drawPixel(x + 3, y + 3, SSD1306_WHITE);
        }

        // Playback position.
        if (currentStep == i) {
            display.drawRect(x - 1, y - 1, 9, 9, SSD1306_WHITE);
        }

        // Editing selection.
        if (selectedStep == i) {
            display.drawFastHLine(x, 46, 7, SSD1306_WHITE);
        }

				if (loopLength < NUM_STEPS && i == loopLength - 1) {
					display.drawFastVLine(x + 10, y - 1, 9, SSD1306_WHITE);
        }
    }

    // --------------------------------
    // Selected step information
    // --------------------------------

    char noteText[8];
    getNoteName(pattern[selectedStep].note, noteText, sizeof(noteText));

    display.setCursor(0, 55);

    char statusText[24];

    snprintf(
        statusText,
        sizeof(statusText),
        "S%02d %s %s %s",
        selectedStep + 1,
        noteText,
        pattern[selectedStep].accent ? "ACC" : "---",
        pattern[selectedStep].slide ? "SLD" : "---"
    );

    display.print(statusText);

    char patText[8];
    snprintf(patText, sizeof(patText), "P%02d", currentPattern + 1);
    display.setCursor(108, 55);
    display.print(patText);		

    display.display();
}

void toggleInternalRun() {
    if (syncMode != SYNC_INTERNAL) {
        return;
    }

    if (!midiRunning) {
        // Start from step 1.
        clockCount = 0;
        currentStep = 0;
        midiRunning = true;
        triggerSequencerStep();
        restartInternalClock();
    } else {
        midiRunning = false;
        if (esp_timer_is_active(internalClockTimer)) {
            esp_timer_stop(internalClockTimer);
        }
        stopSequencerNote();
    }

    displayDirty = true;
}

void updateSyncSwitch() {

    static int lastReading = HIGH;
    static int stableReading = HIGH;
    static uint32_t lastChange = 0;

    int reading = digitalRead(SYNC_SWITCH_PIN);

    if (reading != lastReading) {
        lastChange = millis();
        lastReading = reading;
    }

    if (millis() - lastChange < 30) {
        return;
    }

    if (reading == stableReading) {
        return;
    }

    stableReading = reading;

    SyncMode newMode =
        reading == LOW
        ? SYNC_INTERNAL
        : SYNC_EXTERNAL;

    if (newMode == syncMode) {
        return;
    }

    syncMode = newMode;

    // Stop the previous clock source.
    if (esp_timer_is_active(internalClockTimer)) {
        esp_timer_stop(internalClockTimer);
    }

    stopSequencerNote();

    clockCount = 0;
    lastClockMicros = 0;
    midiRunning = false;

    currentStep = 0;

    if (syncMode == SYNC_INTERNAL) {

        // Trigger the first step immediately.
			//        triggerSequencerStep();

			//        restartInternalClock();

        Serial.println("SYNC: INTERNAL");

    } else {

        Serial.println("SYNC: EXTERNAL");
    }

    displayDirty = true;
}

void setup() {

	pinMode(SYNC_SWITCH_PIN, INPUT_PULLUP);

  Serial.begin(115200);

	loadState();

  delay(500);

  Serial.println();
  Serial.println("ESP32-S3 USB MIDI Sine Synth");

	initializeWavetables();

  setupAudio();

  BaseType_t taskResult = xTaskCreatePinnedToCore(
      audioTask,
      "AudioTask",
      4096,
      nullptr,
      3,
      nullptr,
      0
  );

  if (taskResult != pdPASS) {
    Serial.println("ERROR: Failed to create audio task!");

    while (true) {
      delay(1000);
    }
  }

  midiHandler.addTransport(&usbHost);
  usbHost.begin();
  midiHandler.begin();

	pinMode(WAVE_SWITCH_PIN, INPUT_PULLUP);	

  Serial.println("USB MIDI initialized.");
  Serial.println("Press a key on your MIDI keyboard.");

	Wire.begin(I2C_SDA, I2C_SCL);
	Wire.setClock(400000);

	if (!ads.begin(0x48)) {
		Serial.println("ADS1115 initialization failed!");

		while (true) {
			delay(1000);
		}
	}

	ads.setGain(GAIN_ONE);
	ads.setDataRate(RATE_ADS1115_860SPS);

	Serial.println("ADS1115 initialized.");

	Serial2.begin(
								31250,
								SERIAL_8N1,
								MIDI_RX_PIN,
								MIDI_TX_PIN
								);

	xTaskCreatePinnedToCore(
													midiClockTask,
													"MidiClock",
													4096,
													nullptr,
													5,        // higher than loop() (priority 1)
													nullptr,
													1         // core 1, away from the audio task on core 0
													);	

	Serial.println("DIN MIDI input ready");

	//	Wire.setClock(100000);

	if (!display.begin(
										 SSD1306_SWITCHCAPVCC,
										 OLED_ADDRESS
										 )) {
		Serial.println("OLED initialization failed");
		while (true);
	}

	setupEncoder();

	esp_timer_create_args_t timerArgs = {
    .callback = &internalClockCallback,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "internal_clock"
	};

	ESP_ERROR_CHECK(
									esp_timer_create(&timerArgs, &internalClockTimer)
									);
 
}

void loop() {

	processMidi();

	updateWaveformSwitch();
	updateSynthControls();

	updateEncoder();
	updateSyncSwitch();		

	updateDisplay();

	updateAutosave();
}
