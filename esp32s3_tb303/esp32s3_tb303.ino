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

#define RUN_SWITCH_PIN 40

// screen definitions
enum Screen { SCREEN_SYNTH, SCREEN_MENU, SCREEN_FX, SCREEN_DRUM, SCREEN_CHORDS };

// current screen
Screen currentScreen = SCREEN_MENU;
// currently highlighted menu index
uint8_t menuIndex = 0;                  // 0 synth, 1 fx, 2 drum, 3 chords

// drive amount for bass synth
volatile float driveAmount = 0.0f;      // 0..1

// global delay effect state
volatile float delayMs       = 300.0f;   // 40..740
volatile float delayFeedback = 0.4f;     // 0..1 (scaled to 0..0.85 in the audio task)
volatile float delayMix      = 0.0f;     // 0..1
uint8_t fxIndex = 0;                     // 0 drive, 1 time, 2 feedback, 3 mix

// keep track of pattern queuing state for pattern switching at end of pattern
volatile bool patternQueued = false;
// what pattern to play next
uint8_t queuedPattern = 0;

volatile bool drumPatternQueued = false;
uint8_t queuedDrumPattern = 0;

// midi output pin definition
#define MIDI_TX_PIN 41

// midi output channel
#define MIDI_OUT_CHANNEL 1          // 1-16
// note currently held on the output, -1 = none
int midiOutNote = -1;

// number of bass synth patterns
#define NUM_PATTERNS 16

// button 3 pin which controls pattern selection for the bass synth
#define PATTERN_BUTTON_PIN 47

// target phase increment for our slide functionality
float targetPhaseIncrement = 0.0f;
// whether we're currently generating audible frequency glide
bool glideActive = false;

// glide time
constexpr float GLIDE_TIME_MS = 25.0f;   // time constant; ~60-75 ms to settle

// retrigger modes
enum RetriggerMode {
    RETRIG_ON_PITCH,   // old behavior: only when the pitch changes
    RETRIG_ALWAYS,     // new gated note
    RETRIG_NEVER       // slide: keep envelope and gate running
};

// gate time for retriggering note on each step
// Gate closes after this many of the 6 pulses in a step.
// 3 = 50%, 4 = 67%, 5 = 83%
constexpr uint8_t GATE_PULSES = 4;

// how much accent affects the envelope
constexpr float ACCENT_ENV_MULT   = 1.8f;   // filter envelope depth multiplier
// how much accent affects decay
constexpr float ACCENT_DECAY_MULT = 0.5f;   // envelope decay time multiplier
// how much accent affects resonance
constexpr float ACCENT_RES_BOOST  = 0.03f;  // added to resonance
// how much feedback the filter uses
constexpr float FILTER_FEEDBACK = 2.0f;

// Sync mode switch pin - toggle switch for external or internal sync mode
#define SYNC_SWITCH_PIN 8

// available sync modes
enum SyncMode {
    SYNC_EXTERNAL,
    SYNC_INTERNAL
};

// current sync mode
SyncMode syncMode = SYNC_EXTERNAL;

// encoder pin definitions
#define ENCODER_CLK 14
#define ENCODER_DT  13
#define ENCODER_SW  15

#define ENCODER2_CLK 18
#define ENCODER2_DT  17
#define ENCODER2_SW  21

// Gray-code encoder state
int encoderPosition = 0;
int encoder2Position = 0;

constexpr int DELAY_SIZE = 32768;                 // must be a power of two
constexpr int DELAY_MASK = DELAY_SIZE - 1;
static int16_t delayBuf[DELAY_SIZE];              // zero-initialised

// other 2 push button pins
#define ACCENT_BUTTON_PIN 1
#define START_BUTTON_PIN  2

// button event
enum ButtonEvent { BTN_NONE, BTN_PRESSED, BTN_RELEASED };

// Button definition
struct Button {
    uint8_t pin;
    bool lastReading;
    bool stableState;
    uint32_t lastChange;

    Button(uint8_t p) : pin(p), lastReading(HIGH), stableState(HIGH), lastChange(0) {}
};

// Button instances for debouncing
Button encoder1Button(ENCODER_SW);
Button encoder2Button(ENCODER2_SW);
Button accentButton(ACCENT_BUTTON_PIN);
Button startButton(START_BUTTON_PIN);
Button patternButton(PATTERN_BUTTON_PIN);

// keep track if encoder 2 was turned while encoder 2 button was pressed
// because it sets slide for the current step if not turned
// or sets the internal clock tempo if it was turned while the button was pressed
bool encoder2Turned = false;

// Internal tempo
// current
float internalBpm = 120.0f;
// range
constexpr float MIN_BPM = 40.0f;
constexpr float MAX_BPM = 240.0f;

// timer
esp_timer_handle_t internalClockTimer = nullptr;

// encoder debounce time
#define ENCODER_DEBOUNCE_MS 50

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

// sequence length for bass synth
#define NUM_STEPS 32

// bass synth sequencer Step
struct Step {
    uint8_t note = 60;  // Middle C by default
    bool active = false;
    bool accent = false;
    bool slide = false;
};

// patterns data
Step patternBank[NUM_PATTERNS][NUM_STEPS];
Step* pattern = patternBank[0];          // the pattern currently playing/editing
uint8_t loopLengths[NUM_PATTERNS];       // loopLength is stored per pattern
uint8_t currentPattern = 0;
uint8_t pendingPattern = 0;
bool patternSelectMode = false;

uint8_t selectedStep = 0; // step the cursor is on - actions affect this step index

bool sequencerAccent = false;   // set per step
bool accentActive = false;      // read by the audio task

// ---------- Drums ----------
// Rows: kick, snare, closed hat, open hat. Columns: steps 1-16.
uint8_t currentDrumPattern = 0;
uint8_t pendingDrumPattern = 0;
bool drumPatternSelectMode = false; // mode for selecting a drum pattern

uint8_t drumPatternBank[NUM_PATTERNS][4][16];
uint8_t drumAccentBank[NUM_PATTERNS][4][16];
uint8_t (*drumPattern)[16] = drumPatternBank[0];
uint8_t (*drumAccent)[16]  = drumAccentBank[0];

#define DRUM_ACCENT_MUL 1.8f

volatile float drumLevel = 0.5f;            // 0..1
volatile uint8_t drumTriggerMask = 0, drumAccentMask = 0;       // bit 0 kick, 1 snare, 2 closed, 3 open
uint8_t drumCursor = 0;

// chord synth
uint8_t chordPatternBank[NUM_PATTERNS][4][16] = {};
uint8_t chordNotesBank[NUM_PATTERNS][4][16] = {};

uint8_t (*chordPattern)[16] = chordPatternBank[0];
uint8_t (*chordNotes)[16] = chordNotesBank[0];

uint8_t currentChordPattern = 0;
uint8_t pendingChordPattern = 0;
uint8_t queuedChordPattern = 0;

bool chordPatternSelectMode = false;
volatile bool chordPatternQueued = false;

volatile uint8_t chordCurNotes[4] = { 45, 48, 52, 55 };
volatile float chordLevel = 0.5f;
volatile uint8_t chordGateMask = 0;

uint8_t chordCursor = 0;

// ---------- Autosave to flash ----------
constexpr uint32_t SAVE_DELAY_MS = 3000;
constexpr uint8_t  SAVE_VERSION  = 3;   // bump when SavedState or Step changes

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

struct FxSaved {
    uint8_t version;
    float   timeMs;
    float   feedback;
    float   mix;
};

struct DrumSaved {
    uint8_t version;
    uint8_t currentPattern;
    float   drive;
    float   level;
    uint8_t pattern[NUM_PATTERNS][4][16];
    uint8_t accent[NUM_PATTERNS][4][16];
};

struct DrumSavedV2 {            // single pattern with accents
    uint8_t version;
    float   drive;
    float   level;
    uint8_t pattern[4][16];
    uint8_t accent[4][16];
};

struct DrumSavedV1 {            // single pattern, no accents
    uint8_t version;
    float   drive;
    float   level;
    uint8_t pattern[4][16];
};

struct ChordSaved {
    uint8_t version;
    uint8_t currentPattern;
    float level;
    uint8_t notes[NUM_PATTERNS][4][16];
    uint8_t pattern[NUM_PATTERNS][4][16];
};

// Previous single-pattern format.
struct ChordSavedV2 {
    uint8_t version;
    float level;
    uint8_t notes[4][16];
    uint8_t pattern[4][16];
};

DrumSaved lastSeenDrum;
DrumSaved lastSavedDrum;

Preferences prefs;
SavedState lastSeenState;    // what we saw on the previous loop
SavedState lastSavedState;   // what is in flash
bool saveDirty = false;
uint32_t lastChangeMs = 0;

volatile uint8_t loopLength = NUM_STEPS;   // steps 0..loopLength-1 for bass synth sequencer playback
bool chordUsed = false;

// 5 pin midi in pin
#define MIDI_RX_PIN 16

// i2c pin definitions for oled and adc
#define I2C_SDA 11
#define I2C_SCL 12

// adc definition for pot reads
Adafruit_ADS1115 ads;

// -------- I2S audio wiring --------
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

// current bass synth waveform
Waveform currentWaveform = WAVE_SAW;

// -------- Wavetable settings --------
#define WAVETABLE_SIZE 1024

float sawTable[WAVETABLE_SIZE];
float squareTable[WAVETABLE_SIZE];

// -------- usb MIDI host --------
USBConnection usbHost;
static uint64_t lastEventIndex = 0;

// Protect shared audio parameters between tasks
portMUX_TYPE synthMux = portMUX_INITIALIZER_UNLOCKED;

// bass synth state
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

// check if the filter resets on serial console
volatile uint32_t filterResets = 0;

// -------- Resonant 4-pole ladder (zero-delay feedback) --------
float ladderState[4] = {0.0f, 0.0f, 0.0f, 0.0f};

struct LadderCoeffs {
    float G, G2, G3, G4, beta;
};

constexpr float LADDER_DRIVE = 1.5f;   // input drive into the saturator

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
#define ENCODER_TRANSITIONS_PER_DETENT 4

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

void triggerChords(uint8_t step) {
    uint8_t mask = 0;
    for (int i = 0; i < 4; i++) {
        if (chordPattern[i][step]) {
            mask |= (1 << i);
            chordCurNotes[i] = chordNotes[i][step];
        }
    }
    chordGateMask = mask;
}

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

void triggerDrums(uint8_t step) {
    uint8_t mask = 0, acc = 0;
    for (int i = 0; i < 4; i++) {
        if (drumPattern[i][step]) {
            mask |= (1 << i);
            if (drumAccent[i][step]) acc |= (1 << i);
        }
    }
    if (mask) {
        portENTER_CRITICAL(&synthMux);
        drumTriggerMask |= mask;
        drumAccentMask = (drumAccentMask & ~mask) | acc;
        portEXIT_CRITICAL(&synthMux);
    }
}

void captureDrum(DrumSaved &d) {
    memset(&d, 0, sizeof(d));
    d.version = 3;
    d.currentPattern = currentDrumPattern;
    d.drive = driveAmount;
    d.level = drumLevel;

    memcpy(d.pattern, drumPatternBank, sizeof(drumPatternBank));
    memcpy(d.accent, drumAccentBank, sizeof(drumAccentBank));
}

void loadDrums() {
    // Default groove in pattern 1
    static const uint8_t groove[4][16] = {
        {1,0,0,0, 1,0,0,0, 1,0,0,0, 1,0,0,0},
        {0,0,0,0, 1,0,0,0, 0,0,0,0, 1,0,0,0},
        {1,0,1,0, 1,0,0,0, 1,0,1,0, 1,0,0,0},
        {0,0,0,0, 0,0,1,0, 0,0,0,0, 0,0,1,0},
    };
    memcpy(drumPatternBank[0], groove, sizeof(groove));

		static DrumSaved d;
		static DrumSavedV2 v2;
		static DrumSavedV1 v1;		

 size_t len = prefs.getBytesLength("drums");
    bool migrated = false;

    if (len == sizeof(d) &&
        prefs.getBytes("drums", &d, sizeof(d)) == sizeof(d) &&
        d.version == 3 && d.currentPattern < NUM_PATTERNS &&
        d.drive >= 0.0f && d.drive <= 1.0f &&
        d.level >= 0.0f && d.level <= 1.0f) {

        driveAmount = d.drive;
        drumLevel = d.level;
        currentDrumPattern = d.currentPattern;
        for (int p = 0; p < NUM_PATTERNS; p++)
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 16; c++) {
                    drumPatternBank[p][r][c] = d.pattern[p][r][c] ? 1 : 0;
                    drumAccentBank[p][r][c]  = d.accent[p][r][c] ? 1 : 0;
                }
        Serial.println("Drums loaded from flash.");

    } else if (len == sizeof(v2) &&
               prefs.getBytes("drums", &v2, sizeof(v2)) == sizeof(v2) &&
               v2.version == 2 &&
               v2.drive >= 0.0f && v2.drive <= 1.0f &&
               v2.level >= 0.0f && v2.level <= 1.0f) {

        driveAmount = v2.drive;
        drumLevel = v2.level;
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 16; c++) {
                drumPatternBank[0][r][c] = v2.pattern[r][c] ? 1 : 0;
                drumAccentBank[0][r][c]  = v2.accent[r][c] ? 1 : 0;
            }
        migrated = true;
        Serial.println("Migrated v2 drums into slot 1.");

    } else if (len == sizeof(v1) &&
               prefs.getBytes("drums", &v1, sizeof(v1)) == sizeof(v1) &&
               v1.version == 1 &&
               v1.drive >= 0.0f && v1.drive <= 1.0f &&
               v1.level >= 0.0f && v1.level <= 1.0f) {

        driveAmount = v1.drive;
        drumLevel = v1.level;
        for (int r = 0; r < 4; r++)
            for (int c = 0; c < 16; c++)
                drumPatternBank[0][r][c] = v1.pattern[r][c] ? 1 : 0;
        migrated = true;
        Serial.println("Migrated v1 drums into slot 1.");
    }

    drumPattern = drumPatternBank[currentDrumPattern];
    drumAccent  = drumAccentBank[currentDrumPattern];

		captureDrum(lastSeenDrum);
		lastSavedDrum = lastSeenDrum;		

    if (migrated) {
        prefs.putBytes("drums", &lastSavedDrum, sizeof(lastSavedDrum));
    }
}

FxSaved lastSeenFx;
FxSaved lastSavedFx;

FxSaved captureFx() {
    FxSaved f;
    memset(&f, 0, sizeof(f));
    f.version = 1;
    f.timeMs = delayMs;
    f.feedback = delayFeedback;
    f.mix = delayMix;
    return f;
}

void loadFx() {
    FxSaved f;
    if (prefs.getBytesLength("fx") == sizeof(f) &&
        prefs.getBytes("fx", &f, sizeof(f)) == sizeof(f) &&
        f.version == 1 &&
        f.timeMs >= 40.0f && f.timeMs <= 740.0f &&
        f.feedback >= 0.0f && f.feedback <= 1.0f &&
        f.mix >= 0.0f && f.mix <= 1.0f) {

        delayMs = f.timeMs;
        delayFeedback = f.feedback;
        delayMix = f.mix;
    }
    lastSeenFx = captureFx();
    lastSavedFx = lastSeenFx;
}

ChordSaved lastSeenChord;
ChordSaved lastSavedChord;

void captureChords(ChordSaved& c) {

    memset(&c, 0, sizeof(c));

    c.version = 3;
    c.currentPattern = currentChordPattern;
    c.level = chordLevel;

    memcpy(c.notes, chordNotesBank, sizeof(chordNotesBank));
    memcpy(c.pattern, chordPatternBank, sizeof(chordPatternBank));
}

void loadChords() {

    static ChordSaved c;
    static ChordSavedV2 old;

    // Default notes: A minor 7, one pitch per lane.
    const uint8_t defaults[4] = { 45, 48, 52, 55 };

    for (int p = 0; p < NUM_PATTERNS; p++) {
        for (int v = 0; v < 4; v++) {
            for (int s = 0; s < 16; s++) {
                chordNotesBank[p][v][s] = defaults[v];
                chordPatternBank[p][v][s] = 0;
            }
        }
    }

    currentChordPattern = 0;

    size_t len = prefs.getBytesLength("chords");
    bool migrated = false;

    if (len == sizeof(c) &&
        prefs.getBytes("chords", &c, sizeof(c)) == sizeof(c) &&
        c.version == 3 &&
        c.currentPattern < NUM_PATTERNS &&
        c.level >= 0.0f && c.level <= 1.0f) {

        chordLevel = c.level;
        currentChordPattern = c.currentPattern;

        memcpy(chordNotesBank, c.notes, sizeof(chordNotesBank));
        memcpy(chordPatternBank, c.pattern, sizeof(chordPatternBank));

        Serial.println("Chord patterns loaded from flash.");

    } else if (
        len == sizeof(old) &&
        prefs.getBytes("chords", &old, sizeof(old)) == sizeof(old) &&
        old.version == 2 &&
        old.level >= 0.0f && old.level <= 1.0f) {

        chordLevel = old.level;

        // Migrate the old single pattern into pattern 1.
        memcpy(chordNotesBank[0], old.notes, sizeof(old.notes));
        memcpy(chordPatternBank[0], old.pattern, sizeof(old.pattern));

        migrated = true;

        Serial.println("Migrated old chords into pattern 1.");
    }

    switchChordPattern(currentChordPattern);

    lastSeenChord = {};
    lastSavedChord = {};

    captureChords(lastSeenChord);
    lastSavedChord = lastSeenChord;

    if (migrated) {
        prefs.putBytes("chords", &lastSavedChord, sizeof(lastSavedChord));
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

		Serial.println("DEBUG: before loadDrums");
		loadDrums();

		Serial.println("DEBUG: before loadChords");
		loadChords();

		Serial.println("DEBUG: before loadFx");
		loadFx();

		Serial.println("DEBUG: before captureState");
		lastSeenState = captureState();

		Serial.println("DEBUG: before lastSavedState");
		lastSavedState = lastSeenState;

		Serial.println("DEBUG: loadState complete");		


    // Write the new format immediately after a migration.
    if (len == sizeof(old)) {
        prefs.putBytes("state", &lastSavedState, sizeof(lastSavedState));
    }
}

void updateAutosave() {

	SavedState now = captureState();
    static DrumSaved nowDrum;
		captureDrum(nowDrum);
    static ChordSaved nowChord;
		captureChords(nowChord);
    FxSaved nowFx = captureFx();                       // with the other captures		

    if (memcmp(&now, &lastSeenState, sizeof(now)) != 0 ||
        memcmp(&nowDrum, &lastSeenDrum, sizeof(nowDrum)) != 0 ||
        memcmp(&nowFx, &lastSeenFx, sizeof(nowFx)) != 0 ||   
        memcmp(&nowChord, &lastSeenChord, sizeof(nowChord)) != 0) {
        lastSeenState = now;
        lastSeenDrum = nowDrum;
        lastSeenChord = nowChord;
				lastSeenFx = nowFx;                           
        lastChangeMs = millis();
        saveDirty = true;
    }

    if (saveDirty && (millis() - lastChangeMs) >= SAVE_DELAY_MS) {
        saveDirty = false;

        if (memcmp(&now, &lastSavedState, sizeof(now)) != 0) {
            prefs.putBytes("state", &now, sizeof(now));
            lastSavedState = now;
        }
        if (memcmp(&nowDrum, &lastSavedDrum, sizeof(nowDrum)) != 0) {
            prefs.putBytes("drums", &nowDrum, sizeof(nowDrum));
            lastSavedDrum = nowDrum;
        }
        if (memcmp(&nowChord, &lastSavedChord, sizeof(nowChord)) != 0) {
            prefs.putBytes("chords", &nowChord, sizeof(nowChord));
            lastSavedChord = nowChord;
        }
        if (memcmp(&nowFx, &lastSavedFx, sizeof(nowFx)) != 0) {
					prefs.putBytes("fx", &nowFx, sizeof(nowFx));
					lastSavedFx = nowFx;                            // inside the change block
				}
        Serial.println("Saved.");
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

	if (currentScreen != SCREEN_SYNTH) {
		handleMenuEncoders(d1, d2);
		return;
	}

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
    midiOutRealtime(0xF8);
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

void handleMenuButtons(ButtonEvent patEvent, ButtonEvent accentEvent, ButtonEvent startEvent, ButtonEvent sel, ButtonEvent e2) {

	// menu screen
	if (currentScreen == SCREEN_MENU) {

		// encoder one pressed, switch screen to selection
		if (sel == BTN_PRESSED) {
			if (menuIndex == 0)      currentScreen = SCREEN_SYNTH;
			else if (menuIndex == 1) currentScreen = SCREEN_FX;
			else if (menuIndex == 2) currentScreen = SCREEN_DRUM;
			else currentScreen = SCREEN_CHORDS;
			displayDirty = true;
		}
			
	} else { // other than menu screen or synth screen

		// first button pressed - return to menu
		if (startEvent == BTN_PRESSED) {
			currentScreen = SCREEN_MENU;
			displayDirty = true;
			return;
		}
						
		// Drum screen
		if (currentScreen == SCREEN_DRUM) {
			static bool accentChord = false;

			if (patEvent == BTN_PRESSED) {
				bool accentHeld = (accentButton.stableState == LOW);
				if (accentHeld) accentChord = true;

				if (!drumPatternSelectMode) {
					drumPatternSelectMode = true;
					pendingDrumPattern = currentDrumPattern;
				} else if (accentHeld) {
					switchDrumPatternNow();
				} else {
					confirmDrumPatternSelect();
				}
				displayDirty = true;
			}
			else if (!drumPatternSelectMode) {
				if (sel == BTN_PRESSED) {
					uint8_t& cell = drumPattern[drumCursor / 16][drumCursor % 16];
					cell = cell ? 0 : 1;
					displayDirty = true;
				}

				// Accent toggles on release, unless it was part of the chord.
				if (accentEvent == BTN_RELEASED && !accentChord) {
					uint8_t& a = drumAccent[drumCursor / 16][drumCursor % 16];
					a = a ? 0 : 1;
					displayDirty = true;
				}
			}

			if (accentEvent == BTN_RELEASED) accentChord = false;
		}		

		// Chord sequencer controls
		if (currentScreen == SCREEN_CHORDS) {

			if (patEvent == BTN_PRESSED) {

        bool accentHeld = (accentButton.stableState == LOW);

        if (!chordPatternSelectMode) {

					chordPatternSelectMode = true;
					pendingChordPattern = currentChordPattern;

        } else if (accentHeld) {

					// Accent + Pattern = immediate switch
					switchChordPatternNow();

        } else {

					// Pattern again = confirm / queue
					confirmChordPatternSelect();
        }

        displayDirty = true;
			}

			// Only allow editing when not selecting a pattern.
			else if (!chordPatternSelectMode && sel == BTN_PRESSED) {

        uint8_t& cell =
					chordPattern[chordCursor / 16][chordCursor % 16];

        cell = cell ? 0 : 1;
        displayDirty = true;
			}
		}
	}

	// Keep the synth buttons from toggling when you come back.
	chordUsed = true; // keeps track of button combination presses
	if (accentButton.stableState == HIGH && startButton.stableState == HIGH) {
		chordUsed = false;
	}
}

void handleMenuEncoders(int32_t d1, int32_t d2) {

	if (currentScreen == SCREEN_MENU && d1 != 0) {
		menuIndex = (menuIndex + 4 + (d1 % 4)) % 4;
		displayDirty = true;
	}

	if (currentScreen == SCREEN_DRUM) {

		if (drumPatternSelectMode) {
			if (d1 != 0) {
				pendingDrumPattern =
					(pendingDrumPattern + NUM_PATTERNS + (d1 % NUM_PATTERNS)) % NUM_PATTERNS;
				displayDirty = true;
			}
			return;
		}

		if (d2 != 0) {
			drumLevel = constrain(drumLevel + d2 * 0.05f, 0.0f, 1.0f);
			displayDirty = true;
		}

		if (d1 != 0) {
			drumCursor = (drumCursor + 64 + (d1 % 64)) % 64;
			displayDirty = true;
		}		
	}

	if (currentScreen == SCREEN_CHORDS) {

    if (chordPatternSelectMode) {

			if (d1 != 0) {
				pendingChordPattern =
					(pendingChordPattern + NUM_PATTERNS +
					 (d1 % NUM_PATTERNS)) % NUM_PATTERNS;

				displayDirty = true;
			}

			return;  // Ignore encoder 2 while selecting.
    }

    if (d1 != 0) {
			chordCursor = (chordCursor + 64 + (d1 % 64)) % 64;
			displayDirty = true;
    }

    if (d2 != 0) {

			if (encoder2Button.stableState == LOW) {

				uint8_t& n =
					chordNotes[chordCursor / 16][chordCursor % 16];

				n = constrain((int)n + d2, 24, 96);

			} else {

				chordLevel =
					constrain(chordLevel + d2 * 0.05f, 0.0f, 1.0f);
			}

			displayDirty = true;
    }
	}

	if (currentScreen == SCREEN_FX) {
		if (d1 != 0) {
			fxIndex = (fxIndex + 4 + (d1 % 4)) % 4;
			displayDirty = true;
		}
		if (d2 != 0) {
			switch (fxIndex) {
			case 0: driveAmount    = constrain(driveAmount    + d2 * 0.05f, 0.0f, 1.0f); break;
			case 1: delayMs        = constrain(delayMs        + d2 * 10.0f, 40.0f, 740.0f); break;
			case 2: delayFeedback  = constrain(delayFeedback  + d2 * 0.05f, 0.0f, 1.0f); break;
			case 3: delayMix       = constrain(delayMix       + d2 * 0.05f, 0.0f, 1.0f); break;
			}
			displayDirty = true;
		}
	}	
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

void switchPatternNow() {

	patternSelectMode = false;
	patternQueued = false;          // an immediate switch overrides any queued one

	if (pendingPattern != currentPattern) {
		switchPattern(pendingPattern);

		// Keep the playhead where it is. If the new pattern is shorter than
		// the current step, park on its last step so the next pulse wraps to 1.
		if (currentStep >= loopLength) {
			currentStep = loopLength - 1;
		}
	}

	displayDirty = true;
}

void confirmPatternSelect() {

	patternSelectMode = false;

	if (midiRunning) {
		// Playing: switch at the next pattern wrap, in time.
		queuedPattern = pendingPattern;
		patternQueued = (queuedPattern != currentPattern);
	} else {
		// Stopped: switch right away.
		stopSequencerNote();
		switchPattern(pendingPattern);
		currentStep = 0;
		clockCount = 0;
	}

	displayDirty = true;
}

void switchChordPattern(uint8_t n) {

    currentChordPattern = n;
    chordPattern = chordPatternBank[n];
    chordNotes = chordNotesBank[n];

    for (int v = 0; v < 4; v++) {
        chordCurNotes[v] = chordNotes[v][0];
    }
}

void switchChordPatternNow() {

    chordPatternSelectMode = false;
    chordPatternQueued = false;

    if (pendingChordPattern != currentChordPattern) {
        switchChordPattern(pendingChordPattern);
    }

    displayDirty = true;
}

void confirmChordPatternSelect() {

    chordPatternSelectMode = false;

    if (midiRunning) {
        queuedChordPattern = pendingChordPattern;
        chordPatternQueued =
            (queuedChordPattern != currentChordPattern);
    } else {
        switchChordPattern(pendingChordPattern);
        chordPatternQueued = false;
    }

    displayDirty = true;
}

void switchDrumPattern(uint8_t n) {
	
    currentDrumPattern = n;
    drumPattern = drumPatternBank[n];
    drumAccent = drumAccentBank[n];		
}

void switchDrumPatternNow() {

	drumPatternSelectMode = false;
	drumPatternQueued = false;          // an immediate switch overrides any queued one

	if (pendingDrumPattern != currentDrumPattern) {
		switchDrumPattern(pendingDrumPattern);
	}

	displayDirty = true;
}

void confirmDrumPatternSelect() {
    drumPatternSelectMode = false;

    if (midiRunning) {
        queuedDrumPattern = pendingDrumPattern;
        drumPatternQueued = (queuedDrumPattern != currentDrumPattern);
    } else {
        switchDrumPattern(pendingDrumPattern);
        drumPatternQueued = false;
    }
    displayDirty = true;
}

void updateButtons() {

	// grab all button states
	ButtonEvent patEvent = updateButton(patternButton);
	ButtonEvent accentEvent = updateButton(accentButton);
	ButtonEvent startEvent  = updateButton(startButton);
	ButtonEvent sel = updateButton(encoder1Button);
	ButtonEvent e2  = updateButton(encoder2Button);

	// handleMenuButtons handles button presses when we're not on the bass synth screen
	if (currentScreen != SCREEN_SYNTH) {
		handleMenuButtons(patEvent, accentEvent, startEvent, sel, e2);
		return;
	}

	if (patEvent == BTN_PRESSED) {	// if pattern button was just pressed

		bool accentHeld = (accentButton.stableState == LOW); // find out if accent button is being held

		if (accentHeld) {
			// Chord: stop the accent button's release from toggling accent.
			chordUsed = true;
		}

		if (!patternSelectMode) { // go into bass synth pattern select mode if we're not already there
			patternSelectMode = true;
			pendingPattern = currentPattern;
			displayDirty = true;
		} else if (accentHeld) {
			switchPatternNow();
		} else {
			confirmPatternSelect();
		}
		return;
	}	

	// While selecting, ignore the other buttons so nothing is edited by accident.
	if (patternSelectMode) {
		return; 
	}	
	
	// Encoder 1 button: toggle step active
	if (sel == BTN_PRESSED) {
		pattern[selectedStep].active = !pattern[selectedStep].active;
		displayDirty = true;
	}

	// Encoder 2 button: slide on release, unless it was used for tempo
	if (e2 == BTN_PRESSED) {
		encoder2Turned = false;
	} else if (e2 == BTN_RELEASED && !encoder2Turned) {
		pattern[selectedStep].slide = !pattern[selectedStep].slide;
		displayDirty = true;
	}

	// bass synth seq last step setting ----
	bool accentDown = (accentButton.stableState == LOW);
	bool startDown  = (startButton.stableState == LOW);

	if (accentDown && startDown && !chordUsed) {

		chordUsed = true;

		uint8_t endStep = selectedStep;
		loopLength = endStep + 1;
		displayDirty = true;
	}

	// Single-button actions fire on release, unless the chord happened.
	if (accentEvent == BTN_RELEASED && !chordUsed) {
		pattern[selectedStep].accent = !pattern[selectedStep].accent;
		displayDirty = true;
	}

	// start button 1 now goes to menu
	if (startEvent == BTN_RELEASED && !chordUsed) {
		patternSelectMode = false;
		currentScreen = SCREEN_MENU;
		menuIndex = 0;
		displayDirty = true;
	}

	if (!accentDown && !startDown) {
		chordUsed = false;
	}		
}

static inline float fastTanh(float x) {
    if (x > 3.0f) return 1.0f;
    if (x < -3.0f) return -1.0f;
    float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

LadderCoeffs makeLadderCoeffs(float cutoffHz) {
    float g = tanf(PI * cutoffHz / SAMPLE_RATE);
    LadderCoeffs c;
    c.G = g / (1.0f + g);
    c.G2 = c.G * c.G;
    c.G3 = c.G2 * c.G;
    c.G4 = c.G3 * c.G;
    c.beta = 1.0f / (1.0f + g);
    return c;
}

float processLadder(float input, const LadderCoeffs& c, float k) {
    // Contribution of the filter's stored state to the output.
    float S = c.beta * (c.G3 * ladderState[0] +
                        c.G2 * ladderState[1] +
                        c.G  * ladderState[2] +
                              ladderState[3]);

    // Solve the feedback loop for this sample, then saturate.
    float u = (input * LADDER_DRIVE - k * S) / (1.0f + k * c.G4);
    u = fastTanh(u);

    // Four trapezoidal one-pole stages.
    float x = u;
    for (int i = 0; i < 4; i++) {
        float v = (x - ladderState[i]) * c.G;
        float y = v + ladderState[i];
        ladderState[i] = y + v;
        x = y;
    }

    // Safety net, should never trigger now.
    if (!isfinite(x) || fabsf(x) > 8.0f) {
        for (int i = 0; i < 4; i++) ladderState[i] = 0.0f;
        filterResets++;
        return 0.0f;
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

void midiOutNoteOn(int note, int velocity, bool legato) {
    uint8_t msg[6];
    int len = 0;
    uint8_t onStatus  = 0x90 | ((MIDI_OUT_CHANNEL - 1) & 0x0F);
    uint8_t offStatus = 0x80 | ((MIDI_OUT_CHANNEL - 1) & 0x0F);

    if (legato && midiOutNote >= 0) {
        if (note == midiOutNote) {
            return;                 // tied note at the same pitch: nothing to send
        }
        // Slide: new note on first, then old note off, so a mono synth glides.
        msg[len++] = onStatus;  msg[len++] = note;         msg[len++] = velocity;
        msg[len++] = offStatus; msg[len++] = midiOutNote;  msg[len++] = 0;
    } else {
        if (midiOutNote >= 0) {
            msg[len++] = offStatus; msg[len++] = midiOutNote; msg[len++] = 0;
        }
        msg[len++] = onStatus; msg[len++] = note; msg[len++] = velocity;
    }

    midiOutNote = note;
    Serial2.write(msg, len);        // one write per event, so messages can't interleave
}

void midiOutNoteOff() {
    if (midiOutNote < 0) {
        return;
    }
    uint8_t msg[3] = {
        (uint8_t)(0x80 | ((MIDI_OUT_CHANNEL - 1) & 0x0F)),
        (uint8_t)midiOutNote,
        0
    };
    midiOutNote = -1;
    Serial2.write(msg, 3);
}

inline void midiOutRealtime(uint8_t byte) {
    if (syncMode == SYNC_INTERNAL) {
        Serial2.write(byte);
    }
}

void triggerSequencerStep() {

    triggerDrums(currentStep % 16);
		triggerChords(currentStep % 16);
	
    Step& step = pattern[currentStep];

    if (!step.active) {
        // Rest: close the gate.
        sequencerNote = -1;
        sequencerVelocity = 0;
				midiOutNoteOff();				
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
		midiOutNoteOn(step.note, sequencerVelocity, legato);

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
    midiOutNoteOff();		
    updateSynthVoice(RETRIG_ALWAYS);
}

// One MIDI-clock-equivalent pulse (24 per quarter note).
void sequencerPulse() {

	clockCount++;

	if (clockCount >= 6) {
		clockCount = 0;

		if (currentStep + 1 >= loopLength) {
			currentStep = 0;
			if (patternQueued) {
				patternQueued = false;
				switchPattern(queuedPattern);
			}
						
		} else {
			currentStep++;
		}

		if (currentStep % 16 == 0 && drumPatternQueued) {
			drumPatternQueued = false;
			switchDrumPattern(queuedDrumPattern);
		}

		if (currentStep % 16 == 0 && chordPatternQueued) {
			chordPatternQueued = false;
			switchChordPattern(queuedChordPattern);
		}		

		triggerSequencerStep();					
		triggerSequencerStep();
		displayDirty = true;
	} else if (clockCount == GATE_PULSES) {
		gateOffCurrentStep();
	}
}

void stopSequencerNote() {
    sequencerNote = -1;
    sequencerVelocity = 0;
		chordGateMask = 0;
    midiOutNoteOff();
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

  // Pot smoothing: one-pole per block, ~15 ms time constant.
  const float potSmoothCoeff =
      1.0f - expf(-(float)BLOCK_FRAMES / (0.015f * SAMPLE_RATE));

  float smCutoffLog = -1.0f;   // log2 of cutoff, -1 = uninitialised
  float smResonance = 0.0f;
  float smEnvDepth  = 0.0f;
  float smDecay     = 500.0f;
  float smDrive = 0.0f;	

  // ---- Drum voices ----
  auto decayMul = [](float ms) {
      return expf(-1.0f / (ms * 0.001f * SAMPLE_RATE));
  };
  const float kickAmpMul   = decayMul(120.0f);
  const float kickPitchMul = decayMul(25.0f);
  const float snareNoiseMul = decayMul(70.0f);
  const float snareToneMul = decayMul(40.0f);
  const float chMul        = decayMul(15.0f);
  const float ohMul        = decayMul(120.0f);
  const float chokeMul     = decayMul(2.0f);     // fast fade when the closed hat chokes the open hat
  const float snareToneInc = TWO_PI * 190.0f / SAMPLE_RATE;

  uint32_t noiseState = 22222;
  float kickPhase = 0, kickEnv = 0, kickPitchEnv = 0;
  float snareEnv = 0, snareToneEnv = 0, snareTonePhase = 0;
  float chEnv = 0, ohEnv = 0;
  bool ohChoked = false;
  float hatLp = 0, snareLp = 0;
  float smDrumLevel = 0.0f;

  // ---- Chord voices ----
  float chordPhase[4] = {0, 0, 0, 0};     // 0..1
  float chordEnv[4]   = {0, 0, 0, 0};
  float smChordLevel  = 0.0f;
  const float chordEnvStep = 1.0f / (SAMPLE_RATE * 0.008f);   // 8 ms attack/release

  // ---- Delay ----
  int   delayWrite = 0;
  float smDelaySamples = 0.0f;
  float smDelayFb = 0.0f, smDelayMix = 0.0f;
  float delayLp = 0.0f;
  const float delayTimeCoeff =
      1.0f - expf(-(float)BLOCK_FRAMES / (0.08f * SAMPLE_RATE));   // ~80 ms glide
	float blockDelayMs, blockDelayFb, blockDelayMix;

  float voiceAcc[4] = {1.0f, 1.0f, 1.0f, 1.0f};   // gain latched per hit	
	
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
		float blockDrive;
		uint8_t blockDrumMask;
		uint8_t blockDrumAccentMask;		
		float blockDrumLevel;
		uint8_t blockChordMask; uint8_t blockChordNotes[4]; float blockChordLevel;		

    portENTER_CRITICAL(&synthMux);

    blockTargetAmplitude = targetAmplitude;
    blockCutoff = filterCutoff;
    blockResonance = filterResonance;
		blockEnvDepth = filterEnvDepth;
		blockDecay = envelopeDecay;
    blockTargetInc = targetPhaseIncrement;
    blockGlide = glideActive;
		blockAccent = accentActive;

    blockDrumMask = drumTriggerMask;
    blockDrumAccentMask = drumAccentMask;		
    drumTriggerMask = 0;
		drumAccentMask = 0;
    blockDrumLevel = drumLevel;

		// consume the trigger
		triggerEnvelope = filterEnvelopeTrigger;
		filterEnvelopeTrigger = false;

		blockDrive = driveAmount;

    blockChordMask = chordGateMask;
    blockChordLevel = chordLevel;
    for (int v = 0; v < 4; v++) blockChordNotes[v] = chordCurNotes[v];

    blockDelayMs  = delayMs;
    blockDelayFb  = delayFeedback;
    blockDelayMix = delayMix;		

    portEXIT_CRITICAL(&synthMux);

    if (blockDrumMask & 1) { kickEnv = 1.0f; kickPitchEnv = 1.0f; kickPhase = 0.0f;
                             voiceAcc[0] = (blockDrumAccentMask & 1) ? DRUM_ACCENT_MUL : 1.0f; }
    if (blockDrumMask & 2) { snareEnv = 1.0f; snareToneEnv = 1.0f;
                             voiceAcc[1] = (blockDrumAccentMask & 2) ? DRUM_ACCENT_MUL : 1.0f; }
    if (blockDrumMask & 4) { chEnv = 1.0f; ohChoked = true;
                             voiceAcc[2] = (blockDrumAccentMask & 4) ? DRUM_ACCENT_MUL : 1.0f; }
    if (blockDrumMask & 8) { ohEnv = 1.0f; ohChoked = false;
                             voiceAcc[3] = (blockDrumAccentMask & 8) ? DRUM_ACCENT_MUL : 1.0f; }		

    smDrumLevel += potSmoothCoeff * (blockDrumLevel - smDrumLevel);
    const float drumGain = smDrumLevel * 0.6f;

    smChordLevel += potSmoothCoeff * (blockChordLevel - smChordLevel);
    const float chordGain = smChordLevel * 0.12f;		

    float chordInc[4];
    for (int v = 0; v < 4; v++) {
        chordInc[v] = 440.0f * powf(2.0f, (blockChordNotes[v] - 69) / 12.0f) / SAMPLE_RATE;
    }

    float targetDelaySamples = blockDelayMs * 0.001f * SAMPLE_RATE;
    if (smDelaySamples < 1.0f) smDelaySamples = targetDelaySamples;   // first block
    smDelaySamples += delayTimeCoeff * (targetDelaySamples - smDelaySamples);
    smDelayFb  += potSmoothCoeff * (blockDelayFb  * 0.85f - smDelayFb);
    smDelayMix += potSmoothCoeff * (blockDelayMix - smDelayMix);		

    float targetCutoffLog = log2f(blockCutoff);

    if (smCutoffLog < 0.0f) {
        // First block: start at the current pot positions.
        smCutoffLog = targetCutoffLog;
        smResonance = blockResonance;
        smEnvDepth  = blockEnvDepth;
        smDecay     = blockDecay;
    }

    smCutoffLog += potSmoothCoeff * (targetCutoffLog - smCutoffLog);
    smResonance += potSmoothCoeff * (blockResonance - smResonance);
    smEnvDepth  += potSmoothCoeff * (blockEnvDepth  - smEnvDepth);
    smDecay     += potSmoothCoeff * (blockDecay     - smDecay);

    blockCutoff    = exp2f(smCutoffLog);
    blockResonance = smResonance;
    blockEnvDepth  = smEnvDepth;
    blockDecay     = smDecay;

    smDrive += potSmoothCoeff * (blockDrive - smDrive);
    float driveGain = 1.0f + smDrive * 9.0f;
    float driveNorm = 1.0f / fastTanh(driveGain);		

		if (blockAccent) {
			blockEnvDepth *= ACCENT_ENV_MULT;
			blockDecay = max(blockDecay * ACCENT_DECAY_MULT, 20.0f);
			blockResonance = min(blockResonance + ACCENT_RES_BOOST, 1.0f);
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
		float decayMultiplier = expf(-1.0f / (blockDecay * 0.001f * SAMPLE_RATE));

		// Apply envelope depth to the base cutoff.
		float effectiveCutoff = blockCutoff + (blockEnvDepth * filterEnvelope);

		effectiveCutoff = constrain(effectiveCutoff, 20.0f, SAMPLE_RATE * 0.45f);

    LadderCoeffs ladder = makeLadderCoeffs(effectiveCutoff);

    // Resonance 0..1 maps to feedback 0..4.2. Self-oscillation starts near 4.
    float k = constrain(blockResonance, 0.0f, 1.0f) * 4.2f;		

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

      // ---- Drums ----
      noiseState ^= noiseState << 13;
      noiseState ^= noiseState >> 17;
      noiseState ^= noiseState << 5;
      float n = (int32_t)noiseState * 4.656613e-10f;       // -1..1

      hatLp   += 0.5f  * (n - hatLp);
      snareLp += 0.15f * (n - snareLp);
      float hat      = n - hatLp;                          // bright noise
      float snNoise  = n - snareLp;                        // wider noise

      float kickFreq = 45.0f + 110.0f * kickPitchEnv;
      kickPhase += TWO_PI * kickFreq / SAMPLE_RATE;
      if (kickPhase > TWO_PI) kickPhase -= TWO_PI;
      float kick = sinf(kickPhase) * kickEnv;

      snareTonePhase += snareToneInc;
      if (snareTonePhase > TWO_PI) snareTonePhase -= TWO_PI;
      float snare = snNoise * snareEnv * 0.8f +
                    sinf(snareTonePhase) * snareToneEnv * 0.5f;

      float drums = kick * 0.9f * voiceAcc[0] +
                    snare * 0.5f * voiceAcc[1] +
                    hat * chEnv * 0.35f * voiceAcc[2] +
                    hat * ohEnv * 0.3f * voiceAcc[3];			

      sample += drums * drumGain;

      kickEnv      *= kickAmpMul;
      kickPitchEnv *= kickPitchMul;
      snareEnv     *= snareNoiseMul;
      snareToneEnv *= snareToneMul;
      chEnv        *= chMul;
      ohEnv        *= ohChoked ? chokeMul : ohMul;			

      if (currentAmplitude > 0.00001f) {
				float oscillatorSample = getOscillatorSample();

        float filteredSample = processLadder(oscillatorSample, ladder, k);

        if (smDrive > 0.001f) {
            filteredSample = fastTanh(filteredSample * driveGain) * driveNorm;
        }				

				sample += filteredSample * currentAmplitude;
				//				sample = getOscillatorSample() * currentAmplitude;

      }

      // ---- Chords ----
      float chords = 0.0f;
      for (int v = 0; v < 4; v++) {
          float target = ((blockChordMask >> v) & 1) ? 1.0f : 0.0f;

          if (chordEnv[v] < target)      chordEnv[v] = min(chordEnv[v] + chordEnvStep, target);
          else if (chordEnv[v] > target) chordEnv[v] = max(chordEnv[v] - chordEnvStep, target);

          if (chordEnv[v] > 0.0f) {
              chordPhase[v] += chordInc[v];
              if (chordPhase[v] >= 1.0f) chordPhase[v] -= 1.0f;
              chords += (chordPhase[v] < 0.5f ? 1.0f : -1.0f) * chordEnv[v];
          }
      }
      sample += chords * chordGain;

      // ---- Delay (all voices) ----
      float rpos = (float)delayWrite - smDelaySamples;
      if (rpos < 0.0f) rpos += DELAY_SIZE;
      int i0 = (int)rpos;
      float fr = rpos - i0;
      int i1 = (i0 + 1) & DELAY_MASK;
      float wet = (delayBuf[i0] + (delayBuf[i1] - delayBuf[i0]) * fr) * (1.0f / 32768.0f);

      delayLp += 0.4f * (wet - delayLp);                       // darken the repeats
      float toBuf = fastTanh(sample + delayLp * smDelayFb);
      delayBuf[delayWrite] = (int16_t)(toBuf * 32767.0f);
      delayWrite = (delayWrite + 1) & DELAY_MASK;

      sample += wet * smDelayMix;			

			float out = fastTanh(sample);
			//      float out = constrain(sample, -1.0f, 1.0f); // hard clip
			
      int16_t pcm = (int16_t)(out * 32767.0f);			

      // Duplicate mono signal to left and right.
      buffer[i * 2] = pcm;
      buffer[i * 2 + 1] = pcm;

			// Advance filter envelope.
			filterEnvelope *= decayMultiplier;

			if (filterEnvelope < 0.0001f) {
				filterEnvelope = 0.0f;
			}
		}

    if (kickEnv < 1e-4f) kickEnv = 0.0f;
    if (snareEnv < 1e-4f) snareEnv = 0.0f;
    if (snareToneEnv < 1e-4f) snareToneEnv = 0.0f;
    if (chEnv < 1e-4f) chEnv = 0.0f;
    if (ohEnv < 1e-4f) ohEnv = 0.0f;		

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

  if (millis() - lastRead < 10) {
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
  float newResonance = pot1;
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

 bool drumPatternHasNotes(uint8_t p) {
	for (int d = 0; d < 4; d++)
		for (int i = 0; i < 16; i++)
			if (drumPatternBank[p][d][i]) return true;
	return false;
}

bool chordPatternHasNotes(uint8_t p) {

	 for (int lane = 0; lane < 4; lane++) {
		 for (int step = 0; step < 16; step++) {
			 if (chordPatternBank[p][lane][step]) {
				 return true;
			 }
		 }
	 }

	 return false;
}

void drawChordPatternSelect() {

    for (int i = 0; i < NUM_PATTERNS; i++) {

        int x = 2 + ((i % 8) * 16);
        int numY = (i < 8) ? 10 : 29;
        int y = (i < 8) ? 19 : 38;

        display.setCursor(x, numY);
        display.print(i + 1);

        if (chordPatternHasNotes(i)) {
            display.fillRect(x, y, 7, 7, SSD1306_WHITE);
        } else {
            display.drawPixel(x + 3, y + 3, SSD1306_WHITE);
        }

        if (i == currentChordPattern) {
            display.drawRect(x - 1, y - 1, 9, 9, SSD1306_WHITE);
        }

        if (i == pendingChordPattern) {
            display.drawFastHLine(x, y + 8, 7, SSD1306_WHITE);
        }
    }

    char text[24];

    snprintf(
        text,
        sizeof(text),
        "SELECT P%02d  NOW P%02d",
        pendingChordPattern + 1,
        currentChordPattern + 1
    );

    display.setCursor(0, 55);
    display.print(text);
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

void drawDrumPatternSelect() {

	for (int i = 0; i < NUM_PATTERNS; i++) {

		int x = 2 + ((i % 8) * 16);
		int numY = (i < 8) ? 10 : 29;
		int y    = (i < 8) ? 19 : 38;

		display.setCursor(x, numY);
		display.print(i + 1);

		if (drumPatternHasNotes(i)) {
			display.fillRect(x, y, 7, 7, SSD1306_WHITE);
		} else {
			display.drawPixel(x + 3, y + 3, SSD1306_WHITE);
		}

		if (i == currentDrumPattern) {
			display.drawRect(x - 1, y - 1, 9, 9, SSD1306_WHITE);
		}

		if (i == pendingDrumPattern) {
			display.drawFastHLine(x, y + 8, 7, SSD1306_WHITE);
		}
	}

	char text[24];
	snprintf(text, sizeof(text), "SELECT P%02d  NOW P%02d",
					 pendingDrumPattern + 1, currentDrumPattern + 1);
	display.setCursor(0, 55);
	display.print(text);
}

void drawLaneGrid(uint8_t grid[4][16], uint8_t cursor) {
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 16; c++) {
            int x = 8 + c * 7;
            int y = 38 + r * 6;
            if (grid[r][c]) display.fillRect(x, y, 5, 4, SSD1306_WHITE);
            else display.drawPixel(x + 2, y + 2, SSD1306_WHITE);
            if (r * 16 + c == cursor) {
                display.drawRect(x - 1, y - 1, 7, 6, SSD1306_WHITE);
            }
        }
    }
    if (midiRunning) {
        display.drawFastHLine(8 + (currentStep % 16) * 7, 35, 5, SSD1306_WHITE);
    }
}

void drawMenuScreen() {

	display.setCursor(0, 12);

	if (currentScreen == SCREEN_MENU) {
		const char* items[4] = { "Synth", "FX", "Drum", "Chords" };
		for (int i = 0; i < 4; i++) {
			display.setCursor(8, 16 + i * 11);
			display.print(i == menuIndex ? "> " : "  ");
			display.print(items[i]);
		}
	} else if (currentScreen == SCREEN_FX) {
        display.setCursor(0, 14);
        display.print("FX");

        const char* names[4] = { "Drive", "Time", "Fdbk", "Mix" };
        float values[4] = {
            driveAmount,
            (delayMs - 40.0f) / 700.0f,
            delayFeedback,
            delayMix
        };

        for (int i = 0; i < 4; i++) {
            int y = 24 + i * 10;
            display.setCursor(0, y);
            display.print(i == fxIndex ? ">" : " ");
            display.print(names[i]);
            display.drawRect(42, y, 60, 7, SSD1306_WHITE);
            display.fillRect(42, y, (int)(values[i] * 60.0f), 7, SSD1306_WHITE);
        }

        display.setCursor(106, 24 + 10);
        display.print((int)delayMs / 10);   // time in tens of ms		
	} else if (currentScreen == SCREEN_CHORDS) {

		if (chordPatternSelectMode) {
			drawChordPatternSelect();
			display.display();
			return;
		}
		display.setCursor(0, 14);
		display.print("CHORDS");

		char noteText[8], label[16];
		uint8_t lane = chordCursor / 16;
		uint8_t stp  = chordCursor % 16;
		getNoteName(chordNotes[lane][stp], noteText, sizeof(noteText));
		snprintf(label, sizeof(label), "L%d %s %02d", lane + 1, noteText, stp + 1);		
		display.setCursor(50, 14);
		display.print(label);

		display.setCursor(8, 24);
		display.print("> Level");
		display.drawRect(58, 24, 56, 8, SSD1306_WHITE);
		display.fillRect(58, 24, (int)(chordLevel * 56.0f), 8, SSD1306_WHITE);

		drawLaneGrid(chordPattern, chordCursor);
		
	} else {		// draw drum screen

		if (drumPatternSelectMode) {
        drawDrumPatternSelect();
        display.display();
        return;
    }

		display.setCursor(0, 14);
		char dp[10];
		if (drumPatternQueued) snprintf(dp, sizeof(dp), "DRM>P%02d", queuedDrumPattern + 1);
		else                   snprintf(dp, sizeof(dp), "DRM P%02d", currentDrumPattern + 1);
		display.print(dp);

		static const char* drumNames[4] = { "KICK", "SNARE", "CHAT", "OHAT" };
		char label[16];
		snprintf(label, sizeof(label), "%s %02d",
						 drumNames[drumCursor / 16], (drumCursor % 16) + 1);
		display.setCursor(50, 14);
		display.print(label);

		if (drumAccent[drumCursor / 16][drumCursor % 16]) {
			display.setCursor(100, 14);
			display.print("ACC");
		}

		display.setCursor(8, 24);
		display.print("> Level");
		display.drawRect(58, 24, 56, 8, SSD1306_WHITE);
		display.fillRect(58, 24, (int)(drumLevel * 56.0f), 8, SSD1306_WHITE);

		// 4 x 16 pattern grid with a playhead tick
		for (int r = 0; r < 4; r++) {
			for (int c = 0; c < 16; c++) {
				int x = 8 + c * 7;
				int y = 38 + r * 6;
				if (drumPattern[r][c]) display.fillRect(x, y, 5, 4, SSD1306_WHITE);
				else display.drawPixel(x + 2, y + 2, SSD1306_WHITE);
				if (r * 16 + c == drumCursor) {
					display.drawRect(x - 1, y - 1, 7, 6, SSD1306_WHITE);
				}
			}
		}
		if (midiRunning) {
			display.drawFastHLine(8 + (currentStep % 16) * 7, 35, 5, SSD1306_WHITE);
		}
	}
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
    display.print("EE303+");

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

    if (currentScreen != SCREEN_SYNTH) {			
        drawMenuScreen();
        display.display();
        return;
    }

		if (patternSelectMode) {
        drawPatternSelect();
        display.display();
        return;
    }

    // --------------------------------
    // First row: steps 1-8
    // --------------------------------
    for (int i = 0; i < NUM_STEPS; i++) {
        int x = 2 + ((i % 8) * 16);
        int y = 11 + ((i / 8) * 11);

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
            display.drawFastHLine(x, y + 9, 7, SSD1306_WHITE);
        }

        // Loop end marker.
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
    if (patternQueued) {
        snprintf(patText, sizeof(patText), ">P%02d", queuedPattern + 1);
    } else {
        snprintf(patText, sizeof(patText), "P%02d", currentPattern + 1);
    }
    display.setCursor(patternQueued ? 102 : 108, 55);
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
				midiOutRealtime(0xFA);          // Start
        triggerSequencerStep();
        restartInternalClock();
    } else {
        midiRunning = false;
        if (esp_timer_is_active(internalClockTimer)) {
            esp_timer_stop(internalClockTimer);
        }
				midiOutRealtime(0xFC);          // Stop
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

		if (syncMode == SYNC_INTERNAL && midiRunning) {
			Serial2.write((uint8_t)0xFC);   // tell followers to stop
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

			if (digitalRead(RUN_SWITCH_PIN) == LOW) {
				toggleInternalRun();   // midiRunning is false here, so this starts
			}

			Serial.println("SYNC: INTERNAL");

    } else {

        Serial.println("SYNC: EXTERNAL");
    }

    displayDirty = true;
}

void setup() {

	pinMode(RUN_SWITCH_PIN, INPUT_PULLUP);

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

void updateRunSwitch() {
    static bool initialised = false;
    static int lastReading = HIGH;
    static int stableReading = HIGH;
    static uint32_t lastChange = 0;

    int reading = digitalRead(RUN_SWITCH_PIN);

    if (!initialised) {
        // Adopt the switch position at boot without acting on it.
        initialised = true;
        lastReading = stableReading = reading;
        return;
    }

    if (reading != lastReading) {
        lastChange = millis();
        lastReading = reading;
    }

    if ((millis() - lastChange) < 30 || reading == stableReading) {
        return;
    }

    stableReading = reading;

    // Only the internal clock can be started and stopped from here.
    if (syncMode != SYNC_INTERNAL) {
        return;
    }

    bool wantRun = (reading == LOW);
    if (wantRun != midiRunning) {
        toggleInternalRun();
    }
}

void loop() {

	processMidi();

	updateWaveformSwitch();
	updateSynthControls();

	updateEncoder();
	updateSyncSwitch();
	updateRunSwitch();

	updateDisplay();

	updateAutosave();

  static uint32_t lastResets = 0;
  if (filterResets != lastResets) {
    lastResets = filterResets;
    Serial.printf("Filter reset #%lu\n", (unsigned long)filterResets);
  }	
}
