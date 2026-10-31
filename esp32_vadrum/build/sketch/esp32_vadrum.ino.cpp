#line 1 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
/*
 * ESP32 analog-style drum voice synth
 *   DIN MIDI in -> BD / SD / closed hat / open hat -> PCM5102A (I2S)
 *
 * Controls
 *   Normal mode:  encoder 1..4 turn  = volume of BD / SD / CH / OH
 *                 encoder 1..4 click = preview (trigger) that voice
 *   INST held:    encoder 1..4 turn  = TUNE / DECAY / BEND / TONE of the *selected* voice
 *                 encoder 1..4 click = select voice BD / SD / CH / OH
 *
 * TONE meaning per voice: BD = click+drive, SD = noise/tone mix, hats = resonance.
 *
 * Saved state (flash / NVS): all knob values + selected voice
 *   - auto-saved 3 s after the last change
 *   - or hold INST ~2 s without touching anything to save right now
 *   - reloaded at power-up
 *   - hold INST while powering on to wipe the saved state and start from defaults
 *
 * MIDI notes: BD 30, SD 31, CH 32, OH 33  (edit midiNote[] below)
 */

#include <Arduino.h>
#include <atomic>
#include <Preferences.h>
#include <math.h>
#include "driver/i2s_std.h"

// ---------- Pins ----------
static constexpr int MIDI_RX_PIN = 16;
HardwareSerial MIDI(2);

static constexpr int I2S_BCLK = 26;
static constexpr int I2S_LRCK = 25;
static constexpr int I2S_DOUT = 27;

// Encoders 1..4 = BD, SD, CH, OH.
// GPIO34/35 are input-only with no internal pull-ups: external 10k to 3.3V needed on enc 4 A+B.
struct EncPins { uint8_t a, b, sw; };
static const EncPins encPins[4] = {
    { 18, 19, 13 },   // 1 - BD
    { 22, 21, 14 },   // 2 - SD
    { 33, 32, 17 },   // 3 - CH
    { 35, 34, 23 },   // 4 - OH
};
static constexpr int INST_SW = 4;

// ---------- Tunables ----------
static constexpr int   SAMPLE_RATE   = 44100;
static constexpr int   BLOCK_FRAMES  = 128;
static constexpr float PI_F          = 3.14159265358979323846f;
static constexpr float MASTER_GAIN   = 0.5f;      // keep conservative until you know your output level
static constexpr float ENC_STEP      = 0.02f;     // normalized change per encoder tick (50 ticks full range)
static constexpr int   ENC_DIR       = 1;         // set to -1 to reverse all encoders
static constexpr uint32_t DEBOUNCE_MS = 10;
static constexpr bool     AUTO_SAVE      = true;   // save a few seconds after the last change
static constexpr uint32_t SAVE_DELAY_MS  = 3000;
static constexpr uint32_t LONG_PRESS_MS  = 2000;   // INST held this long, untouched = save now

static const uint8_t midiNote[4] = { 30, 31, 32, 33 };   // BD, SD, CH, OH

// ---------- Voice definitions ----------
enum { P_TUNE, P_DECAY, P_BEND, P_TONE, P_LEVEL, NP };
enum { V_BD, V_SD, V_CH, V_OH, NV };
enum { T_BD, T_SD, T_HH };

struct VoiceDef {
    const char *name, *toneName;
    int   type;
    float tuneLo, tuneHi;     // Hz (exponential map)
    float decLo, decHi;       // seconds to -60 dB (exponential map)
    float bendMax;            // start pitch = tune * (1 + bend)
    float ptau;               // pitch env time constant (s)
    float def[NP];            // tune, decay, bend, tone, level (normalized)
};

static const VoiceDef defs[NV] = {
    { "BD", "CLICK", T_BD,   30.f,   120.f, 0.05f, 1.2f, 6.0f, 0.030f, {0.35f, 0.45f, 0.45f, 0.25f, 0.80f} },
    { "SD", "NOISE", T_SD,  120.f,   400.f, 0.04f, 0.6f, 2.0f, 0.015f, {0.40f, 0.50f, 0.30f, 0.60f, 0.80f} },
    { "CH", "RESO",  T_HH, 3000.f, 14000.f, 0.01f, 0.25f, 1.5f, 0.020f, {0.70f, 0.30f, 0.00f, 0.30f, 0.70f} },
    { "OH", "RESO",  T_HH, 3000.f, 14000.f, 0.10f, 2.0f, 1.5f, 0.020f, {0.70f, 0.50f, 0.00f, 0.30f, 0.70f} },
};
static const char *paramName[4] = { "TUNE", "DECAY", "BEND", nullptr };

// ---------- State shared between loop task (core 1) and audio task (core 0) ----------
static volatile float P[NV][NP];            // normalized 0..1, single-word writes are atomic
static std::atomic<uint8_t> trigVel[NV];    // pending trigger velocity (1..127), 0 = none

// ---------- Types ----------
// Kept above every function on purpose: the Arduino .ino preprocessor auto-generates
// prototypes before the first function, so types used in function signatures must already exist.
struct Voice {
    bool  active = false, choke = false;
    float phase = 0, phase2 = 0;
    float env = 0, env2 = 0, penv = 0;   // env: main amp, env2: click / osc amp
    float vel = 0;
    float ic1 = 0, ic2 = 0;              // SVF state
    uint32_t rng = 1;
};

struct MP {                              // per-block mapped parameters
    float tune, bend, tone, level;
    float coef, coef2, pcoef, chokeCoef;
    float gHp;                           // snare noise highpass
    float k, sqrtK;                      // hat resonance
};

struct SavedState {                      // what goes into flash
    uint32_t magic;
    uint32_t version;
    uint32_t curVoice;
    float    p[NV][NP];
};
static constexpr uint32_t STATE_MAGIC   = 0x44525653;   // 'DRVS'
static constexpr uint32_t STATE_VERSION = 1;

struct Encoder {
    int      lastA = HIGH;
    uint32_t lastTick = 0;
};
struct Button {
    uint8_t  pin = 0;
    bool     down = false, rawPrev = false;
    uint32_t changed = 0;
};
// ---------- Small DSP helpers ----------
#line 126 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static float clamp01(float x);
#line 127 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static float expMap(float n, float lo, float hi);
#line 128 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static float decCoef(float t60, float sr);
#line 129 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static float tcCoef(float tau, float sr);
#line 132 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static float fastSin(float p);
#line 141 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static float fastTanh(float x);
#line 152 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static float noiseGen(Voice &v);
#line 161 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static void svf(Voice &v, float in, float g, float k, float &bp, float &hp);
#line 174 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static void voiceTrigger(int i, float vel);
#line 186 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static void computeParams();
#line 212 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static void renderVoice(int i, float *buf);
#line 270 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
void audioTask(void *parameter);
#line 304 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static void handleMidiMessage(uint8_t status, uint8_t note, uint8_t velocity);
#line 323 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static void readMidi();
#line 360 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static int encoderPoll(int idx);
#line 379 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static bool buttonUpdate(Button &b, bool &pressed);
#line 405 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static void markDirty();
#line 411 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static void saveState(bool verbose);
#line 445 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static bool loadState();
#line 467 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static void clearSavedState();
#line 477 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static void printParam(int v, int k);
#line 490 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static void adjust(int v, int k, int steps);
#line 498 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
void setup();
#line 592 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
void loop();
#line 126 "/home/forrest/programming_2026/drum_synth_seq/esp32_vadrum/esp32_vadrum.ino"
static inline float clamp01(float x) { return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }
static inline float expMap(float n, float lo, float hi) { return lo * powf(hi / lo, n); }
static inline float decCoef(float t60, float sr) { return expf(-6.9078f / (t60 * sr)); }
static inline float tcCoef(float tau, float sr)  { return expf(-1.f / (tau * sr)); }

// sin(2*pi*p) for p in [0,1): parabola + refinement, ~0.1% error, much cheaper than sinf
static inline float fastSin(float p)
{
    float x = 2.f * p - 1.f;
    float y = 4.f * x * (1.f - fabsf(x));
    y += 0.225f * (y * fabsf(y) - y);
    return -y;
}

// Pade tanh approximation, exact +-1 at |x| >= 3
static inline float fastTanh(float x)
{
    if (x > 3.f) return 1.f;
    if (x < -3.f) return -1.f;
    float x2 = x * x;
    return x * (27.f + x2) / (27.f + 9.f * x2);
}

static Voice vo[NV];
static MP    mp[NV];

static inline float noiseGen(Voice &v)
{
    v.rng ^= v.rng << 13;
    v.rng ^= v.rng >> 17;
    v.rng ^= v.rng << 5;
    return (float)(int32_t)v.rng * (1.0f / 2147483648.0f);
}

// TPT state-variable filter (stable at any cutoff)
static inline void svf(Voice &v, float in, float g, float k, float &bp, float &hp)
{
    float a1 = 1.f / (1.f + g * (g + k));
    float a2 = g * a1, a3 = g * a2;
    float v3 = in - v.ic2;
    float v1 = a1 * v.ic1 + a2 * v3;
    float v2 = v.ic2 + a2 * v.ic1 + a3 * v3;
    v.ic1 = 2.f * v1 - v.ic1;
    v.ic2 = 2.f * v2 - v.ic2;
    bp = v1;
    hp = in - k * v1 - v2;
}

static void voiceTrigger(int i, float vel)
{
    Voice &v = vo[i];
    v.active = true;
    v.choke  = false;
    v.env = v.env2 = v.penv = 1.f;
    v.phase = v.phase2 = 0.f;
    v.vel = vel;
    if (i == V_CH && vo[V_OH].active)    // closed hat chokes open hat
        vo[V_OH].choke = true;
}

static void computeParams()
{
    const float sr = (float)SAMPLE_RATE;
    for (int i = 0; i < NV; i++) {
        const VoiceDef &d = defs[i];
        MP &m = mp[i];
        float dec = expMap(clamp01(P[i][P_DECAY]), d.decLo, d.decHi);
        m.tune  = expMap(clamp01(P[i][P_TUNE]), d.tuneLo, d.tuneHi);
        m.bend  = clamp01(P[i][P_BEND]) * d.bendMax;
        m.tone  = clamp01(P[i][P_TONE]);
        m.level = clamp01(P[i][P_LEVEL]);
        m.level *= m.level;
        m.coef  = decCoef(dec, sr);
        m.pcoef = tcCoef(d.ptau, sr);
        m.chokeCoef = decCoef(0.010f, sr);
        switch (d.type) {
        case T_BD: m.coef2 = tcCoef(0.002f, sr); break;          // click
        case T_SD: m.coef2 = decCoef(dec * 0.45f, sr); break;    // osc
        default:   m.coef2 = m.coef; break;
        }
        m.gHp   = tanf(PI_F * 1800.f / sr);
        m.k     = 1.f / (0.7f + m.tone * 20.f);
        m.sqrtK = sqrtf(m.k);
    }
}

static void renderVoice(int i, float *buf)
{
    Voice &v = vo[i];
    if (!v.active) return;
    const MP &m = mp[i];
    const int type = defs[i].type;
    const float inv = 1.0f / (float)SAMPLE_RATE;

    // hats: filter coefficient updated once per block (pitch env is slow vs 2.9 ms)
    float gHat = 0.f;
    if (type == T_HH) {
        float cf = fminf(m.tune * (1.f + m.bend * v.penv), 0.45f * SAMPLE_RATE);
        gHat = tanf(PI_F * cf * inv);
    }

    for (int n = 0; n < BLOCK_FRAMES; n++) {
        float s;
        float f = m.tune * (1.f + m.bend * v.penv);

        if (type == T_BD) {
            v.phase += f * inv;
            if (v.phase >= 1.f) v.phase -= 1.f;
            float click = noiseGen(v) * v.env2 * m.tone;
            s = fastTanh((fastSin(v.phase) + click * 1.5f) * (1.f + m.tone * 2.f)) * v.env;
        } else if (type == T_SD) {
            v.phase  += f * inv;           if (v.phase  >= 1.f) v.phase  -= 1.f;
            v.phase2 += f * 1.62f * inv;   if (v.phase2 >= 1.f) v.phase2 -= 1.f;
            float osc = 0.65f * fastSin(v.phase) + 0.35f * fastSin(v.phase2);
            float bp, hp;
            svf(v, noiseGen(v), m.gHp, 1.4f, bp, hp);
            float go = fminf(1.f, 2.f * (1.f - m.tone));
            float gn = fminf(1.f, 2.f * m.tone);
            s = go * osc * v.env2 + gn * hp * 1.2f * v.env;
        } else {   // T_HH: tuned noise
            float bp, hp;
            svf(v, noiseGen(v), gHat, m.k, bp, hp);
            s = bp * m.sqrtK * 2.f * v.env;
        }

        if (!(s > -8.f && s < 8.f)) {        // NaN / runaway guard: reset the filter and mute this sample
            s = 0.f;
            v.ic1 = v.ic2 = 0.f;
        }

        buf[n] += s * v.vel * m.level;

        float c1 = v.choke ? m.chokeCoef : m.coef;
        float c2 = v.choke ? m.chokeCoef : m.coef2;
        v.env  *= c1;
        v.env2 *= c2;
        v.penv *= m.pcoef;
    }
    if (v.env < 1e-4f && v.env2 < 1e-4f) v.active = false;
}

// ---------- Audio ----------
static i2s_chan_handle_t i2sTx = nullptr;

void audioTask(void *parameter)
{
    (void)parameter;
    int16_t out[BLOCK_FRAMES * 2];
    float mix[BLOCK_FRAMES];

    while (true) {
        for (int i = 0; i < NV; i++) {
            uint8_t t = trigVel[i].exchange(0);
            if (t) voiceTrigger(i, t / 127.f);
        }
        computeParams();

        memset(mix, 0, sizeof mix);
        for (int i = 0; i < NV; i++) renderVoice(i, mix);

        for (int n = 0; n < BLOCK_FRAMES; n++) {
            float s = fastTanh(mix[n]) * MASTER_GAIN;
            int16_t q = (int16_t)(s * 32767.0f);
            out[n * 2]     = q;
            out[n * 2 + 1] = q;
        }

        size_t written = 0;
        i2s_channel_write(i2sTx, out, sizeof(out), &written, portMAX_DELAY);
    }
}

// ---------- MIDI ----------
static uint8_t midiStatus = 0;
static uint8_t midiData[2];
static uint8_t midiDataCount = 0;
static uint8_t midiExpected = 0;

static void handleMidiMessage(uint8_t status, uint8_t note, uint8_t velocity)
{
    if ((status & 0xF0) == 0x90 && velocity > 0) {
        int hitVoice = -1;
        for (int i = 0; i < NV; i++) {
            if (note == midiNote[i]) {
                trigVel[i].store(velocity);
                hitVoice = i;
            }
        }
        if (hitVoice >= 0)
            Serial.printf("NOTE ON  ch=%u note=%u vel=%u -> %s\n",
                          (status & 0x0F) + 1, note, velocity, defs[hitVoice].name);
        else
            Serial.printf("NOTE ON  ch=%u note=%u vel=%u -> (not mapped)\n",
                          (status & 0x0F) + 1, note, velocity);
    }
}

static void readMidi()
{
    while (MIDI.available()) {
        uint8_t b = MIDI.read();
        if (b >= 0xF8) continue;            // ignore real-time

        if (b & 0x80) {
            midiStatus = b;
            midiDataCount = 0;
            uint8_t command = midiStatus & 0xF0;
            if (command == 0x80 || command == 0x90 || command == 0xA0 ||
                command == 0xB0 || command == 0xE0)      midiExpected = 2;
            else if (command == 0xC0 || command == 0xD0) midiExpected = 1;
            else                                         midiExpected = 0;
            continue;
        }

        if (midiExpected == 0) continue;
        midiData[midiDataCount++] = b;

        if (midiDataCount >= midiExpected) {
            uint8_t command = midiStatus & 0xF0;
            if (command == 0x80 || command == 0x90)
                handleMidiMessage(midiStatus, midiData[0], midiData[1]);
            midiDataCount = 0;              // keep running status
        }
    }
}

// ---------- Encoders and buttons ----------
static Encoder enc[4];

static Button encBtn[4];
static Button instBtn;

// Same decoding as the tested version: act on A falling, B level gives direction.
// Fast turns are accelerated.
static int encoderPoll(int idx)
{
    Encoder &e = enc[idx];
    int a = digitalRead(encPins[idx].a);
    if (a == e.lastA) return 0;
    e.lastA = a;
    if (a != LOW) return 0;

    int dir = (digitalRead(encPins[idx].b) != a) ? 1 : -1;
    dir *= ENC_DIR;

    uint32_t now = millis();
    uint32_t dt = now - e.lastTick;
    e.lastTick = now;
    int mult = (dt < 15) ? 4 : (dt < 40 ? 2 : 1);
    return dir * mult;
}

// returns true when the debounced state changes; pressed tells which way
static bool buttonUpdate(Button &b, bool &pressed)
{
    bool raw = (digitalRead(b.pin) == LOW);
    uint32_t now = millis();
    if (raw != b.rawPrev) { b.rawPrev = raw; b.changed = now; }
    if (raw != b.down && (now - b.changed) >= DEBOUNCE_MS) {
        b.down = raw;
        pressed = raw;
        return true;
    }
    return false;
}

// ---------- UI logic ----------
static int curVoice = V_BD;     // voice edited in INST mode
static uint32_t instHeldSince = 0;
static bool instUsed = false, longDone = false;
static int dirtyV = -1, dirtyK = -1;
static uint32_t lastPrint = 0;

// ---------- Saved state ----------
static bool        stateDirty = false;
static uint32_t    lastChangeMs = 0;
static SavedState  lastSaved;
static bool        haveSaved = false;

static void markDirty()
{
    stateDirty = true;
    lastChangeMs = millis();
}

static void saveState(bool verbose)
{
    stateDirty = false;

    SavedState s;
    memset(&s, 0, sizeof s);              // zero padding so memcmp is reliable
    s.magic = STATE_MAGIC;
    s.version = STATE_VERSION;
    s.curVoice = (uint32_t)curVoice;
    for (int v = 0; v < NV; v++)
        for (int k = 0; k < NP; k++) s.p[v][k] = P[v][k];

    if (haveSaved && memcmp(&s, &lastSaved, sizeof s) == 0) {
        if (verbose) Serial.println("State already saved");
        return;
    }

    Preferences prefs;
    if (!prefs.begin("drumvoice", false)) {
        Serial.println("Save failed: cannot open NVS");
        return;
    }
    size_t n = prefs.putBytes("state", &s, sizeof s);
    prefs.end();

    if (n == sizeof s) {
        lastSaved = s;
        haveSaved = true;
        Serial.println("State saved");
    } else {
        Serial.println("Save failed: NVS write error");
    }
}

static bool loadState()
{
    Preferences prefs;
    if (!prefs.begin("drumvoice", true)) return false;   // read-only; fails if nothing saved yet
    SavedState s;
    size_t n = prefs.getBytes("state", &s, sizeof s);
    prefs.end();

    if (n != sizeof s || s.magic != STATE_MAGIC || s.version != STATE_VERSION || s.curVoice >= NV)
        return false;
    for (int v = 0; v < NV; v++)
        for (int k = 0; k < NP; k++)
            if (!(s.p[v][k] >= 0.f && s.p[v][k] <= 1.f)) return false;   // also rejects NaN

    for (int v = 0; v < NV; v++)
        for (int k = 0; k < NP; k++) P[v][k] = s.p[v][k];
    curVoice = (int)s.curVoice;
    lastSaved = s;
    haveSaved = true;
    return true;
}

static void clearSavedState()
{
    Preferences prefs;
    if (prefs.begin("drumvoice", false)) {
        prefs.clear();
        prefs.end();
    }
    haveSaved = false;
}

static void printParam(int v, int k)
{
    const VoiceDef &d = defs[v];
    float n = clamp01(P[v][k]);
    const char *name = (k == P_TONE) ? d.toneName : (k == P_LEVEL ? "LEVEL" : paramName[k]);
    switch (k) {
    case P_TUNE:  Serial.printf("%s %s %.0f Hz\n", d.name, name, expMap(n, d.tuneLo, d.tuneHi)); break;
    case P_DECAY: Serial.printf("%s %s %.0f ms\n", d.name, name, 1000.f * expMap(n, d.decLo, d.decHi)); break;
    case P_BEND:  Serial.printf("%s %s x%.1f\n",   d.name, name, 1.f + n * d.bendMax); break;
    default:      Serial.printf("%s %s %.0f%%\n",  d.name, name, n * 100.f); break;
    }
}

static void adjust(int v, int k, int steps)
{
    P[v][k] = clamp01(P[v][k] + steps * ENC_STEP);
    dirtyV = v; dirtyK = k;
    markDirty();
}

// ---------- Setup / loop ----------
void setup()
{
    Serial.begin(115200);
    delay(500);

    for (int v = 0; v < NV; v++) {
        for (int k = 0; k < NP; k++) P[v][k] = defs[v].def[k];
        vo[v].rng = 0x12345678u + (uint32_t)v * 7919u;
        trigVel[v].store(0);
    }

    // INST held at power-up = factory reset, otherwise restore the saved state
    pinMode(INST_SW, INPUT_PULLUP);
    delay(10);
    if (digitalRead(INST_SW) == LOW) {
        clearSavedState();
        Serial.println("INST held at boot: saved state erased, using defaults");
    } else if (loadState()) {
        Serial.printf("Saved state restored (selected voice: %s)\n", defs[curVoice].name);
    } else {
        Serial.println("No valid saved state, using defaults");
    }

    MIDI.begin(31250, SERIAL_8N1, MIDI_RX_PIN, -1);
    Serial.println("DIN MIDI input started on GPIO16");

    // I2S TX
    i2s_chan_config_t chanConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chanConfig.dma_desc_num  = 4;              // 4 x 128 frames ~ 11.6 ms; raise to 6 if you hear dropouts
    chanConfig.dma_frame_num = BLOCK_FRAMES;
    chanConfig.auto_clear    = true;

    esp_err_t err = i2s_new_channel(&chanConfig, &i2sTx, nullptr);
    if (err != ESP_OK) {
        Serial.printf("I2S channel creation failed: %d\n", err);
        while (true) delay(1000);
    }

    i2s_std_config_t stdConfig = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO),
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

    err = i2s_channel_init_std_mode(i2sTx, &stdConfig);
    if (err != ESP_OK) {
        Serial.printf("I2S initialization failed: %d\n", err);
        while (true) delay(1000);
    }
    err = i2s_channel_enable(i2sTx);
    if (err != ESP_OK) {
        Serial.printf("I2S enable failed: %d\n", err);
        while (true) delay(1000);
    }

    // Encoders and buttons
    for (int i = 0; i < 4; i++) {
        if (i == 3) {                       // GPIO34/35: input-only, external pull-ups
            pinMode(encPins[i].a, INPUT);
            pinMode(encPins[i].b, INPUT);
        } else {
            pinMode(encPins[i].a, INPUT_PULLUP);
            pinMode(encPins[i].b, INPUT_PULLUP);
        }
        pinMode(encPins[i].sw, INPUT_PULLUP);
        enc[i].lastA = digitalRead(encPins[i].a);
        encBtn[i].pin = encPins[i].sw;
    }
    pinMode(INST_SW, INPUT_PULLUP);
    instBtn.pin = INST_SW;

    // Audio on core 0, everything else (MIDI, UI) stays on the Arduino loop task on core 1
    BaseType_t ok = xTaskCreatePinnedToCore(audioTask, "AudioTask", 8192, nullptr, 3, nullptr, 0);
    if (ok != pdPASS) {
        Serial.println("Failed to create audio task");
        while (true) delay(1000);
    }

    Serial.printf("Drum voice ready. Selected voice: %s\n", defs[curVoice].name);
}

void loop()
{
    readMidi();

    bool pressed;
    if (buttonUpdate(instBtn, pressed)) {
        if (pressed) {
            instHeldSince = millis();
            instUsed = false;
            longDone = false;
            Serial.println("INST held - editing voice params");
        } else {
            Serial.println("INST released - volume mode");
        }
    }
    bool shift = instBtn.down;

    for (int i = 0; i < 4; i++) {
        // rotation
        int steps = encoderPoll(i);
        if (steps) {
            if (shift) instUsed = true;
            if (shift) adjust(curVoice, i, steps);   // enc 1..4 = TUNE, DECAY, BEND, TONE
            else       adjust(i, P_LEVEL, steps);    // enc i = volume of voice i
        }

        // click
        if (buttonUpdate(encBtn[i], pressed) && pressed) {
            if (shift) {
                instUsed = true;
                curVoice = i;
                markDirty();
                Serial.printf("Selected voice: %s\n", defs[i].name);
            } else {
                trigVel[i].store(100);               // preview
            }
        }
    }

    // INST held ~2 s with nothing touched = save now
    if (shift && !instUsed && !longDone && millis() - instHeldSince >= LONG_PRESS_MS) {
        longDone = true;
        saveState(true);
    }

    // debounced auto-save
    if (AUTO_SAVE && stateDirty && millis() - lastChangeMs >= SAVE_DELAY_MS)
        saveState(false);

    // print the last-touched parameter at most every 40 ms so fast turns don't flood the UART
    if (dirtyV >= 0 && millis() - lastPrint >= 40) {
        printParam(dirtyV, dirtyK);
        dirtyV = dirtyK = -1;
        lastPrint = millis();
    }

    delay(1);
}

