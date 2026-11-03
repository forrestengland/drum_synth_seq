
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


// Sync mode switch
#define SYNC_SWITCH_PIN 8

enum SyncMode {
    SYNC_EXTERNAL,
    SYNC_INTERNAL
};

SyncMode syncMode = SYNC_EXTERNAL;

// Internal tempo
float internalBpm = 120.0f;
constexpr float MIN_BPM = 40.0f;
constexpr float MAX_BPM = 240.0f;

esp_timer_handle_t internalClockTimer = nullptr;

// encoder button debounce
#define ENCODER_DEBOUNCE_MS 50

bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
uint32_t lastButtonChange = 0;

bool lastButton2Reading = HIGH;
bool stableButton2State = HIGH;
uint32_t lastButton2Change = 0;

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

/*int lastEncoderCLK = HIGH;
int encoderPosition = 0;

int lastEncoder2CLK = HIGH;
int encoder2Position = 0; */

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

Step pattern[NUM_STEPS];

uint8_t selectedStep = 0;

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

// Returns detents since last call and clears the counter.
int32_t takeEncoderDetents(Encoder& e) {
    portENTER_CRITICAL(&synthMux);
    int32_t d = e.detents;
    e.detents = 0;
    portEXIT_CRITICAL(&synthMux);
    return d;
}

void updateEncoder() {
    updateEncoderButton();

    // Encoder 1: select step
    int32_t d1 = takeEncoderDetents(enc1);
    if (d1 != 0) {
        selectedStep = (selectedStep + NUM_STEPS + (d1 % NUM_STEPS)) % NUM_STEPS;
        displayDirty = true;
    }

    // Encoder 2: note, or tempo while its button is held
    int32_t d2 = takeEncoderDetents(enc2);
    if (d2 != 0) {
        bool buttonHeld = (stableButton2State == LOW);

        if (buttonHeld) {
            // Tempo only applies to the internal clock.
            if (syncMode == SYNC_INTERNAL) {
                internalBpm = constrain(internalBpm + d2, MIN_BPM, MAX_BPM);
                restartInternalClock();
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

    enc1.state = (digitalRead(ENCODER_CLK) << 1) | digitalRead(ENCODER_DT);
    enc2.state = (digitalRead(ENCODER2_CLK) << 1) | digitalRead(ENCODER2_DT);

    attachInterruptArg(digitalPinToInterrupt(ENCODER_CLK),  encoderISR, &enc1, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(ENCODER_DT),   encoderISR, &enc1, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(ENCODER2_CLK), encoderISR, &enc2, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(ENCODER2_DT),  encoderISR, &enc2, CHANGE);
}

void internalClockCallback(void* arg) {

    if (syncMode != SYNC_INTERNAL) {
        return;
    }

    currentStep = (currentStep + 1) % NUM_STEPS;

    triggerSequencerStep();

    displayDirty = true;
}

void restartInternalClock() {

    if (esp_timer_is_active(internalClockTimer)) {
        esp_timer_stop(internalClockTimer);
    }

    uint64_t interval =
        (uint64_t)(60000000.0f / (internalBpm * 4.0f));

    esp_timer_start_periodic(
        internalClockTimer,
        interval
    );
}



void updateEncoderButton() {

	bool reading = digitalRead(ENCODER_SW);

    if (reading != lastButtonReading) {
        lastButtonChange = millis();
        lastButtonReading = reading;
    }

    if ((millis() - lastButtonChange) >= ENCODER_DEBOUNCE_MS) {
        if (reading != stableButtonState) {
            stableButtonState = reading;

            // Trigger only once when button is pressed
            if (stableButtonState == LOW) {
                pattern[selectedStep].active =
                    !pattern[selectedStep].active;

                displayDirty = true;

								/*                Serial.printf("Step %d active: %d\n",
                              selectedStep + 1,
                              pattern[selectedStep].active); */
            }
        }
    }

	reading = digitalRead(ENCODER2_SW);

    if (reading != lastButton2Reading) {
        lastButton2Change = millis();
        lastButton2Reading = reading;
    }

    if ((millis() - lastButton2Change) >= ENCODER_DEBOUNCE_MS) {
        if (reading != stableButton2State) {
            stableButton2State = reading;

            // Trigger only once when button is pressed
            if (stableButton2State == LOW) {
							//                Serial.printf("encoder 2 pressed\n");
            }
        }
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

void updateSynthVoice() {
    // Manual keyboard notes take priority.
    int note = manualNote >= 0 ? manualNote : sequencerNote;
    int velocity = manualNote >= 0
        ? manualVelocity
        : sequencerVelocity;

    if (note < 0) {
        portENTER_CRITICAL(&synthMux);
        activeNote = -1;
        targetAmplitude = 0.0f;
        portEXIT_CRITICAL(&synthMux);
        return;
    }

    float frequency = 440.0f *
        powf(2.0f, (note - 69) / 12.0f);

    float increment =
        (frequency * WAVETABLE_SIZE) / SAMPLE_RATE;

    float amplitude =
        MAX_AMPLITUDE * (velocity / 127.0f);

    portENTER_CRITICAL(&synthMux);

    // Only retrigger the filter envelope if the pitch changes.
    if (activeNote != note) {
        filterEnvelopeTrigger = true;
    }

    activeNote = note;
    phaseIncrement = increment;
    targetAmplitude = amplitude;

    portEXIT_CRITICAL(&synthMux);

		//    Serial.printf("SYNTH NOTE: %d velocity: %d\n",
		//                  note, velocity);
}

void setNoteOn(int note, int velocity) {
    if (velocity <= 0) {
        setNoteOff(note);
        return;
    }

    manualNote = note;
    manualVelocity = velocity;

    updateSynthVoice();
}

void setNoteOff(int note) {
    if (manualNote == note) {
        manualNote = -1;
        manualVelocity = 0;

        updateSynthVoice();
    }
}

void triggerSequencerStep() {
    // Release the previous sequencer note.
    sequencerNote = -1;
    sequencerVelocity = 0;

    Step& step = pattern[currentStep];

    if (step.active) {
        sequencerNote = step.note;

        // Accent steps use maximum velocity.
        sequencerVelocity = step.accent ? 127 : 100;

				/*        Serial.printf(
            "SEQUENCER STEP %d: NOTE ON %d velocity %d\n",
            currentStep + 1,
            sequencerNote,
            sequencerVelocity */
				//        );
    } else {
	/*        Serial.printf(
            "SEQUENCER STEP %d: REST\n",
            currentStep + 1
						); */
    }

    updateSynthVoice();
}

void stopSequencerNote() {
    sequencerNote = -1;
    sequencerVelocity = 0;

    updateSynthVoice();
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

	for (;;) {
    float blockTargetAmplitude;
    float blockCutoff;
    float blockResonance;
    float blockPhaseIncrement;
		float blockDecay;
		float blockEnvDepth;
		bool triggerEnvelope;

    portENTER_CRITICAL(&synthMux);

    blockTargetAmplitude = targetAmplitude;
		//    blockPhaseIncrement = phaseIncrement;
    blockCutoff = filterCutoff;
    blockResonance = filterResonance;
		blockEnvDepth = filterEnvDepth;
		blockDecay = envelopeDecay;

		// consume the trigger
		triggerEnvelope = filterEnvelopeTrigger;
		filterEnvelopeTrigger = false;

    portEXIT_CRITICAL(&synthMux);

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
                clockCount++;

                // Six MIDI clock pulses = one sixteenth note.
                if (clockCount >= 6) {
                    clockCount = 0;
                    currentStep = (currentStep + 1) % NUM_STEPS;
                    triggerSequencerStep();
                    displayDirty = true;
                }
            }
            break;
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

    display.display();
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
        triggerSequencerStep();

        restartInternalClock();

        Serial.println("SYNC: INTERNAL");

    } else {

        Serial.println("SYNC: EXTERNAL");
    }

    displayDirty = true;
}

void setup() {

	pinMode(SYNC_SWITCH_PIN, INPUT_PULLUP);

  Serial.begin(115200);
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

 
	/*	display.clearDisplay();
	display.setTextColor(SSD1306_WHITE);
	display.setTextSize(1);
	display.setCursor(0, 0);
	display.println("TB-303");
	display.println("OLED ready");
	display.display();  */

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
}
