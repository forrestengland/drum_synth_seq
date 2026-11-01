
#include <Arduino.h>
#include <math.h>

#include <ESP32_Host_MIDI.h>
#include <USBConnection.h>

#include "driver/i2s_std.h"

// pots
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

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

// waveform switch
#define WAVE_SWITCH_PIN 4

// -------- Synth controls --------
float filterCutoff = 1000.0f;
float filterResonance = 0.0f;
float filterEnvDepth = 0.0f;
float envelopeDecay = 500.0f;

// -------- Resonant low-pass filter --------
float filterStage[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// -------- I2S driver --------
i2s_chan_handle_t txHandle = nullptr;

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

  Serial.println("I2S audio started.");
}

void setNoteOff(int note);

void setNoteOn(int note, int velocity) {

  if (velocity <= 0) {
    setNoteOff(note);
    return;
  }

	float frequency = 440.0f *
    powf(2.0f, (note - 69) / 12.0f);

	float increment =
    (frequency * WAVETABLE_SIZE) / SAMPLE_RATE;
 
  float amplitude =
      MAX_AMPLITUDE * (velocity / 127.0f);

  portENTER_CRITICAL(&synthMux);
  activeNote = note;
  phaseIncrement = increment;
  targetAmplitude = amplitude;
  portEXIT_CRITICAL(&synthMux);

  Serial.printf(
      "NOTE ON: %d, frequency: %.2f Hz, velocity: %d\n",
      note, frequency, velocity
  );
}

void setNoteOff(int note) {
  bool released = false;

  portENTER_CRITICAL(&synthMux);

  if (activeNote == note) {
    targetAmplitude = 0.0f;
    activeNote = -1;
    released = true;
  }

  portEXIT_CRITICAL(&synthMux);

  if (released) {
    Serial.printf("NOTE OFF: %d\n", note);
  }
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
      Serial.printf("MIDI channel: %d\n", channel);
    }
  }
}

void audioTask(void* parameter) {
  int16_t buffer[BLOCK_FRAMES * 2];

  // Approximately 5 ms amplitude ramp.
  constexpr float RAMP_STEP =
      1.0f / (SAMPLE_RATE * 0.005f);

	for (;;) {
    float blockTargetAmplitude;
    float blockCutoff;
    float blockResonance;
    float blockPhaseIncrement;

    portENTER_CRITICAL(&synthMux);

    blockTargetAmplitude = targetAmplitude;
    blockPhaseIncrement = phaseIncrement;

    blockCutoff = filterCutoff;
    blockResonance = filterResonance;

    portEXIT_CRITICAL(&synthMux);

    // Calculate filter coefficient once per block.
    float cutoff = constrain(
        blockCutoff,
        20.0f,
        SAMPLE_RATE * 0.45f
    );

    float filterCoefficient =
        1.0f - expf(
            -TWO_PI * cutoff / SAMPLE_RATE
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
      Serial.printf("I2S write error: %d\n", result);
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

  Serial.println("Wavetables initialized.");
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

    Serial.println(
        currentWaveform == WAVE_SAW
        ? "Waveform: SAW"
        : "Waveform: SQUARE"
    );
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

  Serial.printf(
    "Cutoff: %.0f Hz | Resonance: %.2f | Env: %.0f Hz | Decay: %.0f ms\n",
    cutoff, resonance, envDepth, decay
  );
}

void setup() {

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

	if (!ads.begin(0x48)) {
		Serial.println("ADS1115 initialization failed!");

		while (true) {
			delay(1000);
		}
	}

	ads.setGain(GAIN_ONE);
	ads.setDataRate(RATE_ADS1115_860SPS);

	Serial.println("ADS1115 initialized.");
 
}

void loop() {
  processMidi();
  updateWaveformSwitch();
  updateSynthControls();
	printSynthControls();
}
