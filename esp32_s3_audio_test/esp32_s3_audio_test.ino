#include <Arduino.h>
#include <math.h>

#include "driver/i2s_std.h"

// -------- I2S wiring --------
constexpr int I2S_BCLK = 5;
constexpr int I2S_LRCK = 6;
constexpr int I2S_DOUT = 7;

float targetPhaseIncrement = 0.0f;

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

// Protect shared audio parameters between tasks
portMUX_TYPE synthMux = portMUX_INITIALIZER_UNLOCKED;

int activeNote = -1;
float targetAmplitude = 0.0f;
float currentAmplitude = 0.0f;
float phase = 0.0f;
float phaseIncrement = 0.0f;

int manualNote = -1;
int manualVelocity = 0;

// -------- I2S driver --------
i2s_chan_handle_t txHandle = nullptr;

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

void updateSynthVoice() {

	// Manual keyboard notes take priority.
	int note = 60;
	int velocity = 100;

	float frequency = 440.0f * powf(2.0f, (note - 69) / 12.0f);
	float increment = (frequency * WAVETABLE_SIZE) / SAMPLE_RATE;
	float amplitude = MAX_AMPLITUDE * (velocity / 127.0f);

	portENTER_CRITICAL(&synthMux);

	activeNote = note;
	targetPhaseIncrement = increment;
	targetAmplitude = amplitude;

	portEXIT_CRITICAL(&synthMux);
}

void audioTask(void* parameter) {
	
  int16_t buffer[BLOCK_FRAMES * 2];

	for (;;) {

    float blockTargetAmplitude;
    float blockPhaseIncrement;

    portENTER_CRITICAL(&synthMux);

    blockTargetAmplitude = targetAmplitude;
		blockPhaseIncrement = targetPhaseIncrement;

    portEXIT_CRITICAL(&synthMux);

    for (int i = 0; i < BLOCK_FRAMES; i++) {

			phaseIncrement = blockPhaseIncrement;
			currentAmplitude = blockTargetAmplitude;

      float sample = 0.0f;

			float oscillatorSample = getOscillatorSample();

			sample = oscillatorSample * currentAmplitude;

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

void setup() {

  Serial.begin(115200);

  Serial.println();
  Serial.println("ESP32-S3 i2s audio test");

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

	updateSynthVoice();
}

void loop() {

  delay(2000);

  portENTER_CRITICAL(&synthMux);
  currentWaveform = (currentWaveform == WAVE_SAW) ? WAVE_SQUARE : WAVE_SAW;
  portEXIT_CRITICAL(&synthMux);

  Serial.println(currentWaveform == WAVE_SAW ? "SAW" : "SQUARE");
}
