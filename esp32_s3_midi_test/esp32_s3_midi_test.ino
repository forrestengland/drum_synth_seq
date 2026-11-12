#include <Arduino.h>
#include <math.h>

// pin midi in
#define MIDI_RX_PIN 16
#define MIDI_TX_PIN 41

void setup() {

  Serial.begin(115200);

  Serial.println("ESP32-S3 MIDI test");

	Serial2.begin(31250, SERIAL_8N1, MIDI_RX_PIN, MIDI_TX_PIN);

	Serial.println("DIN MIDI input ready");
}

void loop() {

	while (Serial2.available()) {

		uint8_t data = Serial2.read();

		Serial.print(data);
		Serial.print(" ");

		Serial2.write(data);
	}
}
