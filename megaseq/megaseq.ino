// Arduino Mega 2560 MIDI Out via Serial1 (TX1 = Pin 18)

void setup() {
  // Set the MIDI standard baud rate
  Serial1.begin(31250);
}

void loop() {
  // Play Note On (Middle C, Note number 60, Velocity 100)
  noteOn(0x90, 60, 100);
  delay(1000);

  // Play Note Off (Middle C, Note number 60, Velocity 0)
  noteOff(0x80, 60, 0);
  delay(1000);
}

// Send a MIDI Note On message
void noteOn(byte cmd, byte pitch, byte velocity) {
  Serial1.write(cmd);
  Serial1.write(pitch);
  Serial1.write(velocity);
}

// Send a MIDI Note Off message
void noteOff(byte cmd, byte pitch, byte velocity) {
  Serial1.write(cmd);
  Serial1.write(pitch);
  Serial1.write(velocity);
}
