// Arduino Mega 2560 MIDI Out via Serial1 (TX1 = Pin 18)
const int stepPinCount = 16;
const int stepPins[stepPinCount] = {22,23,24,25,26,27,28,29,
				    30,31,32,33,34,35,36,37};
int stepPinStates[stepPinCount] = {0,0,0,0,0,0,0,0,
				   0,0,0,0,0,0,0,0};
const int ledPinCount = 16;
const int ledPins[ledPinCount] = {38,39,40,41,42,43,44,45,
				  46,47,48,49,51,50,52,53};

const int shiftPin1 = 7;
const int shiftPin2 = 6;
int shiftPin1State = 0;
int shiftPin2State = 0;

void setup() {
  // Set the MIDI standard baud rate
  Serial1.begin(31250);
  
  Serial.begin(115200);
  Serial.println("it works");

  for (int sp = 0; sp < stepPinCount; sp++) {
    pinMode(stepPins[sp], INPUT_PULLUP);
  }

  for (int lp = 0; lp < ledPinCount; lp++) {
    pinMode(ledPins[lp], OUTPUT);
    digitalWrite(ledPins[lp], LOW);
  }

  pinMode(shiftPin1, INPUT_PULLUP);
  pinMode(shiftPin2, INPUT_PULLUP);  
}

void loop() {
  // Play Note On (Middle C, Note number 60, Velocity 100)
  //  noteOn(0x90, 60, 100);
  //  delay(1000);

  // Play Note Off (Middle C, Note number 60, Velocity 0)
  //  noteOff(0x80, 60, 0);
  //  delay(1000);
  int pinState;
  char str[255];
  
  for (int sp=0; sp<stepPinCount; sp++) {

    pinState = digitalRead(stepPins[sp]);
        
    if (pinState != stepPinStates[sp]) {
      stepPinStates[sp] = pinState;
      sprintf(str, "stepPin state: %d = %d\n", sp, pinState);
      Serial.print(str);
      if (sp < ledPinCount) {
	digitalWrite(ledPins[sp], pinState);
      }
    }
  }

  pinState = digitalRead(shiftPin1);
  if (pinState != shiftPin1State) {
    shiftPin1State = pinState;
    sprintf(str, "shiftPin1 state: %d\n", pinState);
    Serial.print(str);
  }

  pinState = digitalRead(shiftPin2);
  if (pinState != shiftPin2State) {
    shiftPin2State = pinState;
    sprintf(str, "shiftPin2 state: %d\n", pinState);
    Serial.print(str);
  }

  int val = analogRead(A0);
  sprintf(str, "analogRead A0: %d\n", val);
  Serial.print(str);
  val = analogRead(A1);
  sprintf(str, "analogRead A1: %d\n", val);
  Serial.print(str);
  
  delay(100);
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
