/**
  Set up an Adafruit Pro Trinket with the following digital pins connected (pins 2 and 7 are used for bootloader).

  0 - 19
  1 - 18
  3 - 17
  4 - 16
  5 - 15
  6 - 14
  8 - 13
  9 - 12
  10- 11

  Here's a diagram of the pin locations

  https://learn.adafruit.com/introducing-pro-trinket/pinouts
  Now run this sketch and watch the serial port on 9600 bps. You'll get an alert for any pin not correctly soldered.

  In case your serial monitor doesn't work, when an error is detected with a pin, the LED on the trinket will
  blink N times, followed by 3 seconds of being off.
  N indicates the number of the pin plus 1 (so the 0th pin can be identified)

*/

// We check for IO on pins 0 to 19, skipping pins 2 and 7
int PINS[] = {0, 1, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
int length = sizeof(PINS) / sizeof(int);

int errorPin = -1;

void setup() {
  Serial.begin(9600);      // open the serial port at 9600 bps:
  setAllInput();
  for (int i = 0; i < length; i++) {
    validatePinValue(PINS[i], LOW);
  }
  Serial.println('Validated all LOW');
}

void loop() {
  errorPin = -1;
  for (int i = 0; i < length; i++) {
    setAllInput();
    pinMode(PINS[i], OUTPUT);
    digitalWrite(PINS[i], HIGH);
    for (int j = 0; j < length; j++) {
      if (j == 1) {
      } else if (j == length - 1) {
        validatePinValue(PINS[j], HIGH);
      } else {
        validatePinValue(PINS[j], LOW);
      }
    }
    Serial.print('Validated pin ');
    Serial.println(i);
    Serial.flush();
  }
  if (errorPin > -1) {
    blinkNTimes(errorPin);
  }
}

void setAllInput() {
  for (int i = 0; i < length; i++) {
    pinMode(PINS[i], INPUT);
  }
}

void validatePinValue(int pinNumber, int value) {
  if (digitalRead(pinNumber) != value) {
    errorPin = pinNumber;
    Serial.print('ALERT*** Digital Pin ');
    Serial.print(pinNumber);
    Serial.print(' is not set to ');
    Serial.print(value);
    Serial.println('!!');
  }
}

void blinkNTimes(int n) {
  for (int i = -1; i < n; i++) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(500);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(500);                       // wait for a second
  }
  delay(3000); // wait 3 seconds
}
