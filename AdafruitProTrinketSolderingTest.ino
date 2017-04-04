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
  
*/

// We check for IO on pins 0 to 19, skipping pins 2 and 7
int PINS[] = [0, 1, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19]

void setUp() {
  Serial.begin(9600);      // open the serial port at 9600 bps:    
  setAllInput();
  for (int i = 0; i < PINS.length; i++) {
    validatePinValue(PINS[i], LOW);
  }
  Serial.println('Validated all LOW');
}

void loop() {
  for (int i = 0; i < PINS.length; i++) {
    setAllInput();
    pinMode(PINS[i], OUTPUT);
    digitalWrite(PINS[i], HIGH);
    for (int j = 0; j < PINS.length; j++) {
      switch (j) {
        case i:
          break;
        case PINS.length - i:
          validatePinValue(PINS[j], HIGH);
          break;
        default:
          validatePinValue(PINS[j], LOW);
          break;
      }
    }
    Serial.print('Validated pin ');
    Serial.println(i);
  }
}

void setAllInput() {
  for (int i = 0; i < PINS.length; i++) {
    pinMode(PINS[i], INPUT);
  }
}

void validatePinValue(int pinNumber, int value) {
  if (digitalRead(pinNumber) != value) {
    Serial.print('ALERT*** Digital Pin ');
    Serial.print(pinNumber);
    Serial.print(' is not set to ');
    Serial.print(value);
    Serial.println('!!');
  }
}
