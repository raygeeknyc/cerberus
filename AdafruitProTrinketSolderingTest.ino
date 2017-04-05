/**
  Set up an Adafruit Pro Trinket 

  On a breadboard, set up a resistor and LED in series with the GND pin of your trinket.
  
  Run a jumper cable from the LED to one of the input pins.

  
  The LED should blink only once per cycle (each cycle is denoted by the onboard LED blinking)
  
  Move the jumper cable to each of the pins, verifying that each one blinks once.
  
  Input pins are   RX,TX,3,4,5,6,8,9,10,11,12,13,A0,A1,A2,A3,A4,A5
  
  Here's a diagram of the pin locations

  https://learn.adafruit.com/introducing-pro-trinket/pinouts
*/

// We check for IO on pins 0 to 19, skipping pins 2 and 7
int PINS[] = {0, 1, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
int length = sizeof(PINS) / sizeof(int);

int errorPin = -1;

void setup() {
  for (int i = 0; i < length; i++) {
    pinMode(PINS[i], OUTPUT);
    digitalWrite(PINS[i], LOW);
  }
}

void loop() {
  for (int i = 0; i < length; i++) {
    digitalWrite(PINS[i], HIGH);
    delay(200);
    digitalWrite(PINS[i], LOW);
  }
}
