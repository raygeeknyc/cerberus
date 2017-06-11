// Cerberus - a two headed robot dog

#include <Servo.h>
#include <NewPing.h>

#define MAX_PING_SENSOR_DISTANCE 60
#include <TimerFreeTone.h>

# Unused for now
#define _TRINKET
// define _DEBUG if you want LOTS of Serial output
// #define _DEBUG
 
// Define which pins each of our sensors and actuators are connected to
#define PIN_PING_TRIGGER_RIGHT 3
#define PIN_PING_ECHO_RIGHT 4
#define PIN_PING_TRIGGER_LEFT 12
#define PIN_PING_ECHO_LEFT 13
#define PIN_LED 6
#define PIN_SERVO_LEFT 9
#define PIN_SERVO_RIGHT 10
#define PIN_BUZZER 11
#define PIN_CDS A0

#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C5 523

bool wakeup_from_sensors;
unsigned long time_sleeping, next_ping_at, sleep_until, shine_until, next_breathe_at, last_sensor_activity_at, awake_since, weave_phase_at;

int current_distance_l, current_distance_r, shine_brightness, light_level, light_delta,
    prev_distance_l, prev_distance_r, ping_delta_l, ping_delta_r;
int weave_phase, weave_dir;
int weave_bias[] = { -1, 0, 1};
int weave_phase_duration_ms[] = {500, 1000, 500};
#define TURN_DUR_MS 400

#define PING_SAMPLES 5
#define PING_SAMPLE_DELAY_MS 50
#define POST_PING_DELAY_MS 2

#define MIN_TIME_AWAKE_SECS 5
const int MIN_TIME_AWAKE_MS = MIN_TIME_AWAKE_SECS * 1000;

#define CLOSE_PROXIMITY_THRESHOLD 10
#define SNORE_DURATION_SECS 3
const int SNORE_DURATION_MS = SNORE_DURATION_SECS * 1000;
#define BREATHE_MIN 0
#define BREATHE_MAX 110
#define BREATHE_STEP 10
#define BREATHE_STEP_DUR_MS 50
int breathe_dir;

// Imported from https://github.com/raygeeknyc/photovore/blob/master/photovore.ino
Servo left_motor;
Servo right_motor;

// Define these based on your servos and controller, the values to cause your servos
// to spin in opposite directions at approx the same speed.
#define CCW 30
#define CW 150

#define CCW_SLOW 80
#define CW_SLOW 100 

#define CW_STOP 90
#define CCW_STOP 90

#define SERVO_L_FWDSLOW CW_SLOW
#define SERVO_R_FWDSLOW CCW_SLOW

#define SERVO_L_FWD CW
#define SERVO_R_FWD CCW

#define SERVO_L_BWD CCW
#define SERVO_R_BWD CW

#define SERVO_L_STOP CCW_STOP
#define SERVO_R_STOP CW_STOP

#define SENSOR_DELTA_THRESHOLD_PCT 20
#define SENSOR_DELTA_THRESHOLD 30

#define HIGHEST_THRESHOLD SENSOR_DELTA_THRESHOLD

#define SENSOR_SAMPLES 5

#define MAX_SENSOR_READING 1023  // Used to seed sensor pair normalization
#define LIGHT_CHANGE_THRESHOLD 200
#define PING_CHANGE_THRESHOLD_CM 15

// How long to spin while callibrating the sensor pair
#define CALLIBRATION_DUR_MS 1000

// How long to pause between steps while spinning to normalize the sensor pair
#define SPIN_STEP_DELAY_MS 15
#define DIR_STOP 0
#define DIR_RIGHT 1
#define DIR_LEFT 2
#define DIR_FWD 3
#define LED_FLASH_DURATION_MS 500
#define LED_SHINE_DURATION_MS 1500
#define WITHDRAW_DUR_MS 700
#define INACTIVITY_TIME_TO_NAP_SECS 15
const int INACTIVITY_TIME_TO_NAP_MS = INACTIVITY_TIME_TO_NAP_SECS * 1000;
#define ACTIVITY_TIME_TO_NAP_SECS 30
const int ACTIVITY_TIME_TO_NAP_MS =  ACTIVITY_TIME_TO_NAP_SECS * 1000;
#define SLEEP_DURATION_SECS 15

NewPing sonarL(PIN_PING_TRIGGER_LEFT, PIN_PING_ECHO_LEFT, MAX_PING_SENSOR_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarR(PIN_PING_TRIGGER_RIGHT, PIN_PING_ECHO_RIGHT, MAX_PING_SENSOR_DISTANCE); // NewPing setup of pins and maximum distance.

int current_dir, last_dir;
int sensor_normalization_delta;

void setup() {
  #ifdef _DEBUG
  Serial.begin(9600);
  Serial.println("setup");
  #endif
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_CDS, INPUT);
  left_motor.attach(PIN_SERVO_LEFT);
  right_motor.attach(PIN_SERVO_RIGHT);
  stop(DIR_LEFT);
  stop(DIR_RIGHT);
  weave_phase = 0;
  weave_dir = 1;
  next_breathe_at = 0L;
  breathe_dir = 1;
  shine_until = 0L;
  sleep_until = 0L;
  next_ping_at = 0L;
  shine_brightness = 0;
  sensor_normalization_delta = 0;
  playTune();
  digitalWrite(PIN_BUZZER, LOW);
  current_distance_l = 100;
  current_distance_r = 100;
  analogWrite(PIN_LED, 10);
}

void recordDirection(int dir) {
  last_dir = current_dir;
  current_dir = dir;
}

int smooth(int array[], int len) {
  /**
    Return the average of the array without the highest and lowest values.
  **/
  int low = MAX_SENSOR_READING;
  int high = -1;
  int total = 0;
  for (int s = 0; s < len; s++) {
    total += array[s];
    low = min(array[s], low);
    high = max(array[s], high);
  }
  total -= low;
  total -= high;
  return total / (len - 2);
}

void readSensors() {  // raygeeknyc@
  wakeup_from_sensors = false;
  int l = getLightLevel();
  light_delta = light_level - l;
  light_level = l;
  // Don't read the ping sensors too often
  if (next_ping_at > millis()) {
    #ifdef _DEBUG
    Serial.print("Reuse old  distances. ");
    Serial.print("right: ");
    Serial.print(current_distance_r);
    Serial.print(",left: ");
    Serial.println(current_distance_l);
    #endif
  } else {
    prev_distance_l = current_distance_l;
    prev_distance_r = current_distance_r;
    current_distance_r = getRightPing();
    delay(POST_PING_DELAY_MS);
    current_distance_l = getLeftPing();
    ping_delta_l = current_distance_l - prev_distance_l;
    ping_delta_r = current_distance_r - prev_distance_r;
    next_ping_at = millis() + PING_SAMPLE_DELAY_MS;
  }
  if ((abs(light_delta) > LIGHT_CHANGE_THRESHOLD)
      || current_distance_l <= CLOSE_PROXIMITY_THRESHOLD
      || current_distance_r <= CLOSE_PROXIMITY_THRESHOLD) {
    last_sensor_activity_at = millis();
    wakeup_from_sensors = true;
    #ifdef _DEBUG
    Serial.println("sensor input");
    #endif
  }
}

int getLightLevel() {
  // Return the median reading from the light sensor
  int samples[SENSOR_SAMPLES];
  for (int sample = 0; sample < SENSOR_SAMPLES; sample++) {
    samples[sample] = analogRead(PIN_CDS);
  }
  return smooth(samples, SENSOR_SAMPLES);
}

bool timeToSnore() {
  int waking_in = sleep_until - millis();
  bool snoring = waking_in < SNORE_DURATION_MS;
  #ifdef _DEBUG
  Serial.print("Waking in ");
  Serial.println(waking_in);
  Serial.print(" snoring ");
  Serial.println(snoring);
  #endif
  return snoring;
}

/* Return true if we are currently sleeping, false if we're awake */
bool isSleeping() {  // raygeeknyc@
  bool sleeping = sleep_until > millis();
  #ifdef _DEBUG
  Serial.print("sleep until ");
  Serial.print(sleep_until);
  Serial.print(" sleeping ");
  Serial.println(sleeping);
  #endif
  return sleeping;
}

void sleep(const unsigned sleep_duration_secs) {
  #ifdef _DEBUG
  Serial.println("Sleep!");
  #endif
  sleep_until = millis() + (sleep_duration_secs * 1000);
  stop(DIR_LEFT);
  stop(DIR_RIGHT);
  awake_since = 0L;
}

void steerTowards(int bias) {
  #ifdef _DEBUG
  Serial.print("steertowards ");
  Serial.println(bias);
  #endif
  if (bias == -1) {
    fwd(DIR_RIGHT);
    slow(DIR_LEFT);
  } else if (bias == 1) {
    slow(DIR_RIGHT);
    fwd(DIR_LEFT);
  } else {
    fwd(DIR_LEFT);
    fwd(DIR_RIGHT);
  }
}

void weave() {
  #ifdef _DEBUG
  Serial.println("weave");
  #endif
  if (weave_phase_at < millis()) {
    #ifdef _DEBUG
    Serial.println("weave phase change");
    #endif
    weave_phase += weave_dir;
    if ((weave_phase == 0) || (weave_phase == (sizeof(weave_bias) / sizeof(int) - 1))) {
      weave_dir *= -1;
    }
    weave_phase_at = millis() + weave_phase_duration_ms[weave_phase];
    steerTowards(weave_bias[weave_phase]);
  }
}

void withdraw() {
  #ifdef _DEBUG
  Serial.println("withdraw");
  #endif
  flashLed();
  alarm();
  reverse(DIR_LEFT);
  reverse(DIR_RIGHT);
  delay(WITHDRAW_DUR_MS);
  turnFrom(DIR_LEFT);
  delay(TURN_DUR_MS);
  stop(DIR_LEFT);
  stop(DIR_RIGHT);
}

void slow(int side) {
  if (side == DIR_LEFT) {
    #ifdef _DEBUG
    Serial.println("slow left");
    #endif
    left_motor.write(SERVO_L_FWDSLOW);
  } else {
    #ifdef _DEBUG
    Serial.println("slow right");
    #endif
    right_motor.write(SERVO_R_FWDSLOW);
  }
}

void fwd(int side) {
  if (side == DIR_LEFT) {
    #ifdef _DEBUG
    Serial.println("fwd left");
    #endif
    left_motor.write(SERVO_L_FWD);
  } else {
    #ifdef _DEBUG
    Serial.println("fwd right");
    #endif
    right_motor.write(SERVO_R_FWD);
  }
}

void reverse(int side) {
  if (side == DIR_LEFT) {
    #ifdef _DEBUG
    Serial.println("bwd left");
    #endif
    left_motor.write(SERVO_L_BWD);
  } else {
    #ifdef _DEBUG
    Serial.println("bwd right");
    #endif
    right_motor.write(SERVO_R_BWD);
  }
}

void stop(int side) {
  if (side == DIR_LEFT) {
    #ifdef _DEBUG
    Serial.println("stop left");
    #endif
   left_motor.write(SERVO_L_STOP);
  } else {
    #ifdef _DEBUG
    Serial.println("stop right");
    #endif
    right_motor.write(SERVO_R_STOP);
  }
}

void turnFrom(const int side) {
  #ifdef _DEBUG
  Serial.print("turnFrom ");
  Serial.println((side == DIR_LEFT)?"left":"right");
  #endif
  fwd(side);
  reverse((side == DIR_LEFT) ? DIR_RIGHT : DIR_LEFT);
  delay(TURN_DUR_MS);
  stop(DIR_LEFT);
  stop(DIR_RIGHT);
}

bool isClose(const int distance) {
  return distance <= CLOSE_PROXIMITY_THRESHOLD;
}

/* Take the current step in moving about */
void roam() {  // raygeeknyc@
  #ifdef _DEBUG
  Serial.println("roam ");
  #endif
  if (!isClose(current_distance_l) && !isClose(current_distance_r)) {
    weave();
  } else if (isClose(current_distance_l) && isClose(current_distance_r)) {
    withdraw();
  } else {
    int closer_side = (current_distance_l  < current_distance_r) ? DIR_LEFT : DIR_RIGHT;
    dimLed();
    turnFrom(closer_side);
  }
}

/* Pulse the LED in sleep mode */
void breathe() {  // raygeeknyc@
  if (!next_breathe_at || (next_breathe_at < millis())) {
    #ifdef _DEBUG
    Serial.println("next breathe step");
    #endif
    next_breathe_at = millis() + BREATHE_STEP_DUR_MS;
    shine_brightness += BREATHE_STEP * ((breathe_dir > 0) ? 1 : -1);
    #ifdef _DEBUG
    Serial.print("breath brightness ");
    Serial.println(shine_brightness);
    #endif
    if ((shine_brightness <= BREATHE_MIN) || (shine_brightness >= BREATHE_MAX)) {
      breathe_dir *= -1;
      #ifdef _DEBUG
      Serial.print("breath direction now ");
      Serial.println(breathe_dir);
      #endif
    }
  }
}

/* Wake up, set flag, maybe make a waking noise or flash the LED */
void awaken() {  // raygeeknyc@
  sleep_until = 0l;
  awake_since = millis();
  next_breathe_at = 0;
  flashLed();
  burp();
}

bool isShining() {
  return (shine_until && shine_until > millis());
}

void flashLed() {
  shine_brightness = 255;
  shine_until = millis() + LED_FLASH_DURATION_MS;
}
void dimLed() {
  shine_brightness = 10;
  shine_until = millis() + LED_SHINE_DURATION_MS;
}

void updateLed() {
  analogWrite(PIN_LED, shine_brightness);
}

/* Make a sleeping sound in sleep mode.
  Since this function blocks, update the breathing state LED */
void snore() {  // raygeeknyc@
    beep(PIN_BUZZER, 125, 75);
    breathe();
    updateLed();
    beep(PIN_BUZZER, 75, 75);
    breathe();
    updateLed();
}

void beep(unsigned char pin, int frequencyInHertz, long timeInMilliseconds) {
  TimerFreeTone(pin, frequencyInHertz, timeInMilliseconds);
}

// The sound producing function for chips without tone() support
void _noToneBeep(unsigned char pin, int frequencyInHertz, long timeInMilliseconds) {
  
  // from http://web.media.mit.edu/~leah/LilyPad/07_sound_code.html
  int x;
  long delayAmount = (long)(1000000 / frequencyInHertz);
  long loopTime = (long)((timeInMilliseconds * 1000) / (delayAmount * 2));
  for (x = 0; x < loopTime; x++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(delayAmount);
    digitalWrite(pin, LOW);
    delayMicroseconds(delayAmount);
  }
}

// Emit a fairly shrill noise
void alarm() {
  beep(PIN_BUZZER, 300, 300);
}

// Emit a fairly rude noise
void burp() {
  beep(PIN_BUZZER, 75, 80);
  beep(PIN_BUZZER, 125, 50);
  beep(PIN_BUZZER, 75, 80);
}

void playTune() {
 beep(PIN_BUZZER,NOTE_C4,1000);
 beep(PIN_BUZZER,NOTE_G4,1000);
 beep(PIN_BUZZER,NOTE_F4,250);
 beep(PIN_BUZZER,NOTE_E4,250);
 beep(PIN_BUZZER,NOTE_D4,250);
 beep(PIN_BUZZER,NOTE_C5,1000);
 beep(PIN_BUZZER,NOTE_G4,500);
 beep(PIN_BUZZER,NOTE_F4,250);
 beep(PIN_BUZZER,NOTE_E4,250);
 beep(PIN_BUZZER,NOTE_D4,250);
 beep(PIN_BUZZER,NOTE_C5,1000);
 beep(PIN_BUZZER,NOTE_G4,500);
 beep(PIN_BUZZER,NOTE_F4,250);
 beep(PIN_BUZZER,NOTE_E4,250);
 beep(PIN_BUZZER,NOTE_F4,250);
 beep(PIN_BUZZER,NOTE_D4,2000);
}

int getLeftPing() {
  return getPingSensorReading(sonarL);
}

int getRightPing() {
  return getPingSensorReading(sonarR);
}

int getPingSensorReading(NewPing sonar) {
  int echoTime = sonar.ping_median(PING_SAMPLES);
  int cm = sonar.convert_cm(echoTime);
  if (cm == 0 || cm > MAX_PING_SENSOR_DISTANCE) {
    #ifdef _DEBUG
    Serial.println("adjusting distance to max distance");
    #endif
    cm = MAX_PING_SENSOR_DISTANCE;
  }
#ifdef _DEBUG
  Serial.print("Distance ");
  Serial.println(cm);
#endif
  return cm;
}

bool hasBeenAwoken() {
  return wakeup_from_sensors;
}

bool checkForSleep() {
  if (checkForWake()) {
    awaken();
    return false;
  }
  if (awake_since && (millis() - awake_since) < MIN_TIME_AWAKE_MS) {
    #ifdef _DEBUG
    Serial.println("not sleeping due to recent awakening");
    #endif
   return false;
  }
  if ((millis() - last_sensor_activity_at) > INACTIVITY_TIME_TO_NAP_MS) {
    #ifdef _DEBUG
    Serial.println("Nap due to lack of sensory input");
    #endif
    return true;
  }
  if ((millis() - awake_since) > ACTIVITY_TIME_TO_NAP_MS) {
    #ifdef _DEBUG
    Serial.println("Nap due to long activity");
    #endif
    return true;
  }
  return false;
}

bool checkForWake() {
  return sleep_until && sleep_until < millis();
}

void loop() {
  readSensors();  // raygeeknyc@
  #ifdef _DEBUG
  Serial.print("LEFT: ");
  Serial.print(current_distance_l);
  Serial.print(", RIGHT: ");
  Serial.print(current_distance_r);
  Serial.print(", LIGHT: ");
  Serial.println(light_level);
  Serial.flush();
  #endif
  updateLed();  // raygeeknyc@ : done
  if (!isSleeping()) {  // raygeeknyc@ : done
    if (!isShining()) {
      shine_brightness = 0;
    }
    if (checkForSleep()) {
       sleep(SLEEP_DURATION_SECS);
    } else {
      roam();  // raygeeknyc@ : done
    }
  }
  if (isSleeping()) {  // raygeeknyc@ : done
    #ifdef _DEBUG
    Serial.println("sleeping");
    #endif
    breathe();  // raygeeknyc@ : done
    if (checkForWake() || hasBeenAwoken()) {  // raygeeknyc@
      awaken();  // raygeeknyc@ : done
    } else {
      if (timeToSnore()) {
        snore();  // raygeeknyc@ : done
      }
    }
  }
}
