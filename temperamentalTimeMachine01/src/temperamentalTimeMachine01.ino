// Temperamental Time Machine — Motor Control
// ============================================================
// Stage 1: Single motor, slide on bass string.
// End goal: 4 motors controlling slide positions → musical pitch/interval control.
//
// Hardware:
//   MCU:    Teensy 3.5
//   Motor:  NEMA 17 42BYGH4807 (200 steps/rev, 4-wire)
//           https://www.circuitspecialists.com/nema_17_stepper_motor_42bygh4807
//   Driver: A4988
//           https://www.amazon.com/dp/B07BND65C8
//   Limit switch: pin 23, external pull-up, active LOW (triggers when slide reaches home)
//   Pot:    A13 (reserved, currently unused)
//
// References:
//   https://lastminuteengineers.com/a4988-stepper-motor-driver-arduino-tutorial/
//   https://www.pololu.com/product/1182
//   https://www.airspayce.com/mikem/arduino/AccelStepper/index.html
//   https://youtu.be/TWMai3oirnM?si=IP0x5L8xtVg3shti  (How Stepper Motors Work)
//   https://youtu.be/sER5GNjcQ70?si=cHnAq-LED0kjjmdD  (Arduino + A4988 circuit)
// ============================================================

#include <AccelStepper.h>
#include <elapsedMillis.h>

// --- Pin assignments ---
AccelStepper myStepper(AccelStepper::DRIVER, 14, 15);  // step=14, dir=15
const int enablePin      = 16;
const int limitSwitchPin = 23;  // external pull-up, active LOW
const int ms1Pin         = 17;
const int ms2Pin         = 18;
const int ms3Pin         = 19;
const int potPin         = A13;

// --- Motor configuration ---
const int STEPS_PER_REV_FULL = 200;  // NEMA 17: 200 steps/rev at full step
const int MICROSTEP_MODE     = 8;    // 8× microstepping → 1600 steps/rev

// Microstepping pin levels (MS1, MS2, MS3):
// 1  (full):      LOW,  LOW,  LOW
// 2  (half):      HIGH, LOW,  LOW
// 4  (quarter):   LOW,  HIGH, LOW
// 8  (eighth):    HIGH, HIGH, LOW   ← current
// 16 (sixteenth): HIGH, HIGH, HIGH
const int MS1_MODE = HIGH;
const int MS2_MODE = HIGH;
const int MS3_MODE = LOW;

// --- Homing ---
const int  HOMING_SPEED     = 400;       // steps/sec during homing approach
const int  HOMING_DIR       = -1;        // direction toward limit switch; flip to +1 if needed
const long HOMING_MAX_STEPS = 200000L;   // max travel before declaring a homing error

// --- Startup motor test (optional) ---
const bool RUN_MOTOR_TEST_AT_STARTUP = false;
const int  MOTOR_TEST_REVOLUTIONS    = 10;
const int  MOTOR_TEST_SPEED          = 600;
const int  MOTOR_TEST_ACCELERATION   = 800;

// --- Switch debounce ---
const unsigned long SWITCH_DEBOUNCE_MS = 30;

// --- Normal operation speeds ---
const int  NORMAL_MAX_SPEED    = 10000;
const int  NORMAL_ACCELERATION = 14000;

// ============================================================
// State machine
// ============================================================
enum MotorState {
  STATE_IDLE,     // pre-homing / error
  STATE_TESTING,  // optional startup motor test
  STATE_HOMING,   // moving toward limit switch to find home
  STATE_HOMED     // position is zeroed; accepts serial commands
};
MotorState motorState = STATE_IDLE;

// --- Limit switch (debounced) ---
bool lastRawLimitSwitchState   = HIGH;
bool debouncedLimitSwitchState = HIGH;
elapsedMillis timeSinceLastSwitchTransition;

// --- Non-blocking serial receive ---
// From: https://forum.arduino.cc/t/serial-input-basics-updated/382007/3
const byte numChars = 32;
char     receivedChars[numChars];
boolean  newData   = false;
int      dataNumber = 0;

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(9600);

  pinMode(ms1Pin,         OUTPUT);
  pinMode(ms2Pin,         OUTPUT);
  pinMode(ms3Pin,         OUTPUT);
  pinMode(enablePin,      OUTPUT);
  pinMode(limitSwitchPin, INPUT);   // relies on external pull-up

  digitalWrite(ms1Pin,    MS1_MODE);
  digitalWrite(ms2Pin,    MS2_MODE);
  digitalWrite(ms3Pin,    MS3_MODE);
  digitalWrite(enablePin, LOW);     // LOW = driver enabled

  myStepper.setMaxSpeed(NORMAL_MAX_SPEED);
  myStepper.setAcceleration(NORMAL_ACCELERATION);

  if (RUN_MOTOR_TEST_AT_STARTUP) {
    startMotorTest();
  } else {
    startHoming();
  }
}

// ============================================================
// Loop
// ============================================================
void loop() {
  updateLimitSwitch();

  switch (motorState) {
    case STATE_TESTING: updateMotorTest(); break;
    case STATE_HOMING:  updateHoming();    break;
    case STATE_HOMED:   updateHomed();     break;
    default:                               break;
  }

  myStepper.run();
}

// ============================================================
// Limit switch — debounced, non-blocking
// ============================================================
void updateLimitSwitch() {
  bool rawState = digitalRead(limitSwitchPin);

  if (rawState != lastRawLimitSwitchState) {
    lastRawLimitSwitchState = rawState;
    timeSinceLastSwitchTransition = 0;
  }

  if (timeSinceLastSwitchTransition >= SWITCH_DEBOUNCE_MS &&
      rawState != debouncedLimitSwitchState) {
    debouncedLimitSwitchState = rawState;
    if (debouncedLimitSwitchState == LOW) {
      Serial.println("[SWITCH] Limit switch pressed.");
    }
  }
}

// ============================================================
// Motor test — optional startup sequence, non-blocking
// ============================================================
void startMotorTest() {
  long testSteps = (long)MOTOR_TEST_REVOLUTIONS * STEPS_PER_REV_FULL * MICROSTEP_MODE;
  Serial.print("[TEST] Starting motor test: ");
  Serial.print(MOTOR_TEST_REVOLUTIONS);
  Serial.print(" rev (");
  Serial.print(testSteps);
  Serial.println(" steps)");

  myStepper.setMaxSpeed(MOTOR_TEST_SPEED);
  myStepper.setAcceleration(MOTOR_TEST_ACCELERATION);
  myStepper.moveTo(myStepper.currentPosition() + testSteps);
  motorState = STATE_TESTING;
}

void updateMotorTest() {
  if (myStepper.distanceToGo() == 0) {
    Serial.println("[TEST] Motor test complete.");
    myStepper.setMaxSpeed(NORMAL_MAX_SPEED);
    myStepper.setAcceleration(NORMAL_ACCELERATION);
    startHoming();
  }
}

// ============================================================
// Homing — non-blocking
// Move toward limit switch; zero position when switch triggers.
// HOMING_DIR = -1 if switch is at the negative end of travel, +1 if positive.
// ============================================================
void startHoming() {
  Serial.println("[HOMING] Starting homing sequence...");
  myStepper.setMaxSpeed(HOMING_SPEED);
  myStepper.moveTo((long)HOMING_DIR * HOMING_MAX_STEPS);
  motorState = STATE_HOMING;
}

void updateHoming() {
  if (debouncedLimitSwitchState == LOW) {
    // Limit switch triggered — stop immediately and zero position.
    // AccelStepper::setCurrentPosition() also clears target and speed.
    myStepper.setCurrentPosition(0);
    myStepper.setMaxSpeed(NORMAL_MAX_SPEED);
    motorState = STATE_HOMED;
    Serial.println("[HOMING] Complete. Position zeroed.");
    return;
  }

  if (myStepper.distanceToGo() == 0) {
    // Traveled HOMING_MAX_STEPS without triggering switch — check wiring.
    Serial.println("[HOMING] ERROR: Limit switch not reached. Check wiring and HOMING_DIR.");
    motorState = STATE_IDLE;
  }
}

// ============================================================
// Normal operation (homed) — accepts angle commands via serial.
// Send an integer 0–360 followed by newline to move to that angle.
// Position is relative to home (0 = home / limit switch position).
// ============================================================
void updateHomed() {
  recvWithEndMarker();

  if (newData) {
    int angle = dataNumber;
    long stepsPerRevolution = (long)STEPS_PER_REV_FULL * MICROSTEP_MODE;
    long targetSteps = map((long)angle, 0L, 360L, 0L, stepsPerRevolution);
    Serial.print("[HOMED] angle=");
    Serial.print(angle);
    Serial.print(" deg → step target=");
    Serial.println(targetSteps);
    myStepper.moveTo(targetSteps);
    newData = false;
  }
}

// ============================================================
// Non-blocking serial receive
// Accumulates chars until newline; sets newData=true and
// populates dataNumber with the integer value.
// From: https://forum.arduino.cc/t/serial-input-basics-updated/382007/3
// ============================================================
void recvWithEndMarker() {
  static byte ndx = 0;
  char rc;

  if (Serial.available() > 0) {
    rc = Serial.read();
    if (rc != '\n') {
      receivedChars[ndx] = rc;
      if (++ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      receivedChars[ndx] = '\0';
      ndx        = 0;
      newData    = true;
      dataNumber = atoi(receivedChars);
    }
  }
}
