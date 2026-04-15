// Temperamental Time Machine — Motor Control
// ============================================================
// Stage 1-2: All four motors declared; homing and serial angle control active.
// End goal: 4 motors controlling slide positions → musical pitch/interval control.
//
// Hardware:
//   MCU:    Teensy 3.5
//   Motor:  NEMA 17 42BYGH4807 (200 steps/rev, 4-wire) × 4
//           https://www.circuitspecialists.com/nema_17_stepper_motor_42bygh4807
//   Driver: A4988 × 4  (enable and MS pins shared)
//           https://www.amazon.com/dp/B07BND65C8
//   Pins:   Motor 0  step=14 dir=15  limit=23
//           Motor 1  step=37 dir=38  limit=22
//           Motor 2  step=35 dir=36  limit=21
//           Motor 3  step=33 dir=34  limit=20
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
const int NUM_MOTORS = 4;

// One AccelStepper per motor. Enable and MS pins are shared across all drivers.
AccelStepper steppers[NUM_MOTORS] = {
  AccelStepper(AccelStepper::DRIVER, 14, 15),  // Motor 0
  AccelStepper(AccelStepper::DRIVER, 37, 38),  // Motor 1
  AccelStepper(AccelStepper::DRIVER, 35, 36),  // Motor 2
  AccelStepper(AccelStepper::DRIVER, 33, 34)   // Motor 3
};

const int enablePin = 16;
const int ms1Pin    = 17;
const int ms2Pin    = 18;
const int ms3Pin    = 19;
const int potPin    = A13;

// Limit switch pins, one per motor (motor 0–3).
// All use external pull-up resistors; active LOW when pressed.
const int limitSwitchPins[NUM_MOTORS] = {23, 22, 21, 20};

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

// --- Mechanical drive ---
// Lead screw: 8 mm linear travel per 360° revolution.
// At 8× microstepping: 1600 steps/rev → 200 steps/mm
const float MM_PER_REV         = 8.0;
const float STEPS_PER_MM       = (STEPS_PER_REV_FULL * MICROSTEP_MODE) / MM_PER_REV;  // 200.0

// --- Homing ---
const int  HOMING_SPEED        = 400;      // steps/sec during homing approach
const int  HOMING_DIR          = -1;       // direction toward limit switch; flip to +1 if needed
const long HOMING_MAX_STEPS    = 200000L;  // max travel before declaring a homing error
const int  HOMING_BACKOFF_MM   = 10;       // mm to reverse off switch after contact
const long HOMING_BACKOFF_STEPS = (long)(HOMING_BACKOFF_MM * STEPS_PER_MM);  // 2000

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
  STATE_IDLE,             // pre-homing / error
  STATE_TESTING,          // optional startup motor test (motor 0 only)
  STATE_HOMING,           // moving toward limit switch to find home
  STATE_HOMING_BACKOFF,   // reversing off the limit switch after contact
  STATE_HOMED             // position is zeroed; accepts commands
};
MotorState motorStates[NUM_MOTORS] = {
  STATE_IDLE, STATE_IDLE, STATE_IDLE, STATE_IDLE
};

// --- Limit switches (debounced, one entry per motor) ---
bool lastRawSwitchState[NUM_MOTORS]   = {HIGH, HIGH, HIGH, HIGH};
bool debouncedSwitchState[NUM_MOTORS] = {HIGH, HIGH, HIGH, HIGH};
elapsedMillis timeSinceLastTransition[NUM_MOTORS];
elapsedMillis timeSinceLastSwitchReport;

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
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(limitSwitchPins[i], INPUT);  // relies on external pull-up
  }

  digitalWrite(ms1Pin,    MS1_MODE);
  digitalWrite(ms2Pin,    MS2_MODE);
  digitalWrite(ms3Pin,    MS3_MODE);
  digitalWrite(enablePin, LOW);     // LOW = driver enabled

  for (int i = 0; i < NUM_MOTORS; i++) {
    steppers[i].setMaxSpeed(NORMAL_MAX_SPEED);
    steppers[i].setAcceleration(NORMAL_ACCELERATION);
  }

  if (RUN_MOTOR_TEST_AT_STARTUP) {
    startMotorTest();                        // motor 0 → STATE_TESTING
    for (int i = 1; i < NUM_MOTORS; i++) {
      startHoming(i);                        // motors 1–3 → straight to homing
    }
  } else {
    for (int i = 0; i < NUM_MOTORS; i++) {
      startHoming(i);
    }
  }
}

// ============================================================
// Loop
// ============================================================
void loop() {
  updateLimitSwitch();

  for (int i = 0; i < NUM_MOTORS; i++) {
    switch (motorStates[i]) {
      case STATE_TESTING:        updateMotorTest();       break;  // motor 0 only
      case STATE_HOMING:         updateHoming(i);         break;
      case STATE_HOMING_BACKOFF: updateHomingBackoff(i);  break;
      case STATE_HOMED:          updateHomed(i);          break;
      default:                                            break;
    }
    steppers[i].run();
  }
}

// ============================================================
// Limit switches — debounced, non-blocking, all motors
// ============================================================
void updateLimitSwitch() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    bool rawState = digitalRead(limitSwitchPins[i]);

    if (rawState != lastRawSwitchState[i]) {
      lastRawSwitchState[i] = rawState;
      timeSinceLastTransition[i] = 0;
      Serial.print("[SWITCH ");
      Serial.print(i);
      Serial.print("] Raw transition → ");
      Serial.println(rawState == LOW ? "LOW (pressed)" : "HIGH (open)");
    }

    if (timeSinceLastTransition[i] >= SWITCH_DEBOUNCE_MS &&
        rawState != debouncedSwitchState[i]) {
      debouncedSwitchState[i] = rawState;
      Serial.print("[SWITCH ");
      Serial.print(i);
      Serial.print("] Debounced → ");
      Serial.println(debouncedSwitchState[i] == LOW ? "LOW (pressed)" : "HIGH (open)");
    }
  }

  // Periodic report every 500 ms so pin states are always visible.
  if (timeSinceLastSwitchReport >= 500) {
    timeSinceLastSwitchReport = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
      Serial.print("[SWITCH ");
      Serial.print(i);
      Serial.print("] raw=");
      Serial.print(digitalRead(limitSwitchPins[i]) == LOW ? "LOW" : "HIGH");
      Serial.print("  debounced=");
      Serial.println(debouncedSwitchState[i] == LOW ? "LOW" : "HIGH");
    }
  }
}

// ============================================================
// Motor test — optional startup sequence, non-blocking
// ============================================================
void startMotorTest() {
  long testSteps = (long)MOTOR_TEST_REVOLUTIONS * STEPS_PER_REV_FULL * MICROSTEP_MODE;
  Serial.print("[TEST] Starting motor test on motor 0: ");
  Serial.print(MOTOR_TEST_REVOLUTIONS);
  Serial.print(" rev (");
  Serial.print(testSteps);
  Serial.println(" steps)");

  steppers[0].setMaxSpeed(MOTOR_TEST_SPEED);
  steppers[0].setAcceleration(MOTOR_TEST_ACCELERATION);
  steppers[0].moveTo(steppers[0].currentPosition() + testSteps);
  motorStates[0] = STATE_TESTING;
}

void updateMotorTest() {
  if (steppers[0].distanceToGo() == 0) {
    Serial.println("[TEST] Motor test complete.");
    steppers[0].setMaxSpeed(NORMAL_MAX_SPEED);
    steppers[0].setAcceleration(NORMAL_ACCELERATION);
    startHoming(0);
  }
}

// ============================================================
// Homing — non-blocking, all motors run in parallel.
// Each motor moves toward its limit switch, zeros its position
// on contact, then reverses HOMING_BACKOFF_STEPS to clear the switch.
// HOMING_DIR = -1 if switch is at the negative end of travel, +1 if positive.
// ============================================================
void startHoming(int i) {
  Serial.print("[HOMING] Motor ");
  Serial.print(i);
  Serial.println(" starting homing sequence...");
  steppers[i].setMaxSpeed(HOMING_SPEED);
  steppers[i].moveTo((long)HOMING_DIR * HOMING_MAX_STEPS);
  motorStates[i] = STATE_HOMING;
}

void updateHoming(int i) {
  if (debouncedSwitchState[i] == LOW) {
    // Limit switch triggered — zero position, then back off the switch.
    // setCurrentPosition() also clears the current target and speed.
    steppers[i].setCurrentPosition(0);
    steppers[i].moveTo(-(long)HOMING_DIR * HOMING_BACKOFF_STEPS);
    motorStates[i] = STATE_HOMING_BACKOFF;
    Serial.print("[HOMING] Motor ");
    Serial.print(i);
    Serial.println(" switch triggered. Backing off...");
    return;
  }

  if (steppers[i].distanceToGo() == 0) {
    Serial.print("[HOMING] ERROR: Motor ");
    Serial.print(i);
    Serial.println(" limit switch not reached. Check wiring and HOMING_DIR.");
    motorStates[i] = STATE_IDLE;
  }
}

void updateHomingBackoff(int i) {
  if (steppers[i].distanceToGo() == 0) {
    steppers[i].setMaxSpeed(NORMAL_MAX_SPEED);
    motorStates[i] = STATE_HOMED;
    Serial.print("[HOMING] Motor ");
    Serial.print(i);
    Serial.print(" homed. Resting at ");
    Serial.print(steppers[i].currentPosition());
    Serial.println(" steps from switch.");
  }
}

// ============================================================
// Normal operation (homed) — serial angle commands for motor 0.
// Send an integer 0–360 followed by newline to move to that angle.
// Position is relative to home (0 = limit switch contact point;
// motor rests at +HOMING_BACKOFF_STEPS after homing).
// Distance-based control for all motors will replace this once
// the mechanical drive system is defined.
// ============================================================
void updateHomed(int i) {
  if (i != 0) return;

  recvWithEndMarker();

  if (newData) {
    int angle = dataNumber;
    long stepsPerRevolution = (long)STEPS_PER_REV_FULL * MICROSTEP_MODE;
    long targetSteps = map((long)angle, 0L, 360L, 0L, stepsPerRevolution);
    Serial.print("[HOMED 0] angle=");
    Serial.print(angle);
    Serial.print(" deg → step target=");
    Serial.println(targetSteps);
    steppers[0].moveTo(targetSteps);
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
