// Stepper Configuration
// Motor: NEMA 17 Stepper Motor (200 steps/revolution at full stepping)
// 4.2 kg-cm 4 Wire NEMA 17 Stepper Motor by CW-MOTOR. SKU: 42BYGH4807
//   https://www.circuitspecialists.com/nema_17_stepper_motor_42bygh4807
// Driver: A4988 Stepper Motor Driver
//   https://www.amazon.com/dp/B07BND65C8

// https://lastminuteengineers.com/a4988-stepper-motor-driver-arduino-tutorial/

// https://www.pololu.com/product/1182 

// https://www.airspayce.com/mikem/arduino/AccelStepper/index.html

// https://youtu.be/TWMai3oirnM?si=IP0x5L8xtVg3shti (How Stepper Motors Work)

// https://youtu.be/sER5GNjcQ70?si=cHnAq-LED0kjjmdD (Shows complete Arduino Uno implementation of circuit)



#include <AccelStepper.h> // Install this library if you haven't already done so!
AccelStepper myStepper(1, 14, 15);  // 1 sets the MotorInterfaceType to DRIVER; pin 14 = step; pin 15 = direction
//AccelStepper myStepper(4, 0, 2, 1, 3); // define motor pins (0, 2, 1, 3) and interface mode (4)

const int STEPS_PER_REV_FULL = 200;  // NEMA 17 motor: 200 steps/revolution at full stepping
const int MICROSTEP_MODE = 8;        // Microstepping divisor: 1, 2, 4, 8, or 16
                                      // Total steps/rev = STEPS_PER_REV_FULL * MICROSTEP_MODE



const int enablePin = 16;
const int limitSwitchPin = 23;  // limit switch for homing; external pull-up, default HIGH
const int ms1Pin = 17;          // define pins for stepping mode
const int ms2Pin = 18;          // stepping modes change the step resolution of the motor
const int ms3Pin = 19;          // higher resolution comes at the expense of higher speeds and torque


// Microstepping pin settings by MICROSTEP_MODE (MS1, MS2, MS3):
// 1  (full step):       LOW,  LOW,  LOW
// 2  (half step):       HIGH, LOW,  LOW
// 4  (quarter step):    LOW,  HIGH, LOW
// 8  (eighth step):     HIGH, HIGH, LOW
// 16 (sixteenth step):  HIGH, HIGH, HIGH
// Current MICROSTEP_MODE = 8 (eighth step): HIGH, HIGH, LOW
const int MS1_MODE = HIGH;
const int MS2_MODE = HIGH;
const int MS3_MODE = LOW;

int steps = 0;
int currentSteps = 0;
int previousSteps = 0;
// End Stepper Configuration

const int potPin = A13;

// Alternative to parseInt
// From: https://forum.arduino.cc/t/serial-input-basics-updated/382007/3
// Example 4 - Receive a number as text and convert it to an int

const byte numChars = 32;
char receivedChars[numChars];  // an array to store the received data

boolean newData = false;

int dataNumber = 0;  // new for this version
// end Alternative to parseInt


void setup() {
  Serial.begin(9600);  myStepper.setMaxSpeed(10000); // sets the maximum steps per second, which determines how fast the motor will turn
  myStepper.setAcceleration(14000); // sets the acceleration rate in steps per second
  myStepper.setSpeed(100);
  
  pinMode(ms1Pin, OUTPUT);        // set step mode pins as outputs
  pinMode(ms2Pin, OUTPUT);
  pinMode(ms3Pin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(limitSwitchPin, INPUT);

  digitalWrite(ms1Pin, MS1_MODE);
  digitalWrite(ms2Pin, MS2_MODE);
  digitalWrite(ms3Pin, MS3_MODE);
  digitalWrite(enablePin, LOW);

  homeMotor();
}

void loop() {
  // randStepper();
  // variableStepper();
   angleInputNB();
  // stepsInputNB();
}

void homeMotor() {
  myStepper.setMaxSpeed(300);
  myStepper.setSpeed(-300);  // slow negative-direction speed for homing
  while (digitalRead(limitSwitchPin) == HIGH) {
    myStepper.runSpeed();
  }
  myStepper.setCurrentPosition(0);  // mark this as the home position
  myStepper.setMaxSpeed(10000);     // restore original settings
  myStepper.setAcceleration(14000);
}

void randStepper()
{
    if (myStepper.distanceToGo() == 0)
    {
  // Random change to speed, position and acceleration
  // Make sure we dont get 0 speed or accelerations
  //delay(1000);
  myStepper.moveTo(rand() % 8000);
  myStepper.setMaxSpeed((rand() % 2000) + 50);
  myStepper.setAcceleration((rand() % 1000) + 1);
    }
    myStepper.run();
}


  void variableStepper(){
    int potVal = analogRead(potPin);
    float speed = map(potVal, 1023, 10, -4000, 4000);
    speed = constrain(speed, -4000, 4000);
    myStepper.setSpeed(speed);
    myStepper.runSpeed();

  }


void angleInputNB() {  //non-blocking version of angle input
  recvWithEndMarker();
  
  if (newData) {
    int angle = dataNumber;
    Serial.print("angle = ");
    Serial.println(angle);
    int stepsPerRevolution = STEPS_PER_REV_FULL * MICROSTEP_MODE;
    angle = map(angle, 0, 360, 0, stepsPerRevolution);
    myStepper.moveTo(angle);
    newData = false;
  }
  myStepper.run();
}

void stepsInputNB() {
  recvWithEndMarker();
  
  if (newData) {
    steps = dataNumber;
    Serial.print("steps = ");
    Serial.println(steps);
    myStepper.moveTo(steps);
    newData = false;
  }
  myStepper.run();
}

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  if (Serial.available() > 0) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      receivedChars[ndx] = '\0';  // terminate the string
      ndx = 0;
      newData = true;
      dataNumber = atoi(receivedChars);
    }
  }
}
