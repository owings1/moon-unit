/**
 * Commands:
 * 
 * 01 - Move single motor n steps in a given direction
 * 
 *  :01 <motorId> <direction> <steps>;
 * 
 * 02 - Set max speed for a motor
 * 
 *  :02 <motorId> <speed>;
 * 
 * 03 - Set acceleration for a motor
 * 
 *  :03 <motorId> <acceleration>;
 * 
 * 04 - Move single motor n degrees in a given direction
 * 
 *  :04 <motorId> <direction> <degrees>;
 */

// TODO:
//  - have separate maxAcceleration values for m1 and m2
//  - add command to move both motors at once
//  - auto home function: move to limit, then back a fixed amount
//  - write real encoder/lcd interface
//  - cancel/stop input pin
//  - state output pins -- ready, operating, error

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <RotaryEncoder.h>
#include <LiquidCrystal_I2C.h>
#include "dwf/digitalWriteFast.h"

// Motor

#define dirPin_m1 5
#define dirPin_m2 8
#define stepPin_m1 6
#define stepPin_m2 9
#define enablePin_m1 7
#define enablePin_m2 10
#define limitPinCw_m1 3
#define limitPinAcw_m1 4
#define limitPinCw_m2 11
#define limitPinAcw_m2 12
#define maxSpeed_m1 2000L
#define maxSpeed_m2 1000L
#define degreesPerStep_m1 0.45
#define degreesPerStep_m2 0.001125
// Whether the limit switches are connected
#define limitsEnabled_m1 0
#define limitsEnabled_m2 1
#define maxAcceleration 10000L
#define motorSleepTimeout 2000L

// limit switches
boolean isLimitCw_m1 = false;
boolean isLimitAcw_m1 = false;
boolean isLimitCw_m2 = false;
boolean isLimitAcw_m2 = false;

// track the acceleration value set in stepper objects.
long acceleration_m1;
long acceleration_m2;

unsigned long lastMotorActionTime = millis();
boolean isMotorEnabled = false;

AccelStepper stepper_m1(AccelStepper::FULL2WIRE, stepPin_m1, dirPin_m1);
AccelStepper stepper_m2(AccelStepper::FULL2WIRE, stepPin_m2, dirPin_m2);

// Encoder

// These cannot be changed without addressing interrupt as below
#define encoderPin1 A2
#define encoderPin2 A3

RotaryEncoder encoder(encoderPin1, encoderPin2);

// This cannot be changed since we are attaching to interrupt 0 (pin 2)
#define encoderSwitchPin 2
#define debouncing_time 15 //Debouncing Time in Milliseconds

// The encoder switch changes the "program"
volatile unsigned long debounceLastMicros;
#define numPrograms 3
volatile int programNum = 1;

// LCD Display

// https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified module

#define displaySleepTimeout 10000L

unsigned long lastDisplayActionTime = millis();
volatile boolean displayUpdateNeeded = false;
boolean isDisplayLightOn = false;

void setup() {
  Serial.begin(115200L);
  setupMotors();
  setupEncoder();
  setupDisplay();
}

void loop() {

  // TODO: move to canMotorMove ?
  if (limitsEnabled_m1) {
    isLimitCw_m1  = digitalReadFast(limitPinCw_m1)  == HIGH;
    isLimitAcw_m2 = digitalReadFast(limitPinAcw_m1) == HIGH;
  }

  if (limitsEnabled_m2) {
    isLimitCw_m2  = digitalReadFast(limitPinCw_m2)  == HIGH;
    isLimitAcw_m2 = digitalReadFast(limitPinAcw_m2) == HIGH;
  }

  static int pos = 0;

  int newPos = encoder.getPosition();
  
  if (pos != newPos) {
    if (shouldTakeInput()) {
      takeKnobInput(pos, newPos);
    }
    // if we are not taking input, this will ignore changes to knob position
    pos = newPos;
  }

  if (!runMotorsIfNeeded()) {
    updateDisplay();
    checkDisplaySleep();
    checkMotorSleep();
    takeCommand(Serial, Serial);
  }
}

// ----------------------------------------------
// Command input functions
// ----------------------------------------------

void takeCommand(Stream &input, Stream &output) {

  if (!input.available()) {
    return;
  }
  
  byte firstByte = input.read();

  if (firstByte != ':') {
    output.write("=40\n");
    return;
  }

  String command = input.readStringUntil(' ');

  if (command.equals("01")) {

    // Move a motor n steps in a direction

    // first param is the motor id, 1 or 2

    int motorId = readMotorIdFromInput(input);
    if (motorId == 0) {
      output.write("=46\n");
      return;
    }

    // second param is direction 1: clockwise, 2: anti-clockwise

    int dir = input.readStringUntil(' ').toInt();

    int dirMult = getDirMultiplier(dir);
    if (dirMult == 0) {
      output.write("=47\n");
      return;
    }

    // third param is how many steps

    long howMuch = input.readStringUntil(';').toInt() * dirMult;
    if (howMuch == 0) {
      output.write("=48\n");
      return;
    }

    // perform action
    jumpOne(motorId, howMuch);

    output.write("=00\n");

  } else if (command.equals("02")) {

    // Set max speed for motor

    // first param is the motor id

    int motorId = readMotorIdFromInput(input);
    if (motorId == 0) {
      output.write("=46\n");
      return;
    }

    // second param is the speed
    long newSpeed = input.readStringUntil(';').toInt();
    if (newSpeed == 0) {
      output.write("=49\n");
      return;
    }

    if (motorId == 1) {
      // TODO: figure out why we couldn't dynamically assign this
      stepper_m1.setMaxSpeed(min(newSpeed, maxSpeed_m1));
    } else if (motorId == 2) {
      stepper_m2.setMaxSpeed(min(newSpeed, maxSpeed_m2));
    }

    output.write("=00\n");
  } else if (command.equals("03")) {

    // Set acceleration for motor

    // first param is the motor id

    int motorId = readMotorIdFromInput(input);
    if (motorId == 0) {
      output.write("=46\n");
      return;
    }

    // second param is the acceleration
    long newAccel = input.readStringUntil(';').toInt();
    if (newAccel == 0) {
      output.write("=50\n");
      return;
    }

    setAcceleration(motorId, newAccel);

    output.write("=00\n");

  } else if (command.equals("04")) {
    // Move a motor n degrees in a direction

    // first param is the motor id

    int motorId = readMotorIdFromInput(input);
    if (motorId == 0) {
      output.write("=46\n");
      return;
    }

    // second param is direction 1: clockwise, 2: anti-clockwise

    int dir = input.readStringUntil(' ').toInt();

    int dirMult = getDirMultiplier(dir);
    if (dirMult == 0) {
      output.write("=47\n");
      return;
    }

    // third param is how many degrees

    float howMuch = input.readStringUntil(';').toFloat() * dirMult;
    if (howMuch == 0) {
      output.write("=48\n");
      return;
    }

    // perform action

    jumpOneByDegrees(motorId, howMuch);

    output.write("=00\n");
  } else {
    output.write("=44\n");
  }
}

int readMotorIdFromInput(Stream &input) {
  int motorId = input.readStringUntil(' ').toInt();
  if (motorId == 1 || motorId == 2) {
    return motorId;
  }
  return 0;
}

int getDirMultiplier(int dirInput) {
  if (dirInput == 1) {
    return 1;
  } else if (dirInput == 2) {
    return -1;
  }
  return 0;
}

// ----------------------------------------------
// Motor functions
// ----------------------------------------------

boolean motorCanMove(int motorId, long howMuch) {
  if (motorId == 1) {
    if (howMuch > 0) {
      return !isLimitCw_m1;
    } else {
      return !isLimitAcw_m1;
    }
  }
  if (motorId == 2) {
    if (howMuch > 0) {
      return !isLimitCw_m2;
    } else {
      return !isLimitAcw_m2;
    }
  }
}

boolean runMotorsIfNeeded() {
  // TODO: make limit drain DRY
  boolean isRun = false;
  if (stepper_m1.distanceToGo() != 0) {
    if (!motorCanMove(1, stepper_m1.distanceToGo())) {
      long oldAcceleration_m1 = acceleration_m1;
      setAcceleration(1, maxAcceleration);
      stepper_m1.stop();
      while (stepper_m1.distanceToGo() != 0) {
        stepper_m1.run();
      }
      setAcceleration(1, oldAcceleration_m1);
    } else {
      stepper_m1.run();
    }
    registerMotorAction();
    isRun = true;
  }
  if (stepper_m2.distanceToGo() != 0) {
    if (!motorCanMove(2, stepper_m2.distanceToGo())) {
      long oldAcceleration_m2 = acceleration_m2;
      setAcceleration(2, maxAcceleration);
      stepper_m2.stop();
      while (stepper_m2.distanceToGo() != 0) {
        stepper_m2.run();
      }
      setAcceleration(2, oldAcceleration_m2);
    } else {
      stepper_m2.run();
    }
    registerMotorAction();
    isRun = true;
  }
  return isRun;
}

void jumpBoth(long howMuch) {
  jumpOne(1, howMuch);
  jumpOne(2, howMuch);
}

void jumpOne(int motorId, long howMuch) {
  if (motorCanMove(motorId, howMuch)) {
    if (motorId == 1) {
      stepper_m1.move(howMuch);
    } else if (motorId == 2) {
      stepper_m2.move(howMuch);
    }
    enableMotors();
  }
}

void jumpOneByDegrees(int motorId, float howMuch) {

  long steps;
  if (motorId == 1) {
    steps = howMuch / degreesPerStep_m1;
    if (motorCanMove(motorId, steps)) {
      stepper_m1.move(steps);
      enableMotors();
    }
  } else if (motorId == 2) {
    steps = howMuch / degreesPerStep_m2;
    if (motorCanMove(motorId, steps)) {
      stepper_m2.move(steps);
      enableMotors();
    }
  }
}

void setMaxSpeed(int motorId, long value) {
  if (motorId == 1) {
    stepper_m1.setMaxSpeed(min(value, maxSpeed_m1));
  } else if (motorId == 2) {
    stepper_m2.setMaxSpeed(min(value, maxSpeed_m2));
  }
}

void setAcceleration(int motorId, long value) {
  if (motorId == 1) {
    acceleration_m1 = min(value, maxAcceleration);
    stepper_m1.setAcceleration(acceleration_m1);
  } else if (motorId == 2) {
    acceleration_m2 = min(value, maxAcceleration);
    stepper_m2.setAcceleration(acceleration_m2);
  }
}

void enableMotors() {
  if (!isMotorEnabled) {
    digitalWrite(enablePin_m1, LOW);
    digitalWrite(enablePin_m2, LOW);
    isMotorEnabled = true;
    delay(2);
  }
  registerMotorAction();
}

void disableMotors() {
  if (isMotorEnabled) {
    digitalWrite(enablePin_m1, HIGH);
    digitalWrite(enablePin_m2, HIGH);
    isMotorEnabled = false;
  }
}

void registerMotorAction() {
  lastMotorActionTime = millis();
}

void checkMotorSleep() {
  unsigned long elapsed = millis() - lastMotorActionTime;
  if (elapsed > motorSleepTimeout) {
    disableMotors();
  }
}

// ----------------------------------------------
// Encoder functions
// ----------------------------------------------

void takeKnobInput(int oldPos, int newPos) {
  // TODO: this is just a placeholder, write a real encoder UI.
  if (oldPos != newPos) {
    turnOnDisplayLight();
    if (newPos > oldPos) {
      jumpBoth(800 * programNum);
    } else {
      jumpBoth(-800 * programNum);
    }
    registerDisplayAction();
  }
}

boolean shouldTakeInput() {
  return stepper_m1.distanceToGo() == 0 && stepper_m2.distanceToGo() == 0;
}

void onEncoderSwitchChange() {
  if (!shouldTakeInput()) {
    // ignore switch input
    return;
  }
  if (programNum >= numPrograms) {
    programNum = 1;
  } else {
    programNum += 1;
  }
  displayUpdateNeeded = true;
  registerDisplayAction();
}

// ----------------------------------------------
// LCD display functions
// ----------------------------------------------

void updateDisplay() {
  if (shouldUpdateDisplay()) {
    lcd.setCursor(0, 0);
    lcd.print("Program: ");
    lcd.print(programNum);
    displayUpdateNeeded = false;
  }
}

boolean shouldUpdateDisplay() {
  return displayUpdateNeeded;
}

void turnOnDisplayLight() {
  lcd.setBacklightPin(3, NEGATIVE);
}

void turnOffDisplayLight() {
  lcd.setBacklightPin(3, POSITIVE);
}

void registerDisplayAction() {
  lastDisplayActionTime = millis();
}

void checkDisplaySleep() {
  unsigned long elapsed = millis() - lastDisplayActionTime;
  if (elapsed > displaySleepTimeout) {
    turnOffDisplayLight();
  } else {
    // for button, so we don't have to do this in an interrupt
    turnOnDisplayLight();
  }
}

// ----------------------------------------------
// Setup routines
// ----------------------------------------------

void setupMotors() {

  // Declare pins as output:
  pinMode(stepPin_m1, OUTPUT);
  pinMode(stepPin_m2, OUTPUT);
  pinMode(dirPin_m1, OUTPUT);
  pinMode(dirPin_m2, OUTPUT);
  pinMode(enablePin_m1, OUTPUT);
  pinMode(enablePin_m2, OUTPUT);

  // Declare limit switch pins as input
  pinMode(limitPinCw_m2, INPUT);
  pinMode(limitPinAcw_m2, INPUT);

  // set initial state of motor to disabled
  digitalWrite(enablePin_m1, HIGH);
  digitalWrite(enablePin_m2, HIGH);
  isMotorEnabled = false;

  // AccelStepper
  stepper_m1.setMaxSpeed(maxSpeed_m1);
  stepper_m2.setMaxSpeed(maxSpeed_m2);
  setAcceleration(1, maxAcceleration);
  setAcceleration(2, maxAcceleration);
}

void setupDisplay() {
  lcd.begin(16, 2);
  lcd.clear();
  displayUpdateNeeded = true;
  turnOnDisplayLight();
  updateDisplay();
}

void setupEncoder() {

  pinMode(encoderSwitchPin, OUTPUT);
  // use pullup resistor
  digitalWrite(encoderSwitchPin, HIGH);
  attachInterrupt(digitalPinToInterrupt(encoderSwitchPin), debounceInterrupt, RISING);

  // from https://github.com/mathertel/RotaryEncoder/blob/master/examples/InterruptRotator/InterruptRotator.ino#L35
  // You may have to modify the next 2 lines if using other pins than A2 and A3
  PCICR |= (1 << PCIE1);    // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);  // This enables the interrupt for pin 2 and 3 of Port C.
}

// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3: exactly where we need to check.
ISR(PCINT1_vect) {
  encoder.tick(); // just call tick() to check the state.
}

// ----------------------------------------------
// Miscellaneous Util
// ----------------------------------------------

// https://www.instructables.com/id/Arduino-Software-debouncing-in-interrupt-function/
void debounceInterrupt() {
  //Serial.println("Debounce interrupt");
  if ((long)(micros() - debounceLastMicros) >= debouncing_time * 1000) {
    onEncoderSwitchChange();
    debounceLastMicros = micros();
  }
}
