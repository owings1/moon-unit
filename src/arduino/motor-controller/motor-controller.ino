/**
 * Commands
 * ---------
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
 *
 * 05 - Get state of limit switches and stop pin
 *
 *  :05 ;
 *
 *    example response:
 *      =00;TFFT|F
 *
 * 06 - Home a single motor
 *
 *  :06 <motorId>;
 *
 * 07 - Home both motors
 *
 *  :07 ;
 *
 * 08 - End a single motor
 *
 *  :08 <motorId>;
 *
 * 09 - End both motors
 *
 *  :09 ;
 *
 * 10 - Move both motors by steps
 *
 *  :10 <direction_1> <steps_1> <direction_2> <steps_2>;
 *
 * 11 - Move both motors by degrees
 *
 *  :11 <direction_1> <degrees_1> <direction_2> <degrees_2>;
 *
 * 12 - Get motor positions
 *
 *  :12 <format>;
 *
 *    example responses:
 *      =00;8500|1200
 *      =00;?|130.195
 *      =00;?|?
 *
 * 13 - No response (debug)
 *
 *  :13 ;
 *
 * 14 - Get orientation (x|y|z|isCalibrated)
 *
 *  :14 ;
 *
 *    example responses:
 *      =00;143.2|43.02|123.5|F
 *      =50;
 *
 * 15 - Get position, followed by orientation
 *
 *  :15 ;
 *
 * 16 - Get orientation calibration status
 *
 *  :16 ;
 *
 *    example response:
 *      =00;0|3|1|2|F
 *      =50;
 *
 * Parameters
 * ----------
 * motorId   - 1: scope, 2: base
 * direction - 1: clockwise, 2: anti-clockwise
 * format    - 1: steps, 2: degrees
 *
 * Response Codes
 * --------------
 * 00 - OK
 * 01 - (App) Device closed
 * 02 - (App) Command timeout
 * 03 - (App) Flush error
 * 40 - Missing : before command
 * 44 - Invalid command
 * 45 - Invalid motorId
 * 46 - Invalid direction
 * 47 - Invalid steps/degrees
 * 48 - Invalid speed/acceleration
 * 49 - Invalid other parameter
 * 50 - Orientation unavailable
 *
 * States
 * ------
 * b00 (decimal 0) - ready for command
 * b01 (decimal 1) - running command
 * b10 (decimal 2) - [unassigned]
 * b11 (decimal 3) - [unassigned]
 */

// TODO:
//  - have separate maxAcceleration values for m1 and m2
//  - write real encoder/lcd interface

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <RotaryEncoder.h>
#include <LiquidCrystal_I2C.h>
#include "dwf/digitalWriteFast.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

/******************************************/
/* Features/Hardware Enable               */
/* ****************************************/
// Whether the limit switches are connected
#define limitsEnabled_m1 true
#define limitsEnabled_m2 true
// Rotary Encoder
#define encoderEnabled false
// LCD Display
#define lcdEnabled false
// Orientation Sensor
#define orientationEnabled true
// Stop Signal pin
#define stopPinEnabled true
/******************************************/

#define baudRate 9600L

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
#define maxSpeed_m1 1800L
#define maxSpeed_m2 1800L
#define degreesPerStep_m1 0.0008125
#define degreesPerStep_m2 0.001125
// for homing, will not overshoot limit switches
#define maxDegrees_m1 190
#define maxDegrees_m2 380

#define maxAcceleration 10000L
#define motorSleepTimeout 2000L

// limit switch states
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

// the position is only meaningful if homed
boolean hasHomed_m1 = false;
boolean hasHomed_m2 = false;

// Rotary Encoder

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

// https://github.com/fmalpartida/New-LiquidCrystal
LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified module

boolean displaySleepEnabled = true;
unsigned long displaySleepTimeout = 10000L;
unsigned long lastDisplayActionTime = millis();
volatile boolean displayUpdateNeeded = false;
boolean isDisplayLightOn = false;

// Orientation sensor

// https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
Adafruit_BNO055 bno = Adafruit_BNO055(55);
// whether it is initialized properly
boolean isOrientation = false;
boolean isOrientationCalibrated = false;
// latest read
float orientation_x;
float orientation_y;
float orientation_z;
// calibration
// 
uint8_t cal_system;
uint8_t cal_gyro;
uint8_t cal_accel;
uint8_t cal_mag;


// State output
#define statePin1 A0
#define statePin2 A1
#define STATE_READY 0
#define STATE_BUSY 1

// Stop/cancel
#define stopPin 13
boolean shouldStop = false;

void setup() {
  setupStatePins();
  setState(STATE_BUSY);
  Serial.begin(baudRate);
  setupMotors();
  setupEncoder();
  setupDisplay();
  setupOrientation();
  setupStopPin();
  setState(STATE_READY);
}

void loop() {

  readLimitSwitches();
  readStopPin();

  static int pos = 0;

  if (encoderEnabled) {
    int newPos = encoder.getPosition();
  
    if (pos != newPos) {
      if (shouldTakeInput()) {
        takeKnobInput(pos, newPos);
      }
      // if we are not taking input, this will ignore changes to knob position
      pos = newPos;
    }
  }

  if (runMotorsIfNeeded()) {
    setState(STATE_BUSY);
  } else {
    updateDisplay();
    checkDisplaySleep();
    checkMotorSleep();
    readOrientation();
    setState(STATE_READY);
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

  // ignore trailing \n
  if (firstByte == '\n') {
    return;
  }
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
      output.write("=45\n");
      return;
    }

    // second param is direction 1: clockwise, 2: anti-clockwise

    int dir = input.readStringUntil(' ').toInt();

    int dirMult = getDirMultiplier(dir);
    if (dirMult == 0) {
      output.write("=46\n");
      return;
    }

    // third param is how many steps

    long howMuch = input.readStringUntil(';').toInt() * dirMult;
    if (howMuch == 0) {
      output.write("=47\n");
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
      output.write("=45\n");
      return;
    }

    // second param is the speed
    long newSpeed = input.readStringUntil(';').toInt();
    if (newSpeed == 0) {
      output.write("=48\n");
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
      output.write("=45\n");
      return;
    }

    // second param is the acceleration
    long newAccel = input.readStringUntil(';').toInt();
    if (newAccel == 0) {
      output.write("=48\n");
      return;
    }

    setAcceleration(motorId, newAccel);

    output.write("=00\n");

  } else if (command.equals("04")) {
    // Move a motor n degrees in a direction

    // first param is the motor id

    int motorId = readMotorIdFromInput(input);
    if (motorId == 0) {
      output.write("=45\n");
      return;
    }

    // second param is direction 1: clockwise, 2: anti-clockwise

    int dir = input.readStringUntil(' ').toInt();

    int dirMult = getDirMultiplier(dir);
    if (dirMult == 0) {
      output.write("=46\n");
      return;
    }

    // third param is how many degrees

    float howMuch = input.readStringUntil(';').toFloat() * dirMult;
    if (howMuch == 0) {
      output.write("=47\n");
      return;
    }

    // perform action

    jumpOneByDegrees(motorId, howMuch);

    output.write("=00\n");

  } else if (command.equals("05")) {

    // read limit switch and stop pin states

    input.readStringUntil(';');

    char states[7] = {
      isLimitCw_m1  ? 'T' : 'F',
      isLimitAcw_m1 ? 'T' : 'F',
      isLimitCw_m2  ? 'T' : 'F',
      isLimitAcw_m2 ? 'T' : 'F',
      '|',
      shouldStop ? 'T' : 'F'
    };

    output.write("=00;");
    output.write(states);
    output.write("\n");

  } else if (command.equals("06")) {

    // home a single motor

    // param is the motor id
    int motorId = readMotorIdFromInput(input);

    if (motorId == 0) {
      output.write("=45\n");
      return;
    }

    input.readStringUntil(';');

    if (!motorCanHome(motorId)) {
      output.write("=47\n");
      return;
    }

    // perform action
    homeMotor(motorId);

    output.write("=00\n");

  } else if (command.equals("07")) {

    // home both motors

    input.readStringUntil(';');

    if (!motorCanHome(1) && !motorCanHome(2)) {
      output.write("=47\n");
      return;
    }

    // perform action
    if (motorCanHome(1)) {
      homeMotor(1);
    }
    if (motorCanHome(2)) {
      homeMotor(2);
    }

    output.write("=00\n");

  } else if (command.equals("08")) {

    // end a single motor

    // param is the motor id
    int motorId = readMotorIdFromInput(input);

    if (motorId == 0) {
      output.write("=45\n");
      return;
    }

    input.readStringUntil(';');

    if (!motorCanHome(motorId)) {
      output.write("=47\n");
      return;
    }

    // perform action
    endMotor(motorId);

    output.write("=00\n");

  } else if (command.equals("09")) {

    // end both motors

    input.readStringUntil(';');

    if (!motorCanHome(1) && !motorCanHome(2)) {
      output.write("=47\n");
      return;
    }

    // perform action
    if (motorCanHome(1)) {
      endMotor(1);
    }
    if (motorCanHome(2)) {
      endMotor(2);
    }

    output.write("=00\n");

  } else if (command.equals("10")) {

    // move both motors by steps

    // first param is direction_1
    int dir1 = input.readStringUntil(' ').toInt();

    int dirMult1 = getDirMultiplier(dir1);
    if (dirMult1 == 0) {
      output.write("=46\n");
      return;
    }

    // second param is steps_1
    long howMuch1 = input.readStringUntil(' ').toInt() * dirMult1;
    if (howMuch1 == 0) {
      output.write("=47\n");
      return;
    }

    // third param is direction_2
    int dir2 = input.readStringUntil(' ').toInt();

    int dirMult2 = getDirMultiplier(dir2);
    if (dirMult2 == 0) {
      output.write("=46\n");
      return;
    }

    // fourth param is steps_2
    long howMuch2 = input.readStringUntil(';').toInt() * dirMult2;
    if (howMuch2 == 0) {
      output.write("=47\n");
      return;
    }

    // perform action
    jumpOne(1, howMuch1);
    jumpOne(2, howMuch2);

    output.write("=00\n");

  } else if (command.equals("11")) {

    // move both motors by degrees

    // first param is direction_1
    int dir1 = input.readStringUntil(' ').toInt();

    int dirMult1 = getDirMultiplier(dir1);
    if (dirMult1 == 0) {
      output.write("=46\n");
      return;
    }

    // second param is degrees_1
    float howMuch1 = input.readStringUntil(' ').toFloat() * dirMult1;
    if (howMuch1 == 0) {
      output.write("=47\n");
      return;
    }

    // third param is direction_2
    int dir2 = input.readStringUntil(' ').toInt();

    int dirMult2 = getDirMultiplier(dir2);
    if (dirMult2 == 0) {
      output.write("=46\n");
      return;
    }

    // fourth param is degrees_2
    float howMuch2 = input.readStringUntil(';').toFloat() * dirMult2;
    if (howMuch2 == 0) {
      output.write("=47\n");
      return;
    }

    // perform action
    jumpOneByDegrees(1, howMuch1);
    jumpOneByDegrees(2, howMuch2);

    output.write("=00\n");

  } else if (command.equals("12")) {

    // get motor positions

    // param is format
    int format = input.readStringUntil(';').toInt();

    if (format != 1 && format != 2) {
      output.write("=49\n");
      return;
    }

    output.write("=00;");

    writePositions(output, format);


    output.write("\n");

  } else if (command.equals("13")) {
    // no response
    input.readStringUntil(';');

  } else if (command.equals("14")) {

    // get orientation

    input.readStringUntil(';');

    if (!isOrientation) {
      output.write("=50;\n");
      return;
    }

    readOrientation();

    output.write("=00;");

    writeOrientation(output);

    output.write("\n");
  } else if (command.equals("15")) {
    // get position by degrees, followed by orientation
    input.readStringUntil(';');
    readOrientation();
    output.write("=00;");
    writePositions(output, 2);
    output.write('|');
    writeOrientation(output);
    output.write("\n");
  } else if (command.equals("16")) {
    // get orientation calibration status
    input.readStringUntil(';');

    if (!isOrientation) {
      output.write("=50;\n");
      return;
    }

    readOrientation();

    output.write("=00;");

    writeString(output, String(cal_system));
    output.write('|');
    writeString(output, String(cal_gyro));
    output.write('|');
    writeString(output, String(cal_accel));
    output.write('|');
    writeString(output, String(cal_mag));
    output.write('|');
    output.write(isOrientationCalibrated ? 'T' : 'F');
    output.write("\n");

  } else {
    output.write("=44\n");
  }
}

void writePositions(Stream &output, int format) {
  if (hasHomed_m1) {
    long mpos = stepper_m1.currentPosition();
    writeString(output, String(format == 1 ? mpos : (mpos * degreesPerStep_m1)));
  } else {
    output.write("?");
  }

  output.write("|");

  if (hasHomed_m2) {
    long mpos = stepper_m2.currentPosition();
    writeString(output, String(format == 1 ? mpos : (mpos * degreesPerStep_m2)));
  } else {
    output.write("?");
  }
}

void writeOrientation(Stream &output) {
  if (isOrientation) {
    writeString(output, String(orientation_x, 4));
    output.write('|');
    writeString(output, String(orientation_y, 4));
    output.write('|');
    writeString(output, String(orientation_z, 4));
  } else {
    output.write("?|?|?");
  }
  output.write('|');
  output.write(isOrientationCalibrated ? 'T' : 'F');
}

void writeString(Stream &output, String s) {
  for (int i = 0; i < s.length(); i++) {
    output.write(s.charAt(i));
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

void readLimitSwitches() {

  if (limitsEnabled_m1) {
    isLimitCw_m1  = digitalReadFast(limitPinCw_m1)  == HIGH;
    isLimitAcw_m1 = digitalReadFast(limitPinAcw_m1) == HIGH;
  }

  if (limitsEnabled_m2) {
    isLimitCw_m2  = digitalReadFast(limitPinCw_m2)  == HIGH;
    isLimitAcw_m2 = digitalReadFast(limitPinAcw_m2) == HIGH;
  }
}

// the howMuch is just a positive/negative direction reference.
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

boolean motorCanHome(int motorId) {
  if (motorId == 1) {
    return limitsEnabled_m1;
  }
  if (motorId == 2) {
    return limitsEnabled_m2;
  }
}

boolean isMotorHome(int motorId) {
  if (!motorCanHome(motorId)) {
    return false;
  }
  if (motorId == 1) {
    return isLimitAcw_m1;
  } else if (motorId == 2) {
    return isLimitAcw_m2;
  }
}

float getMaxDegreesForMotor(int motorId) {
  if (motorId == 1) {
    return maxDegrees_m1;
  } else if (motorId == 2) {
    return maxDegrees_m2;
  }
}

void homeMotor(int motorId) {
  if (!motorCanHome(motorId)) {
    return;
  }
  if (isMotorHome(motorId)) {
    // move forward just a little
    jumpOneByDegrees(motorId, 1.5);
    while (runMotorsIfNeeded()) {
      readLimitSwitches();
      // TODO: fix physical tab for M2 optical switches. This code works
      //       fine, and it's better than moving a full 1.5 degrees.
      if (!isMotorHome(motorId)) {
        //stopMotor(motorId);
      }
    }
    readLimitSwitches();
  }
  jumpOneByDegrees(motorId, -1 * getMaxDegreesForMotor(motorId));
}

void endMotor(int motorId) {
  if (!motorCanHome(motorId)) {
    return;
  }
  jumpOneByDegrees(motorId, getMaxDegreesForMotor(motorId));
}

boolean runMotorsIfNeeded() {
  
  boolean isRun = false;
  if (stepper_m1.distanceToGo() != 0) {
    if (shouldStop || !motorCanMove(1, stepper_m1.distanceToGo())) {
      stopMotor(1);
    } else {
      stepper_m1.run();
    }
    registerMotorAction();
    isRun = true;
  }
  if (stepper_m2.distanceToGo() != 0) {
    if (shouldStop || !motorCanMove(2, stepper_m2.distanceToGo())) {
      stopMotor(2);
    } else {
      stepper_m2.run();
    }
    registerMotorAction();
    isRun = true;
  }

  return isRun;
}

void stopMotor(int motorId) {
  if (motorId == 1) {
    long oldAcceleration_m1 = acceleration_m1;
    setAcceleration(1, maxAcceleration);
    stepper_m1.stop();
    while (stepper_m1.distanceToGo() != 0) {
      stepper_m1.run();
    }
    setAcceleration(1, oldAcceleration_m1);
    if (!shouldStop) {
      // we have reached a limit switch, see if we are home
      if (isMotorHome(1)) {
        hasHomed_m1 = true;
        stepper_m1.setCurrentPosition(0);
      }
    }
  } else if (motorId == 2) {
    long oldAcceleration_m2 = acceleration_m2;
    setAcceleration(2, maxAcceleration);
    stepper_m2.stop();
    while (stepper_m2.distanceToGo() != 0) {
      stepper_m2.run();
    }
    setAcceleration(2, oldAcceleration_m2);
    if (!shouldStop) {
    // we have reached a limit switch, see if we are home
      if (isMotorHome(2)) {
        hasHomed_m2 = true;
        stepper_m2.setCurrentPosition(0);
      }
    }
  }
}

void jumpBoth(long howMuch) {
  jumpOne(1, howMuch);
  jumpOne(2, howMuch);
}

void jumpBothByDegrees(float howMuch) {
  jumpOneByDegrees(1, howMuch);
  jumpOneByDegrees(2, howMuch);
}

void jumpOne(int motorId, long howMuch) {
  setState(STATE_BUSY);
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
  setState(STATE_BUSY);
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
// Stop pin functions
// ----------------------------------------------
void readStopPin() {
  if (stopPinEnabled) {
    shouldStop = digitalReadFast(stopPin) == HIGH;
  }
}

// ----------------------------------------------
// State pin functions
// ----------------------------------------------
void setState(byte state) {
  if (state == 0) {
    digitalWrite(statePin1, LOW);
    digitalWrite(statePin2, LOW);
  } else if (state == 1) {
    digitalWrite(statePin1, HIGH);
    digitalWrite(statePin2, LOW);
  } else if (state == 2) {
    digitalWrite(statePin1, LOW);
    digitalWrite(statePin2, HIGH);
  } else if (state == 3) {
    digitalWrite(statePin1, HIGH);
    digitalWrite(statePin2, HIGH);
  }
}

// ----------------------------------------------
// Rotary Encoder functions
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
  if (lcdEnabled && shouldUpdateDisplay()) {
    lcd.setCursor(0, 0);
    lcd.print("Program: ");
    lcd.print(programNum);
    displayUpdateNeeded = false;
  }
}

boolean shouldUpdateDisplay() {
  return lcdEnabled && displayUpdateNeeded;
}

void turnOnDisplayLight() {
  if (lcdEnabled) {
    lcd.setBacklightPin(3, NEGATIVE);
  }
}

void turnOffDisplayLight() {
  if (lcdEnabled) {
    lcd.setBacklightPin(3, POSITIVE);
  }
}

void registerDisplayAction() {
  lastDisplayActionTime = millis();
}

void checkDisplaySleep() {
  if (!lcdEnabled || !displaySleepEnabled) {
    return;
  }
  unsigned long elapsed = millis() - lastDisplayActionTime;
  if (elapsed > displaySleepTimeout) {
    turnOffDisplayLight();
  } else {
    // for button, so we don't have to do this in an interrupt
    turnOnDisplayLight();
  }
}

// ----------------------------------------------
// Orientation functions
// ----------------------------------------------

void readOrientation() {
  if (!isOrientation) {
    return;
  }
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);

  orientation_x = event.orientation.x;
  orientation_y = event.orientation.y;
  orientation_z = event.orientation.z;
  if (!isOrientationCalibrated) {
    bno.getCalibration(&cal_system, &cal_gyro, &cal_accel, &cal_mag);
    if (cal_system == 3 && cal_gyro == 3 && cal_accel == 3 && cal_mag == 3) {
      isOrientationCalibrated = true;
    }
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
  if (!lcdEnabled) {
    return;
  }
  lcd.begin(16, 2);
  lcd.clear();
  displayUpdateNeeded = true;
  turnOnDisplayLight();
  updateDisplay();
}

void setupEncoder() {

  if (!encoderEnabled){
    return;
  }

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
  if (encoderEnabled) {
    encoder.tick(); // just call tick() to check the state.
  }
}

void setupOrientation() {
  cal_system = cal_gyro = cal_accel = cal_mag = 0;
  if (!orientationEnabled) {
    return;
  }
  if (bno.begin()) {
    isOrientation = true;
    bno.setExtCrystalUse(true);
  }
}

void setupStatePins() {
  pinMode(statePin1, OUTPUT);
  pinMode(statePin2, OUTPUT);
}

void setupStopPin() {
  pinMode(stopPin, INPUT);
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
