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
 * 13 - No response (debug)
 *
 *  :13 ;
 *
 * 17 - Set limit switch enablement for a motor
 *
 *  :17 <motorId> <T|F>;
 *
 * 18 - Get full status
 *
 *  :18 ;
 *
 *    values separated by |:
 *
 *    - <position_m1_degrees>
 *    - <position_m2_degrees>
 *    - <limitsEnabled_m1>
 *    - <limitsEnabled_m2>
 *    - <degreesPerStep_m1>
 *    - <degreesPerStep_m2>
 *    - <maxSpeed_m1>
 *    - <maxSpeed_m2>
 *    - <acceleration_m1>
 *    - <acceleration_m2>
 *    - limit states: <m1_cw><m1_acw><m2_cw><m2_acw>
 *    - <shouldStop>
 *
 * ===================================================
 *
 * Parameters
 * ----------
 * motorId   - 1: scope, 2: base
 * direction - 1: clockwise, 2: anti-clockwise
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
 *
 * State/Ready pin
 * ------
 * HIGH - ready
 * LOW  - busy
 */
#include <AccelStepper.h>
#include <MultiStepper.h>
#include "dwf/digitalWriteFast.h"

/******************************************/
/* Features/Hardware Enable               */
/* ****************************************/

// Stop Signal pin
#define stopPinEnabled true
/******************************************/

#define baudRate 9600L
#define DEG_NULL 1000.00
#define STEPS_NULL -1L
#define STATE_READY HIGH
#define STATE_BUSY LOW

/////////////////////
// Pins            //
/////////////////////

#define statePin A0
#define stopPin 13

#define dirPin_m1 5
#define dirPin_m2 8
#define stepPin_m1 6
#define stepPin_m2 9
#define enablePin_m1 7
#define enablePin_m2 10
#define limitPin_m1_cw 3
#define limitPin_m1_acw 4
#define limitPin_m2_cw 11
#define limitPin_m2_acw 12

#define absMaxSpeed_m1 1600L
#define absMaxSpeed_m2 1600L
// TODO: refactor, since floats are only good to 6 decimal places
#define degreesPerStep_m1 0.0008125
#define degreesPerStep_m2 0.001125
// for homing, will try not to overshoot limit switches
#define maxDegrees_m1 190
#define maxDegrees_m2 380

#define maxAcceleration 10000L
#define motorSleepTimeout 2000L

// limit switches can be disabled during runtime.
boolean limitsEnabled_m1 = true;
boolean limitsEnabled_m2 = true;
// limit switch states
boolean isLimit_m1_cw = false;
boolean isLimit_m1_acw = false;
boolean isLimit_m2_cw = false;
boolean isLimit_m2_acw = false;

// track the max speed and acceleration values set in stepper objects.
unsigned long acceleration_m1;
unsigned long acceleration_m2;
unsigned long maxSpeed_m1;
unsigned long maxSpeed_m2;
// flag to reset acceleration to oldAcceleration after motors are finished
// running, for smooth stop on limits.
boolean isStopping_m1 = false;
boolean isStopping_m2 = false;
unsigned long oldAcceleration_m1;
unsigned long oldAcceleration_m2;
// flag for when we are backing up for homing purposes, so that immediately
// after we can re-initiate homing.
boolean isBacking_m1 = false;
boolean isBacking_m2 = false;

unsigned long lastMotorActionTime_m1 = millis();
unsigned long lastMotorActionTime_m2 = millis();
boolean isMotorActive_m1 = false;
boolean isMotorActive_m2 = false;

// the position is only meaningful if homed
boolean hasHomed_m1 = false;
boolean hasHomed_m2 = false;

// Stop signal
boolean shouldStop = false;

AccelStepper stepper_m1(AccelStepper::FULL2WIRE, stepPin_m1, dirPin_m1);
AccelStepper stepper_m2(AccelStepper::FULL2WIRE, stepPin_m2, dirPin_m2);


struct Motor {
  AccelStepper stepper;
  boolean limitsEnabled;
  boolean isLimit_cw;
  boolean isLimit_acw;
  boolean isActive;
  boolean hasHomed;
  boolean isStopping;
  boolean isBacking;
  unsigned long acceleration;
  unsigned long oldAcceleration;
  unsigned long maxSpeed;
  unsigned long lastActionTime;
};

Motor motors[2];

void setup() {
  setupStatePin();
  setState(STATE_BUSY);
  Serial.begin(baudRate);
  setupMotors();
  setupMotorsOld();
  setupStopPin();
  setState(STATE_READY);
}

void loop() {

  readLimitSwitches();
  readStopPin();

  if (runMotorsIfNeeded()) {
    setState(STATE_BUSY);
  } else {
    checkMotorsSleep();
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

    setMaxSpeed(motorId, newSpeed);

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

  } else if (command.equals("13")) {
    // no response
    input.readStringUntil(';');

  } else if (command.equals("17")) {

    // Set limit switch enablement for a motor

    // first param is the motor id
    int motorId = readMotorIdFromInput(input);

    if (motorId == 0) {
      output.write("=45\n");
      return;
    }
    // last param is T/F
    char flag = input.readStringUntil(';').charAt(0);
    if (flag != 'T' && flag != 'F') {
      output.write("=49\n");
      return;
    }
    if (motorId == 1) {
      limitsEnabled_m1 = flag == 'T';
    } else if (motorId == 2) {
      limitsEnabled_m2 = flag == 'T';
    }

    output.write("=00\n");

  } else if (command.equals("18")) {

    // get full status

    input.readStringUntil(';');

    output.write("=00;");

    // <position_m1_degrees>
    // <position_m2_degrees>
    writePositions(output, 2);
    output.write('|');
    
    // <limitsEnabled_m1>
    // <limitsEnabled_m2>
    
    output.write(limitsEnabled_m1 ? 'T' : 'F');
    output.write('|');
    output.write(limitsEnabled_m2 ? 'T' : 'F');
    output.write('|');

    // <degreesPerStep_m1>
    // <degreesPerStep_m2>
    output.print(degreesPerStep_m1, 8);
    output.write('|');
    output.print(degreesPerStep_m2, 8);
    output.write('|');

    // <maxSpeed_m1>
    // <maxSpeed_m2>
    output.print(maxSpeed_m1);
    output.write('|');
    output.print(maxSpeed_m2);
    output.write('|');

    // <acceleration_m1>
    // <acceleration_m2>
    output.print(acceleration_m1);
    output.write('|');
    output.print(acceleration_m2);
    output.write('|');

    // limit states: <m1_cw><m1_acw><m2_cw><m2_acw>
    // <shouldStop>
    char states[7] = {
      isLimit_m1_cw  ? 'T' : 'F',
      isLimit_m1_acw ? 'T' : 'F',
      isLimit_m2_cw  ? 'T' : 'F',
      isLimit_m2_acw ? 'T' : 'F',
      '|',
      shouldStop ? 'T' : 'F'
    };
    output.write(states);

    output.write("\n");
  } else {
    output.write("=44\n");
  }
}

void writePositions(Stream &output, int format) {
  if (hasHomed_m1) {
    long mpos = stepper_m1.currentPosition();
    output.print(String(format == 1 ? mpos : (mpos * degreesPerStep_m1)));
  } else {
    output.print(format == 1 ? STEPS_NULL : DEG_NULL);
  }

  output.write("|");

  if (hasHomed_m2) {
    long mpos = stepper_m2.currentPosition();
    output.print(String(format == 1 ? mpos : (mpos * degreesPerStep_m2)));
  } else {
    output.print(format == 1 ? STEPS_NULL : DEG_NULL);
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

  isLimit_m1_cw  = digitalReadFast(limitPin_m1_cw)  == LOW;//HIGH;
  isLimit_m1_acw = digitalReadFast(limitPin_m1_acw) == LOW;//HIGH;

  isLimit_m2_cw  = digitalReadFast(limitPin_m2_cw)  == LOW;//HIGH;
  isLimit_m2_acw = digitalReadFast(limitPin_m2_acw) == LOW;//HIGH;
}

// the howMuch is just a positive/negative direction reference.
boolean motorCanMove(int motorId, long howMuch) {
  if (motorId == 1) {
    if (!limitsEnabled_m1) {
      return true;
    }
    if (howMuch > 0) {
      return !isLimit_m1_cw;
    } else {
      return !isLimit_m1_acw;
    }
  }
  if (motorId == 2) {
    if (!limitsEnabled_m2) {
      return true;
    }
    if (howMuch > 0) {
      return !isLimit_m2_cw;
    } else {
      return !isLimit_m2_acw;
    }
  }
}

// returns DEG_NULL if motor has not homed.
float getMotorPositionDegrees(int motorId) {
  if (motorId == 1) {
    if (hasHomed_m1) {
      return stepper_m1.currentPosition() * degreesPerStep_m1;
    }
  } else if (motorId == 2) {
    if (hasHomed_m2) {
      return stepper_m2.currentPosition() * degreesPerStep_m2;
    }
  }
  return DEG_NULL;
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
    return isLimit_m1_acw;
  } else if (motorId == 2) {
    return isLimit_m2_acw;
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
    if (motorId == 1) {
      isBacking_m1 = true;
    } else if (motorId == 2) {
      isBacking_m2 = true;
    }
    jumpOneByDegrees(motorId, 1.5);
    // homing will recommence after backing is complete
    return;
  }

  float degreesToMove = getMaxDegreesForMotor(motorId);
  float mposDegrees = getMotorPositionDegrees(motorId);
  // if we know position, don't way overshoot
  if (mposDegrees != DEG_NULL && mposDegrees > 0) {
    degreesToMove = mposDegrees + 10;
  }

  jumpOneByDegrees(motorId, -1 * degreesToMove);
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
    // this will move at most one step
    stepper_m1.run();
    if (shouldStop || !motorCanMove(1, stepper_m1.distanceToGo())) {
      stopMotor(1);
    }
    registerMotorAction(1);
    isRun = true;
  } else if (isStopping_m1) {
    // we have finished stopping
    isStopping_m1 = false;
    setAcceleration(1, oldAcceleration_m1);
    if (!shouldStop) {
      // we have reached a limit switch, see if we are home
      if (isMotorHome(1)) {
        hasHomed_m1 = true;
        stepper_m1.setCurrentPosition(0);
      }
    }
  } else if (isBacking_m1) {
    // we have finished backing for home
    isBacking_m1 = false;
    homeMotor(1);
  }

  if (stepper_m2.distanceToGo() != 0) {
    // this will move at most one step
    stepper_m2.run();
    if (shouldStop || !motorCanMove(2, stepper_m2.distanceToGo())) {
      stopMotor(2);
    }
    registerMotorAction(2);
    isRun = true;
  } else if (isStopping_m2) {
    // we have finished stopping
    isStopping_m2 = false;
    setAcceleration(2, oldAcceleration_m2);
    if (!shouldStop) {
    // we have reached a limit switch, see if we are home
      if (isMotorHome(2)) {
        hasHomed_m2 = true;
        stepper_m2.setCurrentPosition(0);
      }
    }
  } else if (isBacking_m2) {
    // we have finished backing for home
    isBacking_m2 = false;
    homeMotor(2);
  }

  return isRun;
}

void stopMotor(int motorId) {
  if (motorId == 1) {
    if (isStopping_m1) {
      // don't duplicate action
      return;
    }
    isStopping_m1 = true;
    oldAcceleration_m1 = acceleration_m1;
    setAcceleration(1, maxAcceleration);
    stepper_m1.stop();
  } else if (motorId == 2) {
    if (isStopping_m2) {
      // don't duplicate action
      return;
    }
    isStopping_m2 = true;
    oldAcceleration_m2 = acceleration_m2;
    setAcceleration(2, maxAcceleration);
    stepper_m2.stop();
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
    enableMotor(motorId);
  }
}

void jumpOneByDegrees(int motorId, float howMuch) {
  setState(STATE_BUSY);
  long steps;
  if (motorId == 1) {
    steps = howMuch / degreesPerStep_m1;
    if (motorCanMove(motorId, steps)) {
      stepper_m1.move(steps);
      enableMotor(motorId);
    }
  } else if (motorId == 2) {
    steps = howMuch / degreesPerStep_m2;
    if (motorCanMove(motorId, steps)) {
      stepper_m2.move(steps);
      enableMotor(motorId);
    }
  }
}

void setMaxSpeed(int motorId, long value) {
  if (motorId == 1) {
    maxSpeed_m1 = min(value, absMaxSpeed_m1);
    stepper_m1.setMaxSpeed(maxSpeed_m1);
  } else if (motorId == 2) {
    maxSpeed_m2 = min(value, absMaxSpeed_m2);
    stepper_m2.setMaxSpeed(maxSpeed_m2);
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

void enableMotor(int motorId) {
  if (motorId == 1) {
    if (!isMotorActive_m1) {
      digitalWrite(enablePin_m1, LOW);
      isMotorActive_m1 = true;
      delay(2);
    }
  } else if (motorId == 2) {
    if (!isMotorActive_m2) {
      digitalWrite(enablePin_m2, LOW);
      isMotorActive_m2 = true;
      delay(2);
    }
  }
  registerMotorAction(motorId);
}

void disableMotor(int motorId) {
  if (motorId == 1) {
    if (isMotorActive_m1) {
      digitalWrite(enablePin_m1, HIGH);
      isMotorActive_m1 = false;
    }
  } else if (motorId == 2) {
    if (isMotorActive_m2) {
      digitalWrite(enablePin_m2, HIGH);
      isMotorActive_m2 = false;
    }
  }
}

void disableMotors() {
  disableMotor(1);
  disableMotor(2);
}

void registerMotorAction(int motorId) {
  if (motorId == 1) {
    lastMotorActionTime_m1 = millis();
  } else if (motorId == 2) {
    lastMotorActionTime_m2 = millis();
  }
}

void checkMotorsSleep() {
  unsigned long elapsed_m1 = millis() - lastMotorActionTime_m1;
  unsigned long elapsed_m2 = millis() - lastMotorActionTime_m2;
  if (elapsed_m1 > motorSleepTimeout) {
    disableMotor(1);
  }
  if (elapsed_m2 > motorSleepTimeout) {
    disableMotor(2);
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
  digitalWrite(statePin, state);
}

// ----------------------------------------------
// Setup routines
// ----------------------------------------------

void setupMotors() {
  /*
   *
  AccelStepper stepper;
  boolean limitsEnabled;
  boolean isLimit_cw;
  boolean isLimit_acw;
  boolean isActive;
  boolean hasHomed;
  boolean isStopping;
  boolean isBacking;
  unsigned long acceleration;
  unsigned long oldAcceleration;
  unsigned long maxSpeed;
  unsigned long lastActionTime;
   */
  motors[0] = Motor {
    stepper_m1,
    true
  };
  motors[1] = Motor {
    stepper_m2,
    true
  };
}

void setupMotorsOld() {

  // Declare pins as output:
  pinMode(stepPin_m1, OUTPUT);
  pinMode(stepPin_m2, OUTPUT);
  pinMode(dirPin_m1, OUTPUT);
  pinMode(dirPin_m2, OUTPUT);
  pinMode(enablePin_m1, OUTPUT);
  pinMode(enablePin_m2, OUTPUT);

  // Declare limit switch pins as input
  pinMode(limitPin_m1_cw, INPUT);
  pinMode(limitPin_m1_acw, INPUT);

  // Declare limit switch pins as input
  pinMode(limitPin_m2_cw, INPUT);
  pinMode(limitPin_m2_acw, INPUT);

  // set initial state of motor to disabled
  digitalWrite(enablePin_m1, HIGH);
  digitalWrite(enablePin_m2, HIGH);
  isMotorActive_m1 = false;
  isMotorActive_m2 = false;

  // AccelStepper
  setMaxSpeed(1, absMaxSpeed_m1);
  setMaxSpeed(2, absMaxSpeed_m2);
  setAcceleration(1, maxAcceleration);
  setAcceleration(2, maxAcceleration);
}

void setupStatePin() {
  pinMode(statePin, OUTPUT);
}

void setupStopPin() {
  pinMode(stopPin, INPUT);
}
