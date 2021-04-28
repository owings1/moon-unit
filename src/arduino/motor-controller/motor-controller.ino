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
 *    - <motors[1].maxSpeed>
 *    - <motors[0].acceleration>
 *    - <motors[1].acceleration>
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
 * 49 - Invalid other
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
#define stepPin_m1 6
#define enablePin_m1 7
#define limitPin_m1_cw 3
#define limitPin_m1_acw 4

#define dirPin_m2 8
#define stepPin_m2 9
#define enablePin_m2 10
#define limitPin_m2_cw 11
#define limitPin_m2_acw 12


#define absMaxSpeed_m1 1600L
#define absMaxSpeed_m2 1600L
// TODO: refactor, since floats are only good to 6 decimal places
#define degreesPerStep_m1 0.0008125
#define degreesPerStep_m2 0.001125

#define maxDegrees_m1 190
#define maxDegrees_m2 380

#define maxAcceleration 10000L
#define motorSleepTimeout 2000L


// Stop signal
boolean shouldStop = false;

struct Motor {

  AccelStepper stepper;

  byte id;

  // pins
  byte enablePin;
  byte dirPin;
  byte stepPin;
  byte limitPin_cw;
  byte limitPin_acw;

  // limit switches can be disabled via command.
  boolean limitsEnabled;

  // limit switch states
  boolean isLimit_cw;
  boolean isLimit_acw;

  // whether the motor is on
  boolean isActive;

  // the position is only meaningful if homed
  boolean hasHomed;

  // flag to reset acceleration to oldAcceleration after motors are finished
  // running, for smooth stop on limits.
  boolean isStopping;

  // flag for when we are backing up for homing purposes, so that immediately
  // after we can re-initiate homing.
  boolean isBacking;

  // track the max speed and acceleration values set in stepper objects.
  unsigned long acceleration;
  unsigned long maxSpeed;
  // for temporarily overriding acceleration during stopping.
  unsigned long oldAcceleration;

  // for motor sleep
  unsigned long lastActionTime;

  // for homing, will try not to overshoot limits
  float maxDegrees;
  unsigned long absMaxSpeed;
  float degreesPerStep;
};

Motor motors[2];

void setup() {
  setupStatePin();
  setState(STATE_BUSY);
  Serial.begin(baudRate);
  setupMotors();
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

    byte motorId = readMotorIdFromInput(input);
    if (motorId == 0) {
      output.write("=45\n");
      return;
    }

    // second param is direction 1: clockwise, 2: anti-clockwise

    byte dir = input.readStringUntil(' ').toInt();

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

    byte motorId = readMotorIdFromInput(input);
    if (motorId == 0) {
      output.write("=45\n");
      return;
    }

    // second param is the speed
    unsigned long newSpeed = input.readStringUntil(';').toInt();
    if (newSpeed == 0) {
      output.write("=48\n");
      return;
    }

    setMaxSpeed(motorId, newSpeed);

    output.write("=00\n");

  } else if (command.equals("03")) {

    // Set acceleration for motor

    // first param is the motor id

    byte motorId = readMotorIdFromInput(input);
    if (motorId == 0) {
      output.write("=45\n");
      return;
    }

    // second param is the acceleration
    unsigned long newAccel = input.readStringUntil(';').toInt();
    if (newAccel == 0) {
      output.write("=48\n");
      return;
    }

    setAcceleration(motorId, newAccel);

    output.write("=00\n");

  } else if (command.equals("04")) {
    // Move a motor n degrees in a direction

    // first param is the motor id

    byte motorId = readMotorIdFromInput(input);
    if (motorId == 0) {
      output.write("=45\n");
      return;
    }

    // second param is direction 1: clockwise, 2: anti-clockwise

    byte dir = input.readStringUntil(' ').toInt();

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
    byte motorId = readMotorIdFromInput(input);

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
    byte motorId = readMotorIdFromInput(input);

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
    byte dir1 = input.readStringUntil(' ').toInt();

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
    byte dir2 = input.readStringUntil(' ').toInt();

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
    byte dir1 = input.readStringUntil(' ').toInt();

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
    byte dir2 = input.readStringUntil(' ').toInt();

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
    byte motorId = readMotorIdFromInput(input);

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

    // perform action
    motors[motorId - 1].limitsEnabled = flag == 'T';

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
    
    output.write(motors[0].limitsEnabled ? 'T' : 'F');
    output.write('|');
    output.write(motors[1].limitsEnabled ? 'T' : 'F');
    output.write('|');

    // <degreesPerStep_m1>
    // <degreesPerStep_m2>
    output.print(motors[0].degreesPerStep, 8);
    output.write('|');
    output.print(motors[1].degreesPerStep, 8);
    output.write('|');

    // <maxSpeed_m1>
    // <motors[1].maxSpeed>
    output.print(motors[0].maxSpeed);
    output.write('|');
    output.print(motors[1].maxSpeed);
    output.write('|');

    // <motors[0].acceleration>
    // <motors[1].acceleration>
    output.print(motors[0].acceleration);
    output.write('|');
    output.print(motors[1].acceleration);
    output.write('|');

    // limit states: <m1_cw><m1_acw><m2_cw><m2_acw>
    // <shouldStop>
    char states[7] = {
      motors[0].isLimit_cw  ? 'T' : 'F',
      motors[0].isLimit_acw ? 'T' : 'F',
      motors[1].isLimit_cw  ? 'T' : 'F',
      motors[1].isLimit_acw ? 'T' : 'F',
      '|',
      shouldStop ? 'T' : 'F'
    };
    output.write(states);

    output.write("\n");
  } else {
    output.write("=44\n");
  }
}

void writePositions(Stream &output, byte format) {
  for (byte i = 0; i < 2; i++) {
    if (motors[i].hasHomed) {
      long mpos = motors[i].stepper.currentPosition();
      output.print(String(format == 1 ? mpos : (mpos * motors[i].degreesPerStep)));
    } else {
      output.print(format == 1 ? STEPS_NULL : DEG_NULL);
    }
    if (i == 0) {
      output.write('|');
    }
  }
}

byte readMotorIdFromInput(Stream &input) {
  byte motorId = input.readStringUntil(' ').toInt();
  if (motorId == 1 || motorId == 2) {
    return motorId;
  }
  return 0;
}

int getDirMultiplier(byte dirInput) {
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

// the howMuch is just a positive/negative direction reference.
boolean motorCanMove(byte motorId, long howMuch) {
  byte i = motorId - 1;
  if (!motors[i].limitsEnabled) {
    return true;
  }
  if (howMuch > 0) {
    return !motors[i].isLimit_cw;
  } else {
    return !motors[i].isLimit_acw;
  }
}

// returns DEG_NULL if motor has not homed.
float getMotorPositionDegrees(byte motorId) {
  byte i = motorId - 1;
  if (motors[i].hasHomed) {
    return motors[i].stepper.currentPosition() * motors[i].degreesPerStep;
  }
  return DEG_NULL;
}

boolean motorCanHome(byte motorId) {
  return motors[motorId - 1].limitsEnabled;
}

boolean isMotorHome(byte motorId) {
  if (!motorCanHome(motorId)) {
    return false;
  }
  return motors[motorId - 1].isLimit_acw;
}

float getMaxDegreesForMotor(byte motorId) {
  return motors[motorId - 1].maxDegrees;
}

void homeMotor(byte motorId) {
  if (!motorCanHome(motorId)) {
    return;
  }
  if (isMotorHome(motorId)) {
    // move forward just a little
    motors[motorId - 1].isBacking = true;
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

void endMotor(byte motorId) {
  if (!motorCanHome(motorId)) {
    return;
  }
  jumpOneByDegrees(motorId, getMaxDegreesForMotor(motorId));
}

boolean runMotorsIfNeeded() {
  
  boolean isRun = false;

  for (byte i = 0; i < 2; i++) {
    //Motor motor = motors[i];
    if (motors[i].stepper.distanceToGo() != 0) {
      // this will move at most one step
      motors[i].stepper.run();
      if (shouldStop || !motorCanMove(motors[i].id, motors[i].stepper.distanceToGo())) {
        stopMotor(motors[i].id);
      }
      registerMotorAction(motors[i].id);
      isRun = true;
    } else if (motors[i].isStopping) {
      // we have finished stopping
      motors[i].isStopping = false;
      setAcceleration(motors[i].id, motors[i].oldAcceleration);
      if (!shouldStop) {
        // we have reached a limit switch, see if we are home
        if (isMotorHome(motors[i].id)) {
          motors[i].hasHomed = true;
          motors[i].stepper.setCurrentPosition(0);
        }
      }
    } else if (motors[i].isBacking) {
      // we have finished backing for home
      motors[i].isBacking = false;
      homeMotor(motors[i].id);
    }
  }

  return isRun;
}

void stopMotor(byte motorId) {
  byte i = motorId - 1;
  if (motors[i].isStopping) {
    // don't duplicate action
    return;
  }
  motors[i].isStopping = true;
  motors[i].oldAcceleration = motors[i].acceleration;
  setAcceleration(motorId, maxAcceleration);
  motors[i].stepper.stop();
}

void jumpBoth(long howMuch) {
  jumpOne(1, howMuch);
  jumpOne(2, howMuch);
}

void jumpBothByDegrees(float howMuch) {
  jumpOneByDegrees(1, howMuch);
  jumpOneByDegrees(2, howMuch);
}

void jumpOne(byte motorId, long howMuch) {
  setState(STATE_BUSY);
  if (motorCanMove(motorId, howMuch)) {
    motors[motorId - 1].stepper.move(howMuch);
    enableMotor(motorId);
  }
}

void jumpOneByDegrees(byte motorId, float howMuch) {
  setState(STATE_BUSY);
  byte i = motorId - 1;
  long steps = howMuch / motors[i].degreesPerStep;
  if (motorCanMove(motorId, steps)) {
    motors[i].stepper.move(steps);
    enableMotor(motorId);
  }
}

void setMaxSpeed(byte motorId, unsigned long value) {
  byte i = motorId - 1;
  motors[i].maxSpeed = min(value, motors[i].absMaxSpeed);
  motors[i].stepper.setMaxSpeed(motors[i].maxSpeed);
}

void setAcceleration(byte motorId, unsigned long value) {
  byte i = motorId - 1;
  motors[i].acceleration = min(value, maxAcceleration);
  motors[i].stepper.setAcceleration(motors[i].acceleration);
}

void enableMotor(byte motorId) {
  byte i = motorId - 1;
  if (!motors[i].isActive) {
    digitalWrite(motors[i].enablePin, LOW);
    motors[i].isActive = true;
    delay(2);
  }
  registerMotorAction(motorId);
}

void disableMotor(byte motorId) {
  byte i = motorId - 1;
  if (motors[i].isActive) {
    digitalWrite(motors[i].enablePin, HIGH);
    motors[i].isActive = false;
  }
}

void disableMotors() {
  disableMotor(1);
  disableMotor(2);
}

void registerMotorAction(int motorId) {
  motors[motorId - 1].lastActionTime = millis();
}

void checkMotorsSleep() {
  for (byte i = 0; i < 2; i++) {
    unsigned long elapsed = millis() - motors[i].lastActionTime;
    if (elapsed > motorSleepTimeout) {
      disableMotor(motors[i].id);
    }
  }
}

void readLimitSwitches() {
  for (byte i = 0; i < 2; i++) {
    motors[i].isLimit_cw = digitalReadFast(motors[i].limitPin_cw) == LOW;
    motors[i].isLimit_acw = digitalReadFast(motors[i].limitPin_acw) == LOW;
  }
}

// ----------------------------------------------
// Stop pin functions
// ----------------------------------------------
void readStopPin() {
  shouldStop = stopPinEnabled && (digitalReadFast(stopPin) == HIGH);
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
 
  motors[0].enablePin = enablePin_m1;
  motors[0].stepPin = stepPin_m1;
  motors[0].dirPin = dirPin_m1;
  motors[0].limitPin_cw = limitPin_m1_cw;
  motors[0].limitPin_acw = limitPin_m1_acw;
  motors[0].degreesPerStep = degreesPerStep_m1;
  motors[0].maxDegrees = maxDegrees_m1;
  motors[0].absMaxSpeed = absMaxSpeed_m1;

  motors[1].enablePin = enablePin_m2;
  motors[1].stepPin = stepPin_m2;
  motors[1].dirPin = dirPin_m1;
  motors[1].limitPin_cw = limitPin_m2_cw;
  motors[1].limitPin_acw = limitPin_m2_acw;
  motors[1].degreesPerStep = degreesPerStep_m2;
  motors[1].maxDegrees = maxDegrees_m2;
  motors[1].absMaxSpeed = absMaxSpeed_m2;

  for (byte i = 0; i < 2; i++) {

    // Declare pins as output:
    pinMode(motors[i].stepPin, OUTPUT);
    pinMode(motors[i].dirPin, OUTPUT);
    pinMode(motors[i].enablePin, OUTPUT);
    // Declare limit switch pins as input
    pinMode(motors[i].limitPin_cw, INPUT);
    pinMode(motors[i].limitPin_acw, INPUT);

    motors[i].stepper = AccelStepper(AccelStepper::FULL2WIRE, motors[i].stepPin, motors[i].dirPin);
    motors[i].id = i + 1;
    motors[i].limitsEnabled = true;
    motors[i].lastActionTime = millis();

    // set initial state of motor to disabled
    digitalWrite(motors[i].enablePin, HIGH);
    motors[i].isActive = false;

    // step max speed & acceleration
    setMaxSpeed(motors[i].id, motors[i].absMaxSpeed);
    setAcceleration(motors[i].id, maxAcceleration);
  }
}

void setupStatePin() {
  pinMode(statePin, OUTPUT);
}

void setupStopPin() {
  pinMode(stopPin, INPUT);
}
