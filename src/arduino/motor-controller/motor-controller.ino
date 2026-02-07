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
 * 10 - Move both motors by steps. Last param is arrive at same time.
 *
 *  :10 <direction_1> <steps_1> <direction_2> <steps_2> <T|F>;
 *
 * 11 - Move both motors by degrees. Last param is arrive at same time.
 *
 *  :11 <direction_1> <degrees_1> <direction_2> <degrees_2> <T|F>;
 *
 * 13 - No response (debug)
 *
 *  :13 ;
 *
 * 14 - OK noop, do data
 *
 *  :14 ;
 *
 * 17 - Set limit switch enablement for a motor
 *
 *  :17 <motorId> <T|F>;
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
// TODO: use precision 1/100th of a degree, and replace floating point math
#include <AccelStepper.h>
#include <Wire.h>

/******************************************/
/* I2C                                    */
/******************************************/
#define mainWire Wire
#define SDA_MAIN D14
#define SCL_MAIN D13
#define WIRE_ADDRESS 0x9
// volatile byte wireReq = 0x0;

/******************************************/
/* Stop Signal                            */
/******************************************/
#define stopPin D4
boolean shouldStop = false;

/******************************************/
/* State (ready/busy)                     */
/******************************************/
#define busyPin D5
boolean busy = false;

/******************************************/
/* Constants                              */
/******************************************/
#define BAUD_RATE 9600L
#define DEG_NULL 1000.00

/******************************************/
/* Command Serial                         */
/******************************************/
#define cmdSerial Serial2

/******************************************/
/* Motor Pins                             */
/******************************************/
struct MotorPins {
  byte dir;
  byte step;
  byte enable;
  byte limit_cw;
  byte limit_acw;
};

MotorPins motorPins[] = {
  {D1, D2, D0, D16, D15}, // m1
  {D7, D8, D9, D17, D18}  // m2
};
#define LIMIT_TRIPPED HIGH

/******************************************/
/* Motor Settings                         */
/******************************************/

#define absMaxSpeed_m1 4000L
#define absMaxSpeed_m2 4000L
#define defaultSpeed_m1 2000L
#define defaultSpeed_m2 2000L
/* 1/0.0008125 degrees per step */
const unsigned long millistepsPerDegree_m1 = 1230769UL;
/* 1/0.001125 degrees per step */
const unsigned long millistepsPerDegree_m2 = 888889UL;

#define maxDegrees_m1 190;
#define maxDegrees_m2 380;

#define maxAcceleration 10000L
#define motorSleepTimeout 2000L

/******************************************/
/* Motor Definition                       */
/******************************************/
struct Motor {
  AccelStepper stepper;

  byte id;

  // pins
  MotorPins pins;

  // limit switches can be disabled via command.
  boolean limitsEnabled;

  // limit switch states
  boolean isLimit_cw;
  boolean isLimit_acw;

  // whether the motor is on
  boolean isActive;

  // whether the motor is moving
  boolean isMoving;

  // the position is only meaningful if homed
  boolean hasHomed;

  // flag to reset acceleration to oldAcceleration after motors are finished
  // running, for smooth stop on limits.
  boolean isStopping;

  // flag for when we are backing up for homing purposes, so that immediately
  // after we can re-initiate homing.
  boolean isBacking;
  // flag for timing motors to arrive at same time
  boolean isTiming;

  // track the max speed and acceleration values set in stepper objects.
  unsigned long acceleration;
  unsigned long maxSpeed;
  // for temporarily overriding acceleration during stopping.
  unsigned long oldAcceleration;
  // for temporarily overriding max speed during timing.
  unsigned long oldMaxSpeed;

  // for motor sleep
  unsigned long lastActionTime;

  // configuration
  unsigned long absMaxSpeed;
  // for homing, will try not to overshoot limits
  unsigned int maxDegrees;
  unsigned long defaultSpeed;
  unsigned long millistepsPerDegree;
};

Motor motors[2];

/******************************************/
/* Entrypoint Functions                   */
/******************************************/

void setup() {
  pinMode(busyPin, OUTPUT);
  setBusy(true);
  pinMode(stopPin, INPUT_PULLDOWN);
  mainWire.setSDA(SDA_MAIN);
  mainWire.setSCL(SCL_MAIN);
  mainWire.begin(WIRE_ADDRESS);
  mainWire.onRequest(requestEvent);
  Serial.begin(BAUD_RATE);
  cmdSerial.begin(BAUD_RATE);
  setupMotors();
  setBusy(false);
}

void loop() {
  readLimitSwitches();
  readStopPin();
  if (runMotorsIfNeeded()) {
    setBusy(true);
  } else {
    checkMotorsSleep();
    setBusy(false);
    takeCommand(cmdSerial);
  }
}

/******************************************/
/* I2C Functions                          */
/******************************************/
void requestEvent() {
  TwoWire output = mainWire;
  output.print(getStatusFlag(), HEX);
  output.write('|');
  writePositions(output);
  output.write('|');
  output.print(motors[0].maxSpeed, HEX);
  output.write('|');
  output.print(motors[1].maxSpeed, HEX);
  output.write('|');
  output.print(motors[0].acceleration, HEX);
  output.write('|');
  output.print(motors[1].acceleration, HEX);
  output.write('|');
  // degrees per step
  output.print((float) 1000 / motors[0].millistepsPerDegree, 8);
  output.write('|');
  output.print((float) 1000 / motors[1].millistepsPerDegree, 8);
  output.write('|');
  output.print(motors[0].maxDegrees, HEX);
  output.write('|');
  output.print(motors[1].maxDegrees, HEX);
  output.write('|');
  output.print(motors[0].defaultSpeed, HEX);
  output.write('|');
  output.print(motors[1].defaultSpeed, HEX);
  output.write('|');
  output.print(motors[0].absMaxSpeed, HEX);
  output.write('|');
  output.print(motors[1].absMaxSpeed, HEX);
  output.write('|');
  output.write('\n');
}

/******************************************/
/* Command Input Functions                */
/******************************************/

void takeCommand(Stream &ioStream) {
  takeCommand(ioStream, ioStream);
}

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
    moveMotor(motorId, howMuch);

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
    long steps = degtos(motorId, howMuch);
    moveMotor(motorId, steps);

    output.write("=00;");
    output.print(steps);
    output.write('\n');

  } else if (command.equals("06")) {

    // home a single motor

    // param is the motor id
    byte motorId = readMotorIdFromInput(input, ';');

    if (motorId == 0) {
      output.write("=45\n");
      return;
    }

    // input.readStringUntil(';');

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
    byte motorId = readMotorIdFromInput(input, ';');

    if (motorId == 0) {
      output.write("=45\n");
      return;
    }

    // input.readStringUntil(';');

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
    long howMuch2 = input.readStringUntil(' ').toInt() * dirMult2;
    if (howMuch2 == 0) {
      output.write("=47\n");
      return;
    }

    // fifth param is isSameTime
    boolean isSameTime = input.readStringUntil(';').equals("T");
    
    // perform action
    if (isSameTime) {
      moveBothWithTiming(howMuch1, howMuch2);
    } else {
      moveMotor(1, howMuch1);
      moveMotor(2, howMuch2);
    }

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
    float howMuch2 = input.readStringUntil(' ').toFloat() * dirMult2;
    if (howMuch2 == 0) {
      output.write("=47\n");
      return;
    }

    // fifth param is isSameTime
    boolean isSameTime = input.readStringUntil(';').equals("T");

    long steps1 = degtos(1, howMuch1);
    long steps2 = degtos(2, howMuch2);

    // perform action

    if (isSameTime) {
      moveBothWithTiming(steps1, steps2);
    } else {
      moveMotor(1, steps1);
      moveMotor(2, steps2);
    }

    output.write("=00;");
    output.print(steps1);
    output.write(':');
    output.print(steps2);
    output.write('\n');
  } else if (command.equals("13")) {
    // no response
    input.readStringUntil(';');

  } else if (command.equals("14")) {
    // noop OK, no data
    input.readStringUntil(';');
    output.write("=00\n");
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

  } else {
    output.write("=44\n");
  }
}

// write positions in degrees.
void writePositions(Stream &output) {
  for (byte i = 0; i < 2; i++) {
    if (i > 0) {
      output.write('|');
    }
    output.print(getMotorPositionDegrees(i + 1), 4);
  }
}

byte readMotorIdFromInput(Stream &input) {
  return readMotorIdFromInput(input, ' ');
}

byte readMotorIdFromInput(Stream &input, char terminator) {
  byte motorId = input.readStringUntil(terminator).toInt();
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

/******************************************/
/* Move Functions                         */
/******************************************/

byte runMotorsIfNeeded() {
  
  byte runMask = 0;

  for (byte i = 0; i < 2; i++) {
    if (motors[i].stepper.distanceToGo() != 0) {
      // this will move at most one step
      motors[i].stepper.run();
      if (shouldStop || !motorCanMove(motors[i].id, motors[i].stepper.distanceToGo())) {
        stopMotor(motors[i].id);
      }
      registerMotorAction(motors[i].id);
      runMask |= 1 << i;
      motors[i].isMoving = true;
    } else {
      motors[i].isMoving = false;
      if (motors[i].isBacking) {
        // we have finished backing for home
        motors[i].isBacking = false;
        homeMotor(motors[i].id);
      } else {
        if (motors[i].isStopping) {
          // we have finished stopping
          motors[i].isStopping = false;
          setAcceleration(motors[i].id, motors[i].oldAcceleration);
          if (!shouldStop && isMotorHome(motors[i].id)) {
            // we have reached a limit switch, see if we are home
            motors[i].hasHomed = true;
            motors[i].stepper.setCurrentPosition(0);
          }
        }
        if (motors[i].isTiming) {
          setMaxSpeed(motors[i].id, motors[i].oldMaxSpeed);
          motors[i].isTiming = false;
        }
      }
    }
  }

  return runMask;
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

void moveMotor(byte motorId, long howMuch) {
  setBusy(true);
  if (motorCanMove(motorId, howMuch)) {
    motors[motorId - 1].stepper.move(howMuch);
    enableMotor(motorId);
  }
}

void moveBothWithTiming(long howMuch1, long howMuch2) {
  setBusy(true);
  motors[0].oldMaxSpeed = motors[0].maxSpeed;
  motors[1].oldMaxSpeed = motors[1].maxSpeed;
  motors[0].isTiming = true;
  motors[1].isTiming = true;
  // how long (sec) will it take m1, given its current max speed (steps/sec), to move howMuch1 steps
  float t_pre_m1 = (float) abs(howMuch1) / motors[0].maxSpeed;
  float t_pre_m2 = (float) abs(howMuch2) / motors[1].maxSpeed;
  // max time in seconds
  float t_est = max(t_pre_m1, t_pre_m2);
  // set max speeds
  unsigned long speed_m1 = abs(howMuch1) / t_est;
  unsigned long speed_m2 = abs(howMuch2) / t_est;
  setMaxSpeed(1, speed_m1);
  setMaxSpeed(2, speed_m2);
  // move motors
  moveMotor(1, howMuch1);
  moveMotor(2, howMuch2);
}

// the howMuch is just a positive/negative direction reference.
boolean motorCanMove(byte motorId, long howMuch) {
  byte i = motorId - 1;
  return !motors[i].limitsEnabled || (howMuch > 0 ? !motors[i].isLimit_cw : !motors[i].isLimit_acw);
}

long degtos(byte motorId, float howMuch) {
  return (howMuch * motors[motorId - 1].millistepsPerDegree) / 1000;
}

/******************************************/
/* Home/End Functions                     */
/******************************************/

boolean motorCanHome(byte motorId) {
  return motors[motorId - 1].limitsEnabled;
}

boolean isMotorHome(byte motorId) {
  return motorCanHome(motorId) && motors[motorId - 1].isLimit_acw;
}

void homeMotor(byte motorId) {
  if (!motorCanHome(motorId)) {
    return;
  }
  if (isMotorHome(motorId)) {
    // move forward just a little
    motors[motorId - 1].isBacking = true;
    moveMotor(motorId, degtos(motorId, 1.5));
    // homing will recommence after backing is complete
    return;
  }

  float degreesToMove = motors[motorId - 1].maxDegrees;
  float mposDegrees = getMotorPositionDegrees(motorId);
  // if we know position, don't way overshoot
  if (mposDegrees != DEG_NULL && mposDegrees > 0) {
    degreesToMove = mposDegrees + 10;
  }

  moveMotor(motorId, degtos(motorId, -1 * degreesToMove));
}

void endMotor(byte motorId) {
  if (!motorCanHome(motorId)) {
    return;
  }
  moveMotor(motorId, degtos(motorId, motors[motorId - 1].maxDegrees));
}

// returns DEG_NULL if motor has not homed.
float getMotorPositionDegrees(byte motorId) {
  byte i = motorId - 1;
  if (motors[i].hasHomed) {
    return (motors[i].stepper.currentPosition() * 1000) / motors[i].millistepsPerDegree;
  }
  return DEG_NULL;
}

/******************************************/
/* Other Functions                        */
/******************************************/

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
    digitalWrite(motors[i].pins.enable, LOW);
    motors[i].isActive = true;
    delay(2);
  }
  registerMotorAction(motorId);
}

void disableMotor(byte motorId) {
  byte i = motorId - 1;
  if (motors[i].isActive) {
    digitalWrite(motors[i].pins.enable, HIGH);
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
    motors[i].isLimit_cw = digitalRead(motors[i].pins.limit_cw) == LIMIT_TRIPPED;
    motors[i].isLimit_acw = digitalRead(motors[i].pins.limit_acw) == LIMIT_TRIPPED;
  }
}

/******************************************/
/* Stop Signal Functions                  */
/******************************************/
void readStopPin() {
  shouldStop = digitalRead(stopPin) == HIGH;
}

/******************************************/
/* Status Functions           */
/******************************************/
void setBusy(boolean value) {
  if (value != busy) {
    busy = value;
    digitalWrite(busyPin, busy);
  }
}

unsigned int getStatusFlag() {
  return (
    (busy << 0) |
    (shouldStop << 1) |
    (motors[0].isLimit_cw << 2) |
    (motors[1].isLimit_cw << 3) |
    (motors[0].isLimit_acw << 4) |
    (motors[1].isLimit_acw << 5) |
    (motors[0].isMoving << 6) |
    (motors[1].isMoving << 7) |
    (motors[0].isActive << 8) |
    (motors[1].isActive << 9) |
    (motors[0].hasHomed << 10) |
    (motors[1].hasHomed << 11) |
    (motors[0].limitsEnabled << 12) |
    (motors[1].limitsEnabled << 13)
  );
}

/******************************************/
/* Setup Functions                        */
/******************************************/
void setupMotors() {
  motors[0].millistepsPerDegree = millistepsPerDegree_m1;
  motors[1].millistepsPerDegree = millistepsPerDegree_m2;
  motors[0].maxDegrees = maxDegrees_m1;
  motors[1].maxDegrees = maxDegrees_m2;
  motors[0].absMaxSpeed = absMaxSpeed_m1;
  motors[1].absMaxSpeed = absMaxSpeed_m2;
  motors[0].defaultSpeed = defaultSpeed_m1;
  motors[1].defaultSpeed = defaultSpeed_m2;

  for (byte i = 0; i < 2; i++) {

    motors[i].id = i + 1;
    motors[i].pins = motorPins[i];

    // Declare pins as output:
    pinMode(motors[i].pins.step, OUTPUT);
    pinMode(motors[i].pins.dir, OUTPUT);
    pinMode(motors[i].pins.enable, OUTPUT);
    // Declare limit switch pins as input
    pinMode(motors[i].pins.limit_cw, INPUT_PULLDOWN);
    pinMode(motors[i].pins.limit_acw, INPUT_PULLDOWN);

    motors[i].stepper = AccelStepper(
      AccelStepper::FULL2WIRE, motors[i].pins.step, motors[i].pins.dir
    );

    motors[i].limitsEnabled = true;
    motors[i].lastActionTime = millis();

    // set initial state of motor to disabled
    digitalWrite(motors[i].pins.enable, HIGH);
    motors[i].isActive = false;

    // step max speed & acceleration
    setMaxSpeed(motors[i].id, motors[i].defaultSpeed);
    setAcceleration(motors[i].id, maxAcceleration);
  }
}
