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
 * 06 - Home a single motor
 *
 *  :06 <motorId>;
 *
 * 08 - End a single motor
 *
 *  :08 <motorId>;
 *
 * 10 - Move both motors by steps. Last param is arrive at same time.
 *
 *  :10 <direction_1> <steps_1> <direction_2> <steps_2> <1|0>;
 *
 * 13 - No response (debug)
 *
 *  :13;
 *
 * 14 - OK noop, do data
 *
 *  :14;
 *
 * 17 - Set limit switch enablement for a motor
 *
 *  :17 <motorId> <1|0>;
 *
 * 18 - Set homing speed for a motor
 *
 *  :18 <motorId> <speed>;
 *
 * ===================================================
 *
 * Parameters
 * ----------
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
#include <Wire.h>

typedef enum {
  STEPS   = 0,
  DEGREES = 2,
} Units;

typedef enum {
  OK = 0,
  DEVICE_CLOSED = 1,
  COMMAND_TIMEOUT = 2,
  FLUSH_ERROR = 3,
  MALFORMED_COMMAND = 40,
  UNKNOWN_COMMAND = 44,
  INVALID_MOTORID = 45,
  INVALID_DIRECTION = 46,
  INVALID_DISTANCE = 47,
  INVALID_SPEED_OR_ACCELERATION = 48,
  INVALID_OTHER = 49,
} ResCode;
/******************************************/
/* I2C                                    */
/******************************************/
#define mainWire Wire
#define SDA_MAIN D14
#define SCL_MAIN D13
#define WIRE_ADDRESS 0x9

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
unsigned int statusFlag = 0;

/******************************************/
/* Constants                              */
/******************************************/
#define BAUD_RATE 9600L
#define DEG_NULL 1000.00
#define POS_NULL 10000000UL
#define LIMIT_TRIPPED HIGH
#define STOP_TRIPPED HIGH
#define BUSY_ON HIGH
#define MOTOR_ON LOW

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

/******************************************/
/* Motor Settings                         */
/******************************************/
#define motorSleepTimeout 2000L

/******************************************/
/* Motor Definition                       */
/******************************************/
struct Motor {
  /* Fixed attributes */
  MotorPins pins;
  unsigned long millistepsPerDegree;
  unsigned int maxDegrees;
  unsigned long absMaxSpeed = 5000;
  unsigned long defaultSpeed = 2000;
  unsigned long homingSpeed = 4000;
  unsigned long maxAcceleration = 50000;
  /* Created on setup */
  AccelStepper stepper;
  byte id;
  /* Runtime configurable settings */
  boolean limitsEnabled = true;
  unsigned long acceleration;
  unsigned long maxSpeed;
  /* Stateful attributes */
  long pos = POS_NULL;
  boolean isLimit_cw;
  boolean isLimit_acw;
  boolean isActive;
  boolean isMoving;
  unsigned long lastActionTime;
  // the position is only meaningful if homed
  boolean hasHomed;
  /* Temporary flags */
  // flag to reset acceleration to oldAcceleration after motors are finished
  // running, for smooth stop on limits.
  boolean isStopping;
  // flag for when we are backing up for homing purposes, so that immediately
  // after we can re-initiate homing.
  boolean isBacking;
  // as above for ending purposes
  boolean isForwarding;
  // flag to indicate homing in progress
  boolean isHoming;
  // flag to indicate ending in progress
  boolean isEnding;
  // if we have homed and ended, we know the actual effective range
  unsigned long posMax;
  // for temporarily overriding acceleration during stopping.
  unsigned long oldAcceleration;
  // for temporarily overriding max speed during timing.
  unsigned long oldMaxSpeed;  
};

Motor motors[] = {
  {
    { D1, D2, D0, D16, D15 },
    1230769UL, /* 1/0.0008125 degrees per step */
    190,
  },
  {
    { D7, D8, D9, D17, D18 },
    888889UL, /* 1/0.001125 degrees per step */
    380,
  },
};

const byte numMotors = sizeof(motors) / sizeof(motors[0]);

/******************************************/
/* Entrypoint Functions                   */
/******************************************/
void setup() {
  pinMode(busyPin, OUTPUT);
  pinMode(stopPin, STOP_TRIPPED  == HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);
  digitalWrite(busyPin, 1 - BUSY_ON);
  mainWire.setSDA(SDA_MAIN);
  mainWire.setSCL(SCL_MAIN);
  mainWire.onRequest(requestEvent);
  mainWire.onReceive(receiveEvent);
  setupMotors();
  mainWire.begin(WIRE_ADDRESS);
  Serial.begin(BAUD_RATE);
  cmdSerial.begin(BAUD_RATE);
}

boolean inputToggle = false;

void loop() {
  readLimitSwitches();
  readStopPin();
  if (runMotorsIfNeeded()) {
    setBusy(true);
  } else {
    checkMotorsSleep();
    setBusy(false);
    if (inputToggle) {
      takeCommand(Serial);
    } else {
      takeCommand(cmdSerial);
    }
    inputToggle = !inputToggle;
  }
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
  const byte firstByte = input.read();
  // ignore trailing \n
  if (firstByte == '\n') {
    return;
  }
  if (firstByte != ':') {
    sendCommandErr(input, output, MALFORMED_COMMAND);
    return;
  }
  const long cmdId = input.parseInt(SKIP_NONE);
  if (cmdId == 1) {
    // Move a motor n steps in a direction
    // first param is the motor id, 1 or 2
    const byte motorId = readMotorIdFromInput(input);
    if (motorId == 0) {
      sendCommandErr(input, output, INVALID_MOTORID);
      return;
    }
    // second param is direction 1: clockwise, 2: anti-clockwise
    const int dirMult = getDirMultiplier(input.parseInt(SKIP_WHITESPACE));
    if (dirMult == 0) {
      sendCommandErr(input, output, INVALID_DIRECTION);
      return;
    }
    // third param is how many steps
    const long howMuch = input.parseInt(SKIP_WHITESPACE) * dirMult;
    if (howMuch == 0) {
      sendCommandErr(input, output, INVALID_DISTANCE);
      return;
    }
    input.readStringUntil(';');
    moveMotor(motors[motorId - 1], howMuch);
    output.write("=00\n");
  } else if (cmdId == 2 || cmdId == 3 || cmdId == 18) {
    // 2: Set max speed for motor
    // 3: Set acceleration for motor
    // 18: Set homing speed for motor
    // first param is the motor id
    const byte motorId = readMotorIdFromInput(input);
    if (motorId == 0) {
      sendCommandErr(input, output, INVALID_MOTORID);
      return;
    }
    // second param is the new value
    const unsigned long newValue = input.parseInt(SKIP_WHITESPACE);
    if (newValue == 0) {
      sendCommandErr(input, output, INVALID_SPEED_OR_ACCELERATION);
      return;
    }
    input.readStringUntil(';');
    if (cmdId == 2) {
      setMaxSpeed(motors[motorId - 1], newValue);
    } else if (cmdId == 3) {
      setAcceleration(motors[motorId - 1], newValue);
    } else if (cmdId == 18) {
      setHomingSpeed(motors[motorId - 1], newValue);
    }
    output.write("=00\n");
  } else if (cmdId == 6 || cmdId == 8) {
    // 6: home a single motor
    // 8: end a single motor
    // param is the motor id
    const byte motorId = readMotorIdFromInput(input);
    if (motorId == 0) {
      sendCommandErr(input, output, INVALID_MOTORID);
      return;
    }
    input.readStringUntil(';');
    Motor& m = motors[motorId - 1];
    if (!motorCanHome(m)) {
      sendCommandErr(input, output, INVALID_DISTANCE);
      return;
    }
    if (cmdId == 6) {
      homeMotor(m);
    } else if (cmdId == 8) {
      endMotor(m);
    }
    output.write("=00\n");
  } else if (cmdId == 10) {
    // move both motors by steps
    // first param is direction_1
    const int dirMult1 = getDirMultiplier(input.parseInt(SKIP_WHITESPACE));
    if (dirMult1 == 0) {
      sendCommandErr(input, output, INVALID_DIRECTION);
      return;
    }
    // second param is steps_1
    const long howMuch1 = input.parseInt(SKIP_WHITESPACE) * dirMult1;
    if (howMuch1 == 0) {
      sendCommandErr(input, output, INVALID_DISTANCE);
      return;
    }
    // third param is direction_2
    const int dirMult2 = getDirMultiplier(input.parseInt(SKIP_WHITESPACE));
    if (dirMult2 == 0) {
      sendCommandErr(input, output, INVALID_DIRECTION);
      return;
    }
    // fourth param is steps_2
    const long howMuch2 = input.parseInt(SKIP_WHITESPACE) * dirMult2;
    if (howMuch2 == 0) {
      sendCommandErr(input, output, INVALID_DISTANCE);
      return;
    }
    // fifth param is isSameTime
    input.readStringUntil(' ');
    const int isSameTime = getBoolParam(input.readStringUntil(';'));
    if (isSameTime < 0) {
      sendCommandErr(input, output, INVALID_OTHER);
      return;
    }
    if (isSameTime) {
      moveBothWithTiming(howMuch1, howMuch2);
    } else {
      moveMotor(motors[0], howMuch1);
      moveMotor(motors[1], howMuch2);
    }
    output.write("=00\n");
  } else if (cmdId == 13) {
    // no response
    input.readStringUntil(';');
  } else if (cmdId == 14) {
    // noop OK, no data
    input.readStringUntil(';');
    output.write("=00\n");
  } else if (cmdId == 17) {
    // Set limit switch enablement for a motor
    const byte motorId = readMotorIdFromInput(input);
    if (motorId == 0) {
      sendCommandErr(input, output, INVALID_MOTORID);
      return;
    }
    input.readStringUntil(' ');
    const int value = getBoolParam(input.readStringUntil(';'));
    if (value < 0) {
      sendCommandErr(input, output, INVALID_OTHER);
      return;
    }
    setLimitSwitchEnablement(motors[motorId - 1], (boolean) value);
    output.write("=00\n");
  } else {
    sendCommandErr(input, output, UNKNOWN_COMMAND);
  }
}

void sendCommandErr(Stream &input, Stream &output, const ResCode &errCode) {
  if (input.available() && input.findUntil(";", ":")) {
    input.readStringUntil(';');
  }
  output.write('=');
  output.print(errCode);
  output.write('\n');
}

byte readMotorIdFromInput(Stream &input) {
  const byte motorId = input.parseInt(SKIP_WHITESPACE);
  if (motorId > 0 && motorId <= numMotors) {
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

int getBoolParam(String value) {
  if (value.length() == 1) {
    const char c = value.charAt(0);
    if (c == 'T' || c == '1') {
      return 1;
    }
    if (c == 'F' || c == '0') {
      return 0;
    }
  }
  return -1;
}

/******************************************/
/* Move Functions                         */
/******************************************/

byte runMotorsIfNeeded() {

  byte runMask = 0;

  for (auto& m : motors) {
  // for (byte i = 0; i < numMotors; i++) {
    if (m.stepper.distanceToGo() != 0) {
      // this will move at most one step
      if (m.stepper.run() && m.hasHomed) {
        m.pos = m.stepper.currentPosition();
      }
      if (shouldStop || !motorCanMove(m, m.stepper.distanceToGo())) {
        stopMotor(m);
      }
      registerMotorAction(m);
      runMask |= 1 << (m.id - 1);
      if (!m.isMoving) {
        m.isMoving = true;
      }
    } else {
      if (m.oldAcceleration) {
        setAcceleration(m, m.oldAcceleration);
        m.oldAcceleration = 0;
      }
      if (m.oldMaxSpeed) {
        setMaxSpeed(m, m.oldMaxSpeed);
        m.oldMaxSpeed = 0;
      }
      if (m.isBacking) {
        // we have finished backing for home
        m.isBacking = false;
        homeMotor(m);
      } else if (m.isForwarding) {
        // we have finished forwarding for end
        m.isForwarding = false;
        endMotor(m);
      } else {
        if (m.isHoming) {
          m.isHoming = false;
        }
        if (m.isEnding) {
          m.isEnding = false;
        }
        if (m.isMoving) {
          m.isMoving = false;
        }
        if (m.isStopping) {
          // we have finished stopping
          m.isStopping = false;
          if (!shouldStop) {
            // we have reached a limit switch, see if we are home
            if (isMotorHome(m)) {
              m.hasHomed = true;
              m.stepper.setCurrentPosition(0);
              m.pos = m.stepper.currentPosition();
            } else if (isMotorEnd(m) && m.hasHomed) {
              // store the known max position
              m.posMax = m.stepper.currentPosition();
            }
          }
        }
      }
    }
  }

  return runMask;
}

void stopMotor(Motor &m) {
  if (m.isStopping) {
    // don't duplicate action
    return;
  }
  m.isStopping = true;
  overrideAcceleration(m, m.maxAcceleration);
  m.stepper.stop();
}

void moveMotor(Motor &m, long howMuch) {
  setBusy(true);
  if (motorCanMove(m, howMuch)) {
    m.stepper.move(howMuch);
    enableMotor(m);
  }
}

void moveBothWithTiming(long howMuch1, long howMuch2) {
  setBusy(true);
  motors[0].oldMaxSpeed = motors[0].maxSpeed;
  motors[1].oldMaxSpeed = motors[1].maxSpeed;
  // how long (sec) will it take m1, given its current max speed (steps/sec), to move howMuch1 steps
  float t_pre_m1 = (float) abs(howMuch1) / motors[0].maxSpeed;
  float t_pre_m2 = (float) abs(howMuch2) / motors[1].maxSpeed;
  // max time in seconds
  float t_est = max(t_pre_m1, t_pre_m2);
  // set max speeds
  unsigned long speed_m1 = abs(howMuch1) / t_est;
  unsigned long speed_m2 = abs(howMuch2) / t_est;
  setMaxSpeed(motors[0], speed_m1);
  setMaxSpeed(motors[1], speed_m2);
  // move motors
  moveMotor(motors[0], howMuch1);
  moveMotor(motors[1], howMuch2);
}

// the howMuch is just a positive/negative direction reference.
boolean motorCanMove(const Motor &m, long howMuch) {
  return !m.limitsEnabled || (howMuch > 0 ? !m.isLimit_cw : !m.isLimit_acw);
}

long degtos(const Motor &m, float howMuch) {
  return (howMuch * m.millistepsPerDegree) / 1000;
}

/******************************************/
/* Home/End Functions                     */
/******************************************/

boolean motorCanHome(const Motor &m) {
  return m.limitsEnabled;
}

boolean isMotorHome(const Motor &m) {
  return motorCanHome(m) && m.isLimit_acw;
}

boolean isMotorEnd(const Motor &m) {
  return motorCanHome(m) && m.isLimit_cw;
}

void homeMotor(Motor &m) {
  if (!motorCanHome(m)) {
    return;
  }
  m.isHoming = true;
  overrideMaxSpeed(m, m.homingSpeed);
  overrideAcceleration(m, m.maxAcceleration);
  if (isMotorHome(m)) {
    // move back just a little
    m.isBacking = true;
    moveMotor(m, degtos(m, 1.5));
    // homing will recommence after backing is complete
    return;
  }
  moveMotor(m, -getOverLimitStepsToMove(m));
}

void endMotor(Motor &m) {
  if (!motorCanHome(m)) {
    return;
  }
  m.isEnding = true;
  overrideMaxSpeed(m, m.homingSpeed);
  overrideAcceleration(m, m.maxAcceleration);
  if (isMotorEnd(m)) {
    // move forward just a little
    m.isForwarding = true;
    moveMotor(m, -degtos(m, 1.5));
    // ending will recommence after forwarding is complete
    return;
  }
  moveMotor(m, getOverLimitStepsToMove(m));
}

unsigned long getOverLimitStepsToMove(Motor &m) {
  float degreesToMove = m.maxDegrees;
  const float mposDegrees = getMotorPositionDegrees(m);
  // if we know position, don't way overshoot
  if (mposDegrees != DEG_NULL && mposDegrees > 0) {
    degreesToMove = mposDegrees + 10;
  }
  return degtos(m, degreesToMove);
}

// returns DEG_NULL if motor has not homed.
float getMotorPositionDegrees(const Motor &m) {
  if (m.hasHomed) {
    return (m.pos * 1000) / m.millistepsPerDegree;
  }
  return DEG_NULL;
}

/******************************************/
/* Other Functions                        */
/******************************************/

void setMaxSpeed(Motor &m, unsigned long value) {
  m.maxSpeed = min(value, m.absMaxSpeed);
  m.stepper.setMaxSpeed(m.maxSpeed);
}

void setHomingSpeed(Motor &m, unsigned long value) {
  m.homingSpeed = min(value, m.absMaxSpeed);
}

void setAcceleration(Motor &m, unsigned long value) {
  m.acceleration = min(value, m.maxAcceleration);
  m.stepper.setAcceleration(m.acceleration);
}

void overrideMaxSpeed(Motor &m, unsigned long value) {
  value = min(value, m.absMaxSpeed);
  if (m.maxSpeed != value) {
    if (!m.oldMaxSpeed) {
      m.oldMaxSpeed = m.maxSpeed;
    }
    setMaxSpeed(m, value);
  }
}

void overrideAcceleration(Motor &m, unsigned long value) {
  value = min(value, m.maxAcceleration);
  if (m.acceleration != value) {
    if (!m.oldAcceleration) {
      m.oldAcceleration = m.acceleration;
    }
    setAcceleration(m, value);
  }
}

void enableMotor(Motor &m) {
  if (!m.isActive) {
    digitalWrite(m.pins.enable, MOTOR_ON);
    m.isActive = true;
    delay(2);
  }
  registerMotorAction(m);
}

void disableMotor(Motor &m) {
  if (m.isActive) {
    digitalWrite(m.pins.enable, 1 - MOTOR_ON);
    m.isActive = false;
  }
}

void registerMotorAction(Motor &m) {
  m.lastActionTime = millis();
}

void checkMotorsSleep() {
  for (auto& m : motors) {
    if (m.isActive && millis() - m.lastActionTime > motorSleepTimeout) {
      disableMotor(m);
    }
  }
}

void readLimitSwitches() {
  boolean isChange = false;
  for (auto& m : motors) {
    byte old = m.isLimit_cw | (m.isLimit_acw << 1);
    m.isLimit_cw = digitalRead(m.pins.limit_cw) == LIMIT_TRIPPED;
    m.isLimit_acw = digitalRead(m.pins.limit_acw) == LIMIT_TRIPPED;
    byte cur = m.isLimit_cw | (m.isLimit_acw << 1);
    isChange |= old != cur;
  }
}

void setLimitSwitchEnablement(Motor &m, boolean value) {
  if (m.limitsEnabled != value) {
    m.limitsEnabled = value;
  }
}

/******************************************/
/* Stop Signal Functions                  */
/******************************************/
void readStopPin() {
  shouldStop = digitalRead(stopPin) == STOP_TRIPPED;
}

/******************************************/
/* Status Functions           */
/******************************************/
void setBusy(boolean value) {
  if (value != busy) {
    busy = value;
    digitalWrite(busyPin, busy ? BUSY_ON : 1 - BUSY_ON);
  }
}

byte getMotorFlag1(Motor &m) {
  return (
    (m.isLimit_cw << 0x0) |
    (m.isLimit_acw << 0x1) |
    (m.isMoving << 0x2) |
    (m.isActive << 0x3) |
    (m.hasHomed << 0x4) |
    (m.limitsEnabled << 0x5) |
    (m.isHoming << 0x6) |
    (m.isEnding << 0x7)
  );
}

byte getMotorFlag2(Motor &m) {
  return (
    (false << 0x0) |
    (false << 0x1) |
    (false << 0x2) |
    (false << 0x3) |
    (false << 0x4) |
    (false << 0x5) |
    (false << 0x6) |
    (false << 0x7)
  );
}

volatile byte wireReg1 = 0x0;
volatile byte wireRes1 = 0x0;

void requestEvent() {
  if (wireReg1 == 0x0) {
    return;
  }
  // First 2 significant bits are category
  const byte category = wireReg1 >> 0x6;
  if (category == 0x1) {
    // Category 1: single motor attribute/data
    byte readReg = wireReg1 & ((1 << 0x6) - 1);
    wireReg1 = 0x0;
    // Next 2 bits are motor id
    const byte motorId = (readReg >> 0x4) + 1;
    readReg &= (1 << 0x4) - 1;
    // Remaining 4 bits are motor attribute
    const byte motorReg = readReg;
    if (motorId > numMotors) {
      if (motorReg <= 0x1) {
        mainWire.write(0x0);
      } else {
        byte buf[4];
        mainWire.write(buf, 4);
      }
    } else {
      writeMotorReg(mainWire, motors[motorId - 1], motorReg);
    }
  } else if (category == 0x2 || category == 0x3) {
    mainWire.write(wireRes1);
  }
}

void receiveEvent(int howMany) {
  if (howMany < 1) {
    return;
  }
  wireReg1 = mainWire.read();
  // First 2 significant bits are category
  const byte category = wireReg1 >> 0x6;
  if (category == 0x1) {
    // Category 1: read single motor attribute/data
    while (mainWire.available()) {
      mainWire.read();
      // no more bytes expected
      wireReg1 = 0x0;
    }
  } else if (category == 0x2) {
    // Category 2: single motor operation
    byte readReg = wireReg1 & ((1 << 0x6) - 1);
    // Next 2 bits are motor id
    const byte motorId = (readReg >> 0x4) + 1;
    readReg &= (1 << 0x4) - 1;
    // Remaining 4 bits are motor attribute
    const byte motorReg = readReg;
    if (motorReg <= 0x7 && howMany > 1 || motorReg > 0x7 && howMany != 5) {
      while (mainWire.available()) {
        mainWire.read();
      }
      // wrong number of bytes
      wireRes1 = 0x31;
      return;
    }
    if (motorId > numMotors) {
      // invalid motor id
      wireRes1 = 0x2d;
      return;
    }
    if (motorReg <= 0x7) {
      wireRes1 = applyMotorReg(motors[motorId - 1], motorReg);
    } else {
      byte buf[4];
      mainWire.readBytes(buf, 4);
      wireRes1 = applyMotorReg(motors[motorId - 1], motorReg, unpackLong(buf));
    }
  } else if (category == 0x3) {
    // Category 3: other
    const byte otherReg = wireReg1 & ((1 << 0x6) - 1);
    if (otherReg < 0x10) {
      // Command with no params
      if (howMany > 1) {
        while (mainWire.available()) {
          mainWire.read();
        }
        // wrong number of bytes
        wireRes1 = 0x31;
        return;
      }
      wireRes1 = applyOtherReg(otherReg);
    } else if (otherReg < 0x20) {
      // Command with 2 long params
      if (howMany != 9) {
        while (mainWire.available()) {
          mainWire.read();
        }
        // wrong number of bytes
        wireRes1 = 0x31;
        return;
      }
      byte buf[4];
      mainWire.readBytes(buf, 4);
      unsigned long p1 = unpackLong(buf);
      mainWire.readBytes(buf, 4);
      unsigned long p2 = unpackLong(buf);
      wireRes1 = applyOtherReg(otherReg, p1, p2);
    } else if (otherReg < 0x30) {
      wireRes1 = UNKNOWN_COMMAND;
    } else if (otherReg < 0x40) {
      wireRes1 = UNKNOWN_COMMAND;
    }
  }
}

void writeMotorReg(Stream &output, Motor &m, const byte reg) {
  if (reg <= 0x1) {
    if (reg == 0x0) {
      output.write(getMotorFlag1(m));
    } else if (reg == 0x1) {
      output.write(getMotorFlag2(m));
    }
    return;
  }
  byte buf[4];
  if (reg <= 0x8) {
    if (reg == 0x2) {
      packLong(m.pos, buf);
    } else if (reg == 0x3) {
      packLong(m.maxSpeed, buf);
    } else if (reg == 0x4) {
      packLong(m.acceleration, buf);
    } else if (reg == 0x5) {
      packLong(m.millistepsPerDegree, buf);
    } else if (reg == 0x6) {
      packLong(m.maxDegrees, buf);
    } else if (reg == 0x7) {
      packLong(m.defaultSpeed, buf);
    } else if (reg == 0x8) {
      packLong(m.homingSpeed, buf);
    }
  } else {
    if (reg == 0x9) {
      packLong(m.absMaxSpeed, buf);
    } else if (reg == 0xa) {
      packLong(m.maxAcceleration, buf);
    } else if (reg == 0xb) {
    } else if (reg == 0xc) {
    } else if (reg == 0xd) {
    } else if (reg == 0xe) {
    } else if (reg == 0xf) {
    }
  }
  output.write(buf, 4);
}

ResCode applyMotorReg(Motor &m, const byte reg) {
  if (reg == 0x0) {
    stopMotor(m);
  } else if (reg == 0x1) {
    homeMotor(m);
  } else if (reg == 0x2) {
    endMotor(m);
  } else if (reg == 0x3) {
    setLimitSwitchEnablement(m, true);
  } else if (reg == 0x4) {
    setLimitSwitchEnablement(m, false);
  } else if (reg == 0x5) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0x6) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0x7) {
    return UNKNOWN_COMMAND;
  }
  return OK;
}

ResCode applyMotorReg(Motor &m, const byte reg, const unsigned long value) {
  if (reg == 0x8) {
    moveMotor(m, value);
  } else if (reg == 0x9) {
    moveMotor(m, -value);
  } else if (reg == 0xa) {
    setMaxSpeed(m, value);
  } else if (reg == 0xb) {
    setAcceleration(m, value);
  } else if (reg == 0xc) {
    setHomingSpeed(m, value);
  } else if (reg == 0xd) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0xe) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0xf) {
    return UNKNOWN_COMMAND;
  }
  return OK;
}

ResCode applyOtherReg(const byte reg) {
  if (reg == 0x0) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0x1) {
    // Stop all motors
    for (auto& m : motors) {
      stopMotor(m);
    }
  } else if (reg == 0x2) {
    // Home all motors
    for (auto& m : motors) {
      homeMotor(m);
    }
  } else if (reg == 0x3) {
    // End all motors
    for (auto& m : motors) {
      endMotor(m);
    }
  } else if (reg == 0x4) {
    // Enable limits for all motors
    for (auto& m : motors) {
      setLimitSwitchEnablement(m, true);
    }
  } else if (reg == 0x5) {
    // Disable limits for all motors
    for (auto& m : motors) {
      setLimitSwitchEnablement(m, false);
    }
  } else if (reg == 0x6) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0x7) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0x8) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0x9) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0xa) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0xb) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0xc) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0xd) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0xe) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0xf) {
    return UNKNOWN_COMMAND;
  }
  return OK;
}

ResCode applyOtherReg(const byte reg, const unsigned long p1, const unsigned long p2) {
  if (reg == 0x10) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0x11) {
    // Move m1 and m2 cw without timing
    moveMotor(motors[0], p1);
    moveMotor(motors[1], p2);
  } else if (reg == 0x12) {
    // Move m1 and m2 acw without timing
    moveMotor(motors[0], -p1);
    moveMotor(motors[1], -p2);
  } else if (reg == 0x13) {
    // Move m1 cw and m2 acw without timing
    moveMotor(motors[0], p1);
    moveMotor(motors[1], -p2);
  } else if (reg == 0x14) {
    // Move m1 acw and m2 cw without timing
    moveMotor(motors[0], -p1);
    moveMotor(motors[1], p2);
  } else if (reg == 0x15) {
    // Move m1 and m2 cw with timing
    moveBothWithTiming(p1, p2);
  } else if (reg == 0x16) {
    // Move m1 and m2 acw without timing
    moveBothWithTiming(-p1, -p2);
  } else if (reg == 0x17) {
    // Move m1 cw and m2 acw with timing
    moveBothWithTiming(p1, -p2);
  } else if (reg == 0x18) {
    // Move m1 acw and m2 cw with timing
    moveBothWithTiming(-p1, p2);
  } else if (reg == 0x19) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0x1a) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0x1b) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0x1c) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0x1d) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0x1e) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0x1f) {
    return UNKNOWN_COMMAND;
  }
  return OK;
}

void packLong(const unsigned long value, byte *buf) {
  buf[0] = (byte) ((value >> 0x18) & 0xff);
  buf[1] = (byte) ((value >> 0x10) & 0xff);
  buf[2] = (byte) ((value >> 0x8) & 0xff);
  buf[3] = (byte) (value & 0xff);
}

unsigned long unpackLong(byte *buf) {
  return (
    buf[0] << 0x18 |
    buf[1] << 0x10 |
    buf[2] << 0x8 |
    buf[3]
  );
}

/******************************************/
/* Setup Functions                        */
/******************************************/
void setupMotors() {
  for (byte i = 0; i < numMotors; i++) {
    Motor& m = motors[i];
    m.id = i + 1;
    // Declare pins as output:
    pinMode(m.pins.step, OUTPUT);
    pinMode(m.pins.dir, OUTPUT);
    pinMode(m.pins.enable, OUTPUT);
    // Declare limit switch pins as input
    pinMode(m.pins.limit_cw, LIMIT_TRIPPED == HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);
    pinMode(m.pins.limit_acw, LIMIT_TRIPPED == HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);
    // set initial state of motor to disabled
    digitalWrite(m.pins.enable, 1 - MOTOR_ON);
    m.stepper = AccelStepper(AccelStepper::FULL2WIRE, m.pins.step, m.pins.dir);
    m.lastActionTime = millis();
    // step max speed & acceleration
    if (m.defaultSpeed == 0) {
      m.defaultSpeed = m.absMaxSpeed;
    } else {
      m.defaultSpeed = min(m.defaultSpeed, m.absMaxSpeed);
    }
    m.homingSpeed = min(m.homingSpeed, m.absMaxSpeed);
    m.maxSpeed = m.defaultSpeed;
    m.stepper.setMaxSpeed(m.maxSpeed);
    m.acceleration = m.maxAcceleration;
    m.stepper.setAcceleration(m.acceleration);
  }
}
