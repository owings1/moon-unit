/**
 *
 * RP2350
 *
 *  Unallocated pins:
 *    D4 (SDA1)
 *    D5 (SCL1)
 *    D6
 *    D10
 *    D11
 *    D12
 */
#include <AccelStepper.h>
#include <Wire.h>

typedef enum {
  STEPS   = 0,
  DEGREES = 2,
} Units;

typedef enum {
  OK = 0,
  MOTOR_BUSY = 31,
  MALFORMED_COMMAND = 40,
  UNKNOWN_COMMAND = 44,
  INVALID_MOTORID = 45,
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
#define stopPin D3
boolean shouldStop = false;

/******************************************/
/* Constants                              */
/******************************************/
#define BAUD_RATE 9600L
#define DEG_NULL 1000.00
#define POS_NULL 10000000UL
#define LIMIT_TRIPPED HIGH
#define STOP_TRIPPED HIGH
#define MOTOR_ON LOW
#define MAX_MOTORS 4
#define ENABLE_DELAY_MS 2

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
  // when a motor is stopped by a command instead of naturally from limit
  boolean isForceStop;
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
  // for delaying after enabling motor
  unsigned long enabledAt;
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

const byte numMotors = min(sizeof(motors) / sizeof(motors[0]), MAX_MOTORS);

/******************************************/
/* Entrypoint Functions                   */
/******************************************/
void setup() {
  pinMode(stopPin, STOP_TRIPPED == HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);
  mainWire.setSDA(SDA_MAIN);
  mainWire.setSCL(SCL_MAIN);
  mainWire.onRequest(requestEvent);
  mainWire.onReceive(receiveEvent);
  setupMotors();
  mainWire.begin(WIRE_ADDRESS);
  Serial.begin(BAUD_RATE);
}

void loop() {
  shouldStop = digitalRead(stopPin) == STOP_TRIPPED;
  if (!runMotorsIfNeeded()) {
    readLimitSwitches();
  }
}

byte runMotorsIfNeeded() {
  byte runMask = 0;
  for (auto& m : motors) {
    if (m.stepper.distanceToGo() != 0) {
      runActiveMotor(m);
      runMask |= 1 << (m.id - 1);
    } else {
      updateIdleMotor(m);
    }
  }
  return runMask;
}

void runActiveMotor(Motor &m) {
  readLimitSwitches(m);
  if (millis() > m.enabledAt + ENABLE_DELAY_MS) {
    // this will move at most one step
    if (m.stepper.run() && m.hasHomed) {
      m.pos = m.stepper.currentPosition();
    }
    if (shouldStop || !motorCanMove(m, m.stepper.distanceToGo())) {
      stopMotor(m, shouldStop);
    }
  }
  registerMotorAction(m);
  if (!m.isMoving) {
    m.isMoving = true;
  }
}

void updateIdleMotor(Motor &m) {
  restoreAcceleration(m);
  restoreMaxSpeed(m);
  if (m.isBacking) {
    // we have finished backing for home
    m.isBacking = false;
    homeMotor(m);
    return;
  }
  if (m.isForwarding) {
    // we have finished forwarding for end
    m.isForwarding = false;
    endMotor(m);
    return;
  }
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
    if (m.isForceStop) {
      m.isForceStop = false;
    } else {
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
  checkMotorSleep(m);
}

/******************************************/
/* Move Functions                         */
/******************************************/

void stopMotor(Motor &m, boolean force) {
  if (m.isStopping) {
    // don't duplicate action
    return;
  }
  if (!m.isMoving) {
    return;
  }
  m.isStopping = true;
  m.isForceStop = force;
  overrideAcceleration(m, m.maxAcceleration);
  m.stepper.stop();
}

void moveMotor(Motor &m, long howMuch) {
  if (motorCanMove(m, howMuch)) {
    m.stepper.move(howMuch);
    enableMotor(m);
  }
}

void moveBothWithTiming(long howMuch1, long howMuch2) {
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
boolean motorCanMove(const Motor &m, const long howMuch) {
  return !m.limitsEnabled || (howMuch > 0 ? !m.isLimit_cw : !m.isLimit_acw);
}

long degtos(const Motor &m, float howMuch) {
  return (howMuch * m.millistepsPerDegree) / 1000;
}

/******************************************/
/* Home/End Functions                     */
/******************************************/

void homeMotor(Motor &m) {
  if (!motorCanHome(m)) {
    return;
  }
  if (m.isHoming || m.isEnding || m.isBacking || m.isForwarding) {
    return;
  }
  overrideMaxSpeed(m, m.homingSpeed);
  overrideAcceleration(m, m.maxAcceleration);
  if (isMotorHome(m)) {
    // move back just a little
    m.isBacking = true;
    moveMotor(m, degtos(m, 1.5));
    // homing will recommence after backing is complete
    return;
  }
  m.isHoming = true;
  moveMotor(m, -getOverLimitStepsToMove(m));
}

void endMotor(Motor &m) {
  if (!motorCanHome(m)) {
    return;
  }
  if (m.isHoming || m.isEnding || m.isBacking || m.isForwarding) {
    return;
  }
  overrideMaxSpeed(m, m.homingSpeed);
  overrideAcceleration(m, m.maxAcceleration);
  if (isMotorEnd(m)) {
    // move forward just a little
    m.isForwarding = true;
    moveMotor(m, -degtos(m, 1.5));
    // ending will recommence after forwarding is complete
    return;
  }
  m.isEnding = true;
  moveMotor(m, getOverLimitStepsToMove(m));
}

boolean motorCanHome(const Motor &m) {
  return m.limitsEnabled;
}

boolean isMotorHome(const Motor &m) {
  return motorCanHome(m) && m.isLimit_acw;
}

boolean isMotorEnd(const Motor &m) {
  return motorCanHome(m) && m.isLimit_cw;
}

void readLimitSwitches() {
  for (auto& m : motors) {
    readLimitSwitches(m);
  }
}

void readLimitSwitches(Motor &m) {
  m.isLimit_cw = digitalRead(m.pins.limit_cw) == LIMIT_TRIPPED;
  m.isLimit_acw = digitalRead(m.pins.limit_acw) == LIMIT_TRIPPED;
}

void setLimitSwitchEnablement(Motor &m, boolean value) {
  if (m.limitsEnabled != value) {
    m.limitsEnabled = value;
  }
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

void restoreMaxSpeed(Motor &m) {
  if (m.oldMaxSpeed) {
    setMaxSpeed(m, m.oldMaxSpeed);
    m.oldMaxSpeed = 0;
  }
}

void restoreAcceleration(Motor &m) {
  if (m.oldAcceleration) {
    setAcceleration(m, m.oldAcceleration);
    m.oldAcceleration = 0;
  }
}

void enableMotor(Motor &m) {
  if (!m.isActive) {
    digitalWrite(m.pins.enable, MOTOR_ON);
    m.isActive = true;
    m.enabledAt = millis();
  }
  registerMotorAction(m);
}

void disableMotor(Motor &m) {
  if (m.isActive) {
    digitalWrite(m.pins.enable, 1 - MOTOR_ON);
    m.isActive = false;
    m.enabledAt = 0;
  }
}

void registerMotorAction(Motor &m) {
  m.lastActionTime = millis();
}

void checkMotorSleep(Motor &m) {
  if (m.isActive && !m.isMoving && millis() - m.lastActionTime > motorSleepTimeout) {
    disableMotor(m);
  }
}

/******************************************/
/* I2C Functions                          */
/******************************************/

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
  TwoWire& wire = mainWire;
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
        wire.write(0x0);
      } else {
        byte buf[4];
        wire.write(buf, 4);
      }
    } else {
      writeMotorReg(wire, motors[motorId - 1], motorReg);
    }
  } else if (category == 0x2 || category == 0x3) {
    wire.write(wireRes1);
  }
}

void receiveEvent(int howMany) {
  if (howMany < 1) {
    return;
  }
  TwoWire& wire = mainWire;
  wireReg1 = wire.read();
  // First 2 significant bits are category
  const byte category = wireReg1 >> 0x6;
  if (category == 0x1) {
    // Category 1: read single motor attribute/data
    while (wire.available()) {
      wire.read();
      // no more bytes expected
      wireReg1 = 0x0;
    }
  } else if (category == 0x2) {
    // Category 2: single motor operation
    byte readReg = wireReg1 & ((1 << 0x6) - 1);
    // Next 2 bits are motor id
    const byte motorId = (readReg >> 0x4) + 1;
    readReg &= (1 << 0x4) - 1;
    // Remaining 4 bits are command/attribute
    const byte motorReg = readReg;
    if (motorReg <= 0x7 && howMany > 1 || motorReg > 0x7 && howMany != 5) {
      while (wire.available()) {
        wire.read();
      }
      // wrong number of bytes
      wireRes1 = MALFORMED_COMMAND;
      return;
    }
    if (motorId > numMotors) {
      wireRes1 = INVALID_MOTORID;
      return;
    }
    if (motorReg <= 0x7) {
      wireRes1 = applyMotorReg(motors[motorId - 1], motorReg);
    } else {
      byte buf[4];
      wire.readBytes(buf, 4);
      wireRes1 = applyMotorReg(motors[motorId - 1], motorReg, unpackLong(buf));
    }
  } else if (category == 0x3) {
    // Category 3: other
    const byte otherReg = wireReg1 & ((1 << 0x6) - 1);
    if (otherReg < 0x10) {
      // Command with no params
      if (howMany > 1) {
        while (wire.available()) {
          wire.read();
        }
        // wrong number of bytes
        wireRes1 = MALFORMED_COMMAND;
        return;
      }
      wireRes1 = applyOtherReg(otherReg);
    } else if (otherReg < 0x20) {
      // Command with 2 long params
      if (howMany != 9) {
        while (wire.available()) {
          wire.read();
        }
        // wrong number of bytes
        wireRes1 = MALFORMED_COMMAND;
        return;
      }
      byte buf[4];
      wire.readBytes(buf, 4);
      unsigned long p1 = unpackLong(buf);
      wire.readBytes(buf, 4);
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
      packLong(m.posMax, buf);
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
    stopMotor(m, true);
  } else if (reg <= 0x4) {
    if (m.isMoving) {
      return MOTOR_BUSY;
    }
    if (reg == 0x1) {
      homeMotor(m);
    } else if (reg == 0x2) {
      endMotor(m);
    } else if (reg == 0x3) {
      setLimitSwitchEnablement(m, true);
    } else if (reg == 0x4) {
      setLimitSwitchEnablement(m, false);
    }
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
  if (reg <= 0xc) {
    if (m.isMoving) {
      return MOTOR_BUSY;
    }
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
    }
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
      stopMotor(m, true);
    }
  } else if (reg <= 0x5) {
    for (auto& m : motors) {
      if (m.isMoving) {
        return MOTOR_BUSY;
      }
    }
    if (reg == 0x2) {
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
  } else if (reg <= 0x18) {
    if (numMotors < 2) {
      return INVALID_MOTORID;
    }
    if (motors[0].isMoving || motors[1].isMoving) {
      return MOTOR_BUSY;
    }
    if (reg == 0x11) {
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
    }
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
