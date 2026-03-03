/**
 *
 * RP2350
 *
 *  Unallocated pins:
 *    D3
 *    D4 (SDA1)
 *    D5 (SCL1)
 *    D6
 *    D10
 *    D11
 *    D12
 */
#include <AccelStepper.h>
#include <Wire.h>

#define DEBUG 0

#if DEBUG
#define D_SerialBegin(...) Serial.begin(__VA_ARGS__)
#define D_print(...) Serial.print(__VA_ARGS__)
#define D_write(...) Serial.write(__VA_ARGS__)
#define D_println(...) Serial.println(__VA_ARGS__)
#else
#define D_SerialBegin(...)
#define D_print(...)
#define D_write(...)
#define D_println(...)
#endif

typedef enum {
  OK = 0x0,
  MOTOR_BUSY = 0x1f,
  MALFORMED_COMMAND = 0x28,
  UNKNOWN_COMMAND = 0x2c,
  INVALID_MOTORID = 0x2d,
  COMMAND_IGNORED = 0x2e,
} ResCode;

/******************************************/
/* I2C                                    */
/******************************************/
#define mainWire Wire
#define SDA_MAIN D14
#define SCL_MAIN D13
#define WIRE_ADDRESS 0x9

/******************************************/
/* Constants                              */
/******************************************/
#define BAUD_RATE 9600L
#define DEG_NULL 1000.00
#define POS_NULL 10000000UL
#define LIMIT_TRIPPED HIGH
#define MOTOR_ON LOW
#define MAX_MOTORS 4
#define ENABLE_DELAY_MS 2
#define MOTOR_SLEEP_TIMEOUT_MS 2000

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
  long targetPos = POS_NULL;
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
  mainWire.setSDA(SDA_MAIN);
  mainWire.setSCL(SCL_MAIN);
  mainWire.onRequest(requestEvent);
  mainWire.onReceive(receiveEvent);
  setupMotors();
  mainWire.begin(WIRE_ADDRESS);
  D_SerialBegin(BAUD_RATE);
}

void loop() {
  if (!runMotorsIfNeeded()) {
    readLimitSwitches();
  }
}

byte runMotorsIfNeeded() {
  byte runMask = 0;
  for (auto& m : motors) {
    if (m.stepper.distanceToGo() != 0) {
      runActiveMotor(m);
      runMask |= 1 << m.id - 1;
    } else {
      if (m.isMoving) {
        updateIdleMotor(m);
      } else if (m.isActive) {
        checkMotorSleep(m);
      }
    }
  }
  return runMask;
}

void runActiveMotor(Motor &m) {
  readLimitSwitches(m);
  if (millis() > m.enabledAt + ENABLE_DELAY_MS) {
    // this will move at most one step
    m.stepper.run();
    if (!motorCanMove(m, m.stepper.distanceToGo())) {
      stopMotor(m, false);
    }
  }
  registerMotorAction(m);
  if (!m.isMoving) {
    m.isMoving = true;
  }
  if (m.hasHomed) {
    m.pos = m.stepper.currentPosition();
    m.targetPos = m.stepper.targetPosition();
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
        m.stepper.setMaxSpeed(m.maxSpeed);
        m.pos = m.stepper.currentPosition();
      } else if (isMotorEnd(m) && m.hasHomed) {
        // store the known max position
        m.posMax = m.stepper.currentPosition();
      }
    }
  }
  m.targetPos = m.pos;
  checkMotorSleep(m);
}

/******************************************/
/* Move Functions                         */
/******************************************/

void stopMotor(Motor &m, const boolean force) {
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

boolean moveMotor(Motor &m, const long howMuch) {
  if (motorCanMove(m, howMuch)) {
    m.stepper.move(howMuch);
    enableMotor(m);
    return true;
  }
  return false;
}

// the howMuch is just a positive/negative direction reference.
boolean motorCanMove(const Motor &m, const long howMuch) {
  return !m.limitsEnabled || (howMuch > 0 ? !m.isLimit_cw : !m.isLimit_acw);
}

long degtos(const Motor &m, const float howMuch) {
  return (howMuch * m.millistepsPerDegree) / 1000;
}

/******************************************/
/* Home/End Functions                     */
/******************************************/

boolean homeMotor(Motor &m) {
  if (!m.limitsEnabled || m.isHoming || m.isEnding || m.isBacking || m.isForwarding) {
    return false;
  }
  overrideMaxSpeed(m, m.homingSpeed);
  overrideAcceleration(m, m.maxAcceleration);
  if (isMotorHome(m)) {
    // move back just a little
    m.isBacking = true;
    moveMotor(m, degtos(m, 1.5));
    // homing will recommence after backing is complete
  } else {
    m.isHoming = true;
    moveMotor(m, -getOverLimitStepsToMove(m));
  }
  return true;
}

boolean endMotor(Motor &m) {
  if (!m.limitsEnabled || m.isHoming || m.isEnding || m.isBacking || m.isForwarding) {
    return false;
  }
  overrideMaxSpeed(m, m.homingSpeed);
  overrideAcceleration(m, m.maxAcceleration);
  if (isMotorEnd(m)) {
    // move forward just a little
    m.isForwarding = true;
    moveMotor(m, -degtos(m, 1.5));
    // ending will recommence after forwarding is complete
  } else {
    m.isEnding = true;
    moveMotor(m, getOverLimitStepsToMove(m));
  }
  return true;
}

boolean isMotorHome(const Motor &m) {
  return m.limitsEnabled && m.isLimit_acw;
}

boolean isMotorEnd(const Motor &m) {
  return m.limitsEnabled && m.isLimit_cw;
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

void setLimitSwitchEnablement(Motor &m, const boolean value) {
  if (m.limitsEnabled != value) {
    m.limitsEnabled = value;
  }
}

unsigned long getOverLimitStepsToMove(const Motor &m) {
  float degreesToMove = m.maxDegrees;
  const float mposDegrees = m.hasHomed ? (m.pos * 1000) / m.millistepsPerDegree : DEG_NULL;
  // if we know position, don't way overshoot
  if (mposDegrees != DEG_NULL && mposDegrees > 0) {
    degreesToMove = mposDegrees + 10;
  }
  return degtos(m, degreesToMove);
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
  if (m.isActive && !m.isMoving && millis() - m.lastActionTime > MOTOR_SLEEP_TIMEOUT_MS) {
    disableMotor(m);
  }
}

/******************************************/
/* I2C Functions                          */
/******************************************/

byte getMotorFlag1(const Motor &m) {
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

byte getMotorFlag2(const Motor &m) {
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
    if (motorReg <= 0x6 && howMany > 1 || motorReg > 0x6 && motorReg <= 0xc && howMany != 5 || motorReg > 0xc && howMany != 9) {
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
    if (motorReg <= 0x6) {
      wireRes1 = applyMotorReg(motors[motorId - 1], motorReg);
    } else if (motorReg <= 0xc) {
      // Command with 1 long param
      byte buf[4];
      wire.readBytes(buf, 4);
      wireRes1 = applyMotorReg(motors[motorId - 1], motorReg, unpackLong(buf));
    } else {
      // Command with 2 long params
      byte buf[4];
      wire.readBytes(buf, 4);
      const unsigned long p1 = unpackLong(buf);
      wire.readBytes(buf, 4);
      const unsigned long p2 = unpackLong(buf);
      wireRes1 = applyMotorReg(motors[motorId - 1], motorReg, p1, p2);
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
      wireRes1 = UNKNOWN_COMMAND;
    } else if (otherReg < 0x30) {
      if (otherReg < 0x24) {
        // 1 byte param, followed by 2 to MAX_MOTORS long params
        if (MAX_MOTORS < 2 || howMany != 10 && (MAX_MOTORS < 3 || howMany != 14) && (MAX_MOTORS < 4 || howMany != 18)) {
          while (wire.available()) {
            wire.read();
          }
          // wrong number of bytes
          wireRes1 = MALFORMED_COMMAND;
          return;
        }
        const byte flag = wire.read();
        const byte numValues = (howMany - 2) / 4;
        long values[MAX_MOTORS];
        byte buf[4];
        for (byte i = 0; i < numValues; ++i) {
          wire.readBytes(buf, 4);
          values[i] = (long) unpackLong(buf);
        }
        wireRes1 = applyOtherReg(otherReg, flag, values, numValues);
      } else {
        wireRes1 = UNKNOWN_COMMAND;
      }
    } else if (otherReg < 0x40) {
      wireRes1 = UNKNOWN_COMMAND;
    }
  }
}

void writeMotorReg(Stream &output, const Motor &m, const byte reg) {
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
      packLong(m.targetPos, buf);
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
      if (!homeMotor(m)) {
        return COMMAND_IGNORED;
      }
    } else if (reg == 0x2) {
      if (!endMotor(m)) {
        return COMMAND_IGNORED;
      }
    } else if (reg == 0x3) {
      setLimitSwitchEnablement(m, true);
    } else if (reg == 0x4) {
      setLimitSwitchEnablement(m, false);
    }
  } else if (reg == 0x5) {
    return UNKNOWN_COMMAND;
  } else if (reg == 0x6) {
    return UNKNOWN_COMMAND;
  }
  return OK;
}

ResCode applyMotorReg(Motor &m, const byte reg, const unsigned long value) {
  if (m.isMoving) {
    return MOTOR_BUSY;
  }
  if (reg == 0x7) {
    if (m.pos == POS_NULL || !moveMotor(m, value - m.pos)) {
      return COMMAND_IGNORED;
    }
  } else if (reg == 0x8) {
    if (!moveMotor(m, value)) {
      return COMMAND_IGNORED;
    }
  } else if (reg == 0x9) {
    if (!moveMotor(m, -value)) {
      return COMMAND_IGNORED;
    }
  } else if (reg == 0xa) {
    setMaxSpeed(m, value);
  } else if (reg == 0xb) {
    setAcceleration(m, value);
  } else if (reg == 0xc) {
    setHomingSpeed(m, value);
  }
  return OK;
}

ResCode applyMotorReg(Motor &m, const byte reg, const unsigned long p1, const unsigned long p2) {
  if (m.isMoving) {
    return MOTOR_BUSY;
  }
  overrideMaxSpeed(m, p2);
  boolean ret = false;
  if (reg == 0xd) {
    ret = m.pos != POS_NULL && moveMotor(m, p1 - m.pos);
  } else if (reg == 0xe) {
    ret = moveMotor(m, p1);
  } else if (reg == 0xf) {
    ret = moveMotor(m, -p1);
  }
  if (!ret) {
    restoreMaxSpeed(m);
    return COMMAND_IGNORED;
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

ResCode applyOtherReg(const byte reg, const byte flag, long *values, const byte numValues) {
  // first 4 flag bits select the motors
  byte ids[MAX_MOTORS];
  byte count = 0;
  for (byte id = 1; id <= MAX_MOTORS; ++id) {
    if (flag >> 3 + id & 1) {
      ids[count++] = id;
    }
  }
  if (count != numValues) {
    // different number of motors selected than the number of values passed
    return MALFORMED_COMMAND;
  }
  // ensure valid motor ids
  for (byte i = 0; i < count; ++i) {
    if (ids[i] > numMotors) {
      return INVALID_MOTORID;
    }
  }
  // ensure all motors are not busy
  for (byte i = 0; i < count; ++i) {
    Motor& m = motors[ids[i] - 1];
    if (m.isMoving) {
      return MOTOR_BUSY;
    }
  }
  if (reg == 0x20 || reg == 0x21) {
    // move relative number of steps
    // last 4 bits select direction for each motor (1 is ACW)
    for (byte i = 0; i < count; ++i) {
      if (flag >> ids[i] - 1 & 1) {
        values[i] *= -1;
      }
    }
  } else {
    // move to absolute position
    for (byte i = 0; i < count; ++i) {
      Motor& m = motors[ids[i] - 1];
      if (m.pos == POS_NULL) {
        // not all motors have known position
        return COMMAND_IGNORED;
      }
      values[i] -= m.pos;
    }
  }
  if (reg == 0x21 || reg == 0x23) {
    // calculate max time
    float maxTime = 0;
    for (byte i = 0; i < count; ++i) {
      maxTime = max(maxTime, (float) abs(values[i]) / motors[ids[i] - 1].maxSpeed);
    }
    // calculate max speeds
    if (maxTime > 0) {
      for (byte i = 0; i < count; ++i) {
        const unsigned long maxSpeed = abs(values[i]) / maxTime;
        if (maxSpeed > 0) {
          overrideMaxSpeed(motors[ids[i] - 1], maxSpeed);
        }
      }
    }
  }
  for (byte i = 0; i < count; ++i) {
    Motor& m = motors[ids[i] - 1];
    long& howMuch = values[i];
    if (howMuch != 0 && motorCanMove(m, howMuch)) {
      moveMotor(m, howMuch);
    }
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
