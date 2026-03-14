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
#define checkbit(flags, bit) ((flags >> bit) & 1)

#include "Motor.h"

typedef enum {
  OK = 0x0,
  MOTOR_BUSY = 0x1f,
  MALFORMED_COMMAND = 0x28,
  UNKNOWN_COMMAND = 0x2c,
  INVALID_MOTORID = 0x2d,
  COMMAND_IGNORED = 0x2e,
  COMMAND_PARTIALLY_IGNORED = 0x2f,
  READONLY_ATTRIBUTE = 0x30,
} ResCode;

/******************************************/
/* I2C                                    */
/******************************************/
#define I2C_MAIN Wire
#define SDA_MAIN D14
#define SCL_MAIN D13
#define I2C_ADDRESS 0x9

/******************************************/
/* Constants                              */
/******************************************/
#define BAUD_RATE 9600L
#define MAX_MOTORS 4

const uint8_t numMotors = 2;

Motor motors[] = {
  Motor({ D1, D2, D0, D16, D15 }),
  Motor({ D7, D8, D9, D17, D18 })
};

void setup() {
  for (auto& m : motors) {
    m.begin();
  }
  // motors[0].setMaxSteps(233846); //1230769UL, /* 1/0.0008125 degrees per step */
  // motors[0].setBackingSteps(1846);
  // motors[1].setMaxSteps(337777); //888889UL, /* 1/0.001125 degrees per step */
  // motors[1].setBackingSteps(1333);
  I2C_MAIN.setSDA(SDA_MAIN);
  I2C_MAIN.setSCL(SCL_MAIN);
  I2C_MAIN.onRequest(requestEvent);
  I2C_MAIN.onReceive(receiveEvent);
  I2C_MAIN.begin(I2C_ADDRESS);
  D_SerialBegin(BAUD_RATE);
}

void loop() {
  if (!runMotorsIfNeeded()) {
    readLimitSwitches();
  }
}

byte runMotorsIfNeeded() {
  uint8_t runMask = 0;
  for (auto& m : motors) {
    runMask |= m.runIfNeeded() << m.id - 1;
  }
  return runMask;
}

void readLimitSwitches() {
  for (auto& m : motors) {
    m.readLimitSwitches();
  }
}

enum Reg : uint8_t {
  REG_UNSET = 0x0,
  REG_RESPONSE_CODE = 0x1
};

enum MReg : uint8_t {
  MREG_STATE_FLAGS = 0x0,
  MREG_SETTINGS_FLAGS = 0x1,
  MREG_POSITION = 0x2,
  MREG_MAX_SPEED = 0x3,
  MREG_ACCELERATION = 0x4,
  MREG_MOVE_CW = 0x5,
  MREG_MOVE_ACW = 0x6,
  MREG_SPEED = 0x7,
  MREG_TARGET_POSITION = 0xc,
  MREG_STOP = 0xf
};

volatile uint8_t wireReg = REG_UNSET;
volatile uint8_t wireRes = OK;

void requestEvent() {
  auto& wire = I2C_MAIN;
  if (wireReg == REG_UNSET) {
    return;
  }
  if (wireReg == REG_RESPONSE_CODE) {
    wireReg = REG_UNSET;
    wire.write(wireRes);
  }
  // First 2 significant bits are category
  const uint8_t category = wireReg >> 0x6;
  if (category == 0x1) {
    // Category 1: single motor attribute/data
    // Next 2 bits are motor id
    const uint8_t motorId = ((wireReg >> 0x4) & 0x3) + 1;
    // Remaining 4 bits are motor attribute
    const uint8_t motorReg = wireReg & 0xf;
    wireReg = REG_UNSET;
    if (motorId > numMotors) {
      if (motorReg == MREG_STATE_FLAGS || motorReg == MREG_SETTINGS_FLAGS) {
        wire.write(0x0);
      } else if (motorReg == MREG_POSITION || motorReg == MREG_TARGET_POSITION) {
        wire.write(0x0);
        wire.write(0x0);
        wire.write(0x0);
        wire.write(0x0);
      } else {
        wire.write(0x0);
        wire.write(0x0);
      }
    } else {
      sendMotorAttr(wire, motors[motorId - 1], motorReg);
    }
  }
}

void receiveEvent(int howMany) {
  if (howMany < 1) {
    return;
  }
  auto& wire = I2C_MAIN;
  wireReg = wire.read();
  // First 2 significant bits are category
  const byte category = wireReg >> 0x6;
  if (category == 0x1) {
    // Category 1: read/write single motor attribute/data
    // Next 2 bits are motor id
    const uint8_t motorId = ((wireReg >> 0x4) & 0x3) + 1;
    // Remaining 4 bits are command/attribute
    const uint8_t motorReg = wireReg & 0xf;
    if (howMany > 1 || motorReg == MREG_MOVE_CW || motorReg == MREG_MOVE_ACW || motorReg == MREG_STOP) {
      wireReg = REG_RESPONSE_CODE;
      if (motorReg == MREG_STATE_FLAGS || motorReg == MREG_TARGET_POSITION) {
        wireRes = READONLY_ATTRIBUTE;
        slurp(wire);
        return;
      }
      boolean isNoParamReg = motorReg == MREG_STOP;
      boolean isByteParamReg = motorReg == MREG_SETTINGS_FLAGS;
      boolean isShortParamReg = (
        motorReg == MREG_MAX_SPEED ||
        motorReg == MREG_ACCELERATION
      );
      boolean isLongParamReg = (
        motorReg == MREG_POSITION ||
        motorReg == MREG_MOVE_CW ||
        motorReg == MREG_MOVE_ACW
      );
      if (
        isNoParamReg ||
        isByteParamReg ||
        isShortParamReg ||
        isLongParamReg
      ) {
        if (
          (isNoParamReg) && howMany != 1 ||
          (isByteParamReg) && howMany != 2 ||
          (isShortParamReg) && howMany != 3 ||
          (isLongParamReg) && howMany != 5
        ) {
          wireRes = MALFORMED_COMMAND;
          slurp(wire);
          return;
        }
        if (motorId > numMotors) {
          wireRes = INVALID_MOTORID;
          slurp(wire);
          return;
        }
        Motor& m = motors[motorId - 1];
        if (motorReg == MREG_STOP) {
          wireRes = m.stop() ? OK : COMMAND_IGNORED;
        } else if (motorReg == MREG_SETTINGS_FLAGS) {
          wireRes = setMotorAttr(m, motorReg, (uint8_t) wire.read());
        } else if (isLongParamReg) {
          uint8_t buf[4];
          wire.readBytes(buf, 4);
          const uint32_t value = unpackLong(buf);
          if (motorReg == MREG_POSITION) {
            wireRes = setMotorAttr(m, motorReg, value);
          } else if (motorReg == MREG_MOVE_CW) {
            wireRes = m.move(value) ? OK : COMMAND_IGNORED;
          } else if (motorReg == MREG_MOVE_ACW) {
            wireRes = m.move(-value) ? OK : COMMAND_IGNORED;
          }
        } else {
          uint8_t buf[2];
          wire.readBytes(buf, 2);
          wireRes = setMotorAttr(m, motorReg, unpackShort(buf));
        }
      } else {
        wireRes = UNKNOWN_COMMAND;
        slurp(wire);
      }
    }
  } else {
    slurp(wire);
    wireReg = REG_UNSET;
  }
}

void sendMotorAttr(Stream& output, const Motor& m, const uint8_t reg) {
  switch (reg) {
    case MREG_STATE_FLAGS:
      output.write((uint8_t*) &m.state.values.flags, 1);
      break;
    case MREG_POSITION:
      output.write((uint8_t*) &m.state.values.pos, 4);
      break;
    case MREG_TARGET_POSITION:
      output.write((uint8_t*) &m.state.values.targetPos, 4);
      break;
    case MREG_SPEED:
      output.write((uint8_t*) &m.state.values.speed, 2);
      break;
    case MREG_SETTINGS_FLAGS:
      output.write((uint8_t*) &m.settings.values.flags, 1);
      break;
    case MREG_MAX_SPEED:
      output.write((uint8_t*) &m.settings.values.maxSpeed, 2);
      break;
    case MREG_ACCELERATION:
      output.write((uint8_t*) &m.settings.values.acceleration, 2);
      break;
  }
  // if (reg == MREG_STATE_FLAGS) {
  //   output.write((uint8_t*) &m.state.values.flags, 1);
  // } else if (reg == MREG_SETTINGS_FLAGS) {
  //   output.write((uint8_t*) &m.settings.values.flags, 1);
  // } else if (reg == MREG_POSITION) {
  //   output.write((uint8_t*) &m.state.values.pos, 4);
  // } else if (reg == MREG_MAX_SPEED) {
  //   output.write((uint8_t*) &m.settings.values.maxSpeed, 2);
  // } else if (reg == MREG_ACCELERATION) {
  //   output.write((uint8_t*) &m.settings.values.acceleration, 2);
  // } else if (reg == MREG_ABS_MAX_SPEED) {
  //   output.write((uint8_t*) &m.settings.values.absMaxSpeed, 2);
  // } else if (reg == MREG_MAX_ACCELERATION) {
  //   output.write((uint8_t*) &m.settings.values.maxAcceleration, 2);
  // } else if (reg == MREG_TARGET_POSITION) {
  //   output.write((uint8_t*) &m.state.values.targetPos, 4);
  // }
}

ResCode setMotorAttr(Motor& m, const uint8_t reg, const uint8_t value) {
  if (reg == MREG_SETTINGS_FLAGS) {
    m.setSettingsFlags(value);
  } else {
    return UNKNOWN_COMMAND;
  }
  return OK;
}

ResCode setMotorAttr(Motor& m, const uint8_t reg, const uint16_t value) {
  if (reg == MREG_MAX_SPEED) {
    m.setMaxSpeed(value);
  } else if (reg == MREG_ACCELERATION) {
    m.setAcceleration(value);
  } else {
    return UNKNOWN_COMMAND;
  }
  return OK;
}

ResCode setMotorAttr(Motor& m, const uint8_t reg, const uint32_t value) {
  if (reg == MREG_POSITION) {
    if (checkbit(m.state.values.flags, Motor::BitIsMoving)) {
      return MOTOR_BUSY;
    }
    m.setPosition(value);
  } else {
    return UNKNOWN_COMMAND;
  }
  return OK;
}

// void packLong(const unsigned long value, byte* buf) {
//   buf[0] = (byte)((value >> 0x18) & 0xff);
//   buf[1] = (byte)((value >> 0x10) & 0xff);
//   buf[2] = (byte)((value >> 0x8) & 0xff);
//   buf[3] = (byte)(value & 0xff);
// }

uint32_t unpackLong(uint8_t* buf) {
  return (
    buf[0] << 0x18 | buf[1] << 0x10 | buf[2] << 0x8 | buf[3]);
}

// void packShort(const uint16_t value, byte* buf) {
//   buf[0] = (byte)((value >> 0x8) & 0xff);
//   buf[1] = (byte)(value & 0xff);
// }

uint16_t unpackShort(uint8_t* buf) {
  return buf[0] << 0x8 | buf[1];
}

void slurp(Stream& stream) {
  while (stream.available()) {
    stream.read();
  }
}