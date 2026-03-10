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

const byte numMotors = 2;

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
  byte runMask = 0;
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

volatile uint8_t wireReg1 = 0x0;
volatile uint8_t wireRes1 = 0x0;

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
  MREG_ABS_MAX_SPEED = 0x9,
  MREG_MAX_ACCELERATION = 0xa,
  MREG_TARGET_POSITION = 0xc,
  MREG_STOP = 0xf
};
void requestEvent() {
  if (wireReg1 == REG_UNSET) {
    return;
  }
  TwoWire& wire = I2C_MAIN;
  if (wireReg1 == REG_RESPONSE_CODE) {
    wireReg1 = REG_UNSET;
    wire.write(wireRes1);
  }
  // First 2 significant bits are category
  const byte category = wireReg1 >> 0x6;
  if (category == 0x1) {
    // Category 1: single motor attribute/data
    // Next 2 bits are motor id
    const byte motorId = ((wireReg1 >> 0x4) & 0x3) + 1;
    // Remaining 4 bits are motor attribute
    const byte motorReg = wireReg1 & 0xf;
    wireReg1 = REG_UNSET;
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
  TwoWire& wire = I2C_MAIN;
  wireReg1 = wire.read();
  // First 2 significant bits are category
  const byte category = wireReg1 >> 0x6;
  if (category == 0x1) {
    // Category 1: read/write single motor attribute/data
    // Next 2 bits are motor id
    const uint8_t motorId = ((wireReg1 >> 0x4) & 0x3) + 1;
    // Remaining 4 bits are command/attribute
    const uint8_t motorReg = wireReg1 & 0xf;
    if (howMany > 1 || motorReg == MREG_MOVE_CW || motorReg == MREG_MOVE_ACW || motorReg == MREG_STOP) {
      wireReg1 = REG_RESPONSE_CODE;
      if (motorReg == MREG_STATE_FLAGS || motorReg == MREG_TARGET_POSITION) {
        wireRes1 = READONLY_ATTRIBUTE;
        slurp(wire);
        return;
      }
      if (
        motorReg == MREG_SETTINGS_FLAGS ||
        motorReg == MREG_MAX_SPEED ||
        motorReg == MREG_ABS_MAX_SPEED ||
        motorReg == MREG_ACCELERATION ||
        motorReg == MREG_MAX_ACCELERATION ||
        motorReg == MREG_POSITION ||
        motorReg == MREG_MOVE_CW ||
        motorReg == MREG_MOVE_ACW ||
        motorReg == MREG_STOP
      ) {
        if (
          (motorReg == MREG_STOP) && howMany != 1 ||
          (motorReg == MREG_SETTINGS_FLAGS) && howMany != 2 ||
          (motorReg == MREG_MAX_SPEED || motorReg == MREG_ABS_MAX_SPEED || motorReg == MREG_ACCELERATION || motorReg == MREG_MAX_ACCELERATION) && howMany != 3 ||
          (motorReg == MREG_POSITION || motorReg == MREG_MOVE_CW || motorReg == MREG_MOVE_ACW) && howMany != 5
        ) {
          wireRes1 = MALFORMED_COMMAND;
          slurp(wire);
          return;
        }
        if (motorId > numMotors) {
          wireRes1 = INVALID_MOTORID;
          slurp(wire);
          return;
        }
        Motor& m = motors[motorId - 1];
        if (motorReg == MREG_STOP) {
          wireRes1 = m.stop() ? OK : COMMAND_IGNORED;
        } else if (motorReg == MREG_SETTINGS_FLAGS) {
          wireRes1 = setMotorAttr(m, motorReg, (uint8_t) wire.read());
        } else if (motorReg == MREG_POSITION || motorReg == MREG_MOVE_CW || motorReg == MREG_MOVE_ACW) {
          byte buf[4];
          wire.readBytes(buf, 4);
          const uint32_t value = unpackLong(buf);
          if (motorReg == MREG_POSITION) {
            wireRes1 = setMotorAttr(m, motorReg, value);
          } else if (motorReg == MREG_MOVE_CW) {
            wireRes1 = m.move(value) ? OK : COMMAND_IGNORED;
          } else if (motorReg == MREG_MOVE_ACW) {
            wireRes1 = m.move(-value) ? OK : COMMAND_IGNORED;
          }
        } else {
          byte buf[2];
          wire.readBytes(buf, 2);
          wireRes1 = setMotorAttr(m, motorReg, unpackShort(buf));
        }
      } else {
        wireRes1 = UNKNOWN_COMMAND;
        slurp(wire);
      }
    }
  } else {
    slurp(wire);
    wireReg1 = REG_UNSET;
  }
}

void sendMotorAttr(Stream& output, const Motor& m, const byte reg) {
  if (reg <= 0x1) {
    if (reg == 0x0) {
      output.write((uint8_t*)&m.state.values.flags, 1);
    } else if (reg == 0x1) {
      output.write((uint8_t*)&m.settings.values.flags, 1);
    }
  }
  if (reg <= 0x8) {
    if (reg == 0x2) {
      output.write((uint8_t*)&m.state.values.pos, 4);
    } else if (reg == 0x3) {
      output.write((uint8_t*)&m.settings.values.maxSpeed, 2);
    } else if (reg == 0x4) {
      output.write((uint8_t*)&m.settings.values.acceleration, 2);
    }
  } else {
    if (reg == 0x9) {
      output.write((uint8_t*)&m.settings.values.absMaxSpeed, 2);
    } else if (reg == 0xa) {
      output.write((uint8_t*)&m.settings.values.maxAcceleration, 2);
    } else if (reg == 0xc) {
      output.write((uint8_t*)&m.state.values.targetPos, 4);
    }
  }
}

ResCode setMotorAttr(Motor& m, const byte reg, const uint8_t value) {
  if (reg == MREG_SETTINGS_FLAGS) {
    m.setSettingsFlags(value);
  } else {
    return UNKNOWN_COMMAND;
  }
  return OK;
}

ResCode setMotorAttr(Motor& m, const byte reg, const uint16_t value) {
  if (reg == MREG_MAX_SPEED) {
    m.setMaxSpeed(value);
  } else if (reg == MREG_ACCELERATION) {
    m.setAcceleration(value);
  } else if (reg == MREG_ABS_MAX_SPEED) {
    m.setAbsMaxSpeed(value);
  } else if (reg == MREG_MAX_ACCELERATION) {
    m.setMaxAcceleration(value);
  } else {
    return UNKNOWN_COMMAND;
  }
  return OK;
}

ResCode setMotorAttr(Motor& m, const byte reg, const uint32_t value) {
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

uint32_t unpackLong(byte* buf) {
  return (
    buf[0] << 0x18 | buf[1] << 0x10 | buf[2] << 0x8 | buf[3]);
}

// void packShort(const uint16_t value, byte* buf) {
//   buf[0] = (byte)((value >> 0x8) & 0xff);
//   buf[1] = (byte)(value & 0xff);
// }

uint16_t unpackShort(byte* buf) {
  return buf[0] << 0x8 | buf[1];
}

void slurp(Stream& stream) {
  while (stream.available()) {
    stream.read();
  }
}