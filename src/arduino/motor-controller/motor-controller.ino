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

volatile byte wireReg1 = 0x0;
volatile byte wireRes1 = 0x0;

void requestEvent() {
  if (wireReg1 == 0x0) {
    return;
  }
  TwoWire& wire = I2C_MAIN;
  if (wireReg1 == 0x1) {
    wireReg1 = 0x0;
    wire.write(wireRes1);
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
  TwoWire& wire = I2C_MAIN;
  wireReg1 = wire.read();
  // First 2 significant bits are category
  const byte category = wireReg1 >> 0x6;
  if (category == 0x1) {
    // Category 1: read/write single motor attribute/data
    // Next 2 bits are motor id
    const byte motorId = ((wireReg1 >> 0x4) & 0x3) + 1;
    // Remaining 4 bits are command/attribute
    const byte motorReg = wireReg1 & 0xf;
    if (howMany > 1) {
      wireReg1 = 0x1;
      if (motorReg >= 0x1 && motorReg <= 0xa) {
        if (motorReg == 0x1) {
          if (howMany == 2) {
            if (motorId > numMotors) {
              wireRes1 = INVALID_MOTORID;
              while (wire.available()) {
                wire.read();
              }
              return;
            }
            const uint8_t value = wire.read();
            wireRes1 = setMotorAttr(motors[motorId - 1], motorReg, value);
            return;
          } else {
            wireRes1 = MALFORMED_COMMAND;
            while (wire.available()) {
              wire.read();
            }
            return;
          }
        } else if (motorReg == 0x2) {
          if (howMany == 5) {
            if (motorId > numMotors) {
              wireRes1 = INVALID_MOTORID;
              while (wire.available()) {
                wire.read();
              }
              return;
            }
            byte buf[4];
            wire.readBytes(buf, 4);
            const uint32_t value = unpackLong(buf);
            wireRes1 = setMotorAttr(motors[motorId - 1], motorReg, value);
            return;
          } else {
            wireRes1 = MALFORMED_COMMAND;
            while (wire.available()) {
              wire.read();
            }
            return;
          }
        } else {
          if (howMany == 3) {
            if (motorId > numMotors) {
              wireRes1 = INVALID_MOTORID;
              while (wire.available()) {
                wire.read();
              }
              return;
            }
            byte buf[2];
            wire.readBytes(buf, 2);
            const uint16_t value = unpackShort(buf);
            wireRes1 = setMotorAttr(motors[motorId - 1], motorReg, value);
            return;
          } else {
            wireRes1 = MALFORMED_COMMAND;
            while (wire.available()) {
              wire.read();
            }
            return;
          }
        }
      } else {
        wireRes1 = READONLY_ATTRIBUTE;
        while (wire.available()) {
          wire.read();
        }
        return;
      }
    }
  } else if (category == 0x2) {
    // Category 2: single motor operation
    // Next 2 bits are motor id
    const byte motorId = ((wireReg1 >> 0x4) & 0x3) + 1;
    // Remaining 4 bits are command/attribute
    const byte motorReg = wireReg1 & 0xf;
    if (motorReg <= 0x6 && howMany > 1 || motorReg > 0x6 && motorReg <= 0xc && howMany != 5) {
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
      wireRes1 = UNKNOWN_COMMAND;
    }
  } else if (category == 0x3) {
    while (wire.available()) {
      wire.read();
    }
    wireRes1 = UNKNOWN_COMMAND;
    return;
  }
}

void writeMotorReg(Stream& output, const Motor& m, const byte reg) {
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
  if (reg == 0x1) {
    m.setSettingsFlags(value);
  } else {
    return UNKNOWN_COMMAND;
  }
  return OK;
}

ResCode setMotorAttr(Motor& m, const byte reg, const uint16_t value) {
  if (reg == 0x3) {
    m.setMaxSpeed(value);
  } else if (reg == 0x4) {
    m.setAcceleration(value);
  } else if (reg == 0x9) {
    m.setAbsMaxSpeed(value);
  } else if (reg == 0xa) {
    m.setMaxAcceleration(value);
  } else {
    return UNKNOWN_COMMAND;
  }
  return OK;
}

ResCode setMotorAttr(Motor& m, const byte reg, const uint32_t value) {
  if (reg == 0x2) {
    if ((m.state.values.flags >> Motor::BitIsMoving) & 1) {
      return MOTOR_BUSY;
    }
    m.setPosition(value);
  } else {
    return UNKNOWN_COMMAND;
  }
  return OK;
}

ResCode applyMotorReg(Motor& m, const byte reg) {
  if (reg == 0x0) {
    if (!m.stop()) {
      return COMMAND_IGNORED;
    }
  } else {
    return UNKNOWN_COMMAND;
  }
  return OK;
}

ResCode applyMotorReg(Motor& m, const byte reg, const unsigned long value) {
  if ((m.state.values.flags >> Motor::BitIsMoving) & 1) {
    return MOTOR_BUSY;
  }
  if (reg == 0x8) {
    if (!m.move(value)) {
      return COMMAND_IGNORED;
    }
  } else if (reg == 0x9) {
    if (!m.move(-value)) {
      return COMMAND_IGNORED;
    }
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

unsigned long unpackLong(byte* buf) {
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
