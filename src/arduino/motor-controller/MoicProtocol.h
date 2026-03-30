#ifndef MOIC_PROTOCOL_H
#define MOIC_PROTOCOL_H
#include <stddef.h>

#include <Arduino.h>
namespace Moic {
enum ResCode : uint8_t {
  OK = 0x00,
  OTHER_ERROR = 0x07,
  MOTOR_BUSY = 0x1F,
  CANCELED = 0x20,
  UNKNOWN_COMMAND = 0x2C,
  INVALID_MOTOR = 0x2D,
  COMMAND_IGNORED = 0x2E,
  UNINVITED_POINTER = 0x2F,
  READONLY_ATTRIBUTE = 0x30,
  OVERFLOW = 0x31,
  USR1 = 0xFA,
  USR2 = 0xFB,
  USR3 = 0xFC,
  USR4 = 0xFD,
  USR5 = 0xFE,
  UNSET = 0xFF,
};

enum FunId : uint8_t {
  AND_STATEFLAGS_RHS = 0x00,
  EQL_RETURNCODE_RHS = 0x03,
  AND_SETTINGSFLAGS_RHS = 0x10,
  EQL_LASTCONDARG_RHS = 0x30,
  AND_LASTCONDARG_RHS = 0x30 + 1,
  ALWAYS_TRUE = 0x7F,
};

enum InternalFlagBit : uint8_t {
  BitIsScriptActive = 0,
};
}
#endif
