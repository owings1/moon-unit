#include <sys/_stdint.h>
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
  INVALID_COMMAND = 0x2C,
  INVALID_MOTOR = 0x2D,
  COMMAND_IGNORED = 0x2E,
  UNINVITED_POINTER = 0x2F,
  READONLY_ATTRIBUTE = 0x30,
  OVERFLOW = 0x31,
  INVALID_CTLOP = 0x32,
  INVALID_OPFLAG = 0x33,
  INVALID_FUNID = 0x34,
  INVALID_MATHOPER = 0x35,
  USR1 = 0xFA,
  USR2 = 0xFB,
  USR3 = 0xFC,
  USR4 = 0xFD,
  USR5 = 0xFE,
  UNSET = 0xFF,
};

enum FunId : uint8_t {
  // Constraint:
  // This is matched to attribute register stateFlags for auto-flag logic
  // in the Python compiler.
  AND_STATEFLAGS_RHS = 0x00,
  EQL_RETURNCODE_RHS = 0x03,
  // Constraint:
  // This is matched to attribute register settingsFlags for auto-flag logic
  // in the Python compiler.
  AND_SETTINGSFLAGS_RHS = 0x10,
  // 0x20-0x60: Normal Predicates
  //  Lower Bits:
  //    0b00 EQL
  //    0b01 LT
  //    0b10 AND
  //    0b11 LTE
  P_CALLARG_EQL = 0x20,
  P_CALLARG_AND = 0x22,
  // -----------------------
  // Constraint:
  //
  // P_LASTCONDARG and P_LASTCOMPARG IDs must occupy a contiguous block,
  // meaning no other functions must lie between them, and the high & low values
  // reflected in MotorVM condition() to be excluded from updating the last
  // condArg RHS value.
  P_LASTCONDARG_EQL = 0x24,
  P_LASTCONDARG_AND = 0x26,
  P_LASTCOMPARG_EQL = 0x28,
  P_LASTCOMPARG_LT = 0x29,
  P_LASTCOMPARG_LTE = 0x2B,

  ALWAYS_TRUE = 0x7F,
};

enum Ctrl : uint8_t {
  SET_VAR = 0x01,
  VAR_MATH1 = 0x02,
  VAR_MATH2 = 0x03,
  CALL = 0x08,
  JUMP = 0x09,
  COND_CALL = 0x0A,
  COND_JUMP = 0x0B,
};

enum Math1Oper : uint8_t {
  MATH1_INC = 0x01, // Increment, x + 1
  MATH1_DEC = 0x02, // Decrement, x - 1
  MATH1_NEG = 0x03, // Negative, x * -1
  MATH1_HLF = 0x04, // Half, x/2
  MATH1_DBL = 0x05, // Double x * 2
};
enum Math2Oper : uint8_t {
  MATH2_ADD = 0x01,
  MATH2_SUB = 0x02,
  MATH2_MUL = 0x03,
  MATH2_CMP = 0x04,
  // SAFEDIV returns 0 when divide by zero is attempted, because C++ makes it
  // too expensive to add a conditional branch to return the error.
  MATH2_SAFEDIV = 0x05,
};
enum WriteSource : uint8_t {
  VMEXC = 0x00,
  BUSIO = 0x01,
};
}

#endif
