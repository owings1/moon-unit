#include <sys/_stdint.h>
#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

#include <Arduino.h>
#include "MoicProtocol.h"
#include "IMotor.h"

namespace Moic {
static const uint8_t MOTOR_BLOCK_SIZE = 0x40;
// Constraint: Needs to be no more than (0x100 - MOTOR_BASE_ADDR)
//             to support global registers on all pages.
static const uint8_t SCRIPT_PAGE_SIZE = 0xF8;
// Constraint: Needs to be no more than 0x0D for I2C paging logic.
static const uint8_t NUM_SCRIPT_PAGES = 0x08;
// Constraint: Needs to be less than (0x80 - VARPTR_START) to support
//             var ptr references with INDIRECT_OPCODE_FLAG 0x40 but
//             without FARPTR_OPCODE_FLAG 0x80.
static const uint8_t NUM_SCRIPT_GLOBAL_VARS = 0x04;
static const uint8_t SCRIPT_STACK_SIZE = 0x08;
// Constraint: Must be greater than max return value of getOpCodeDataLength()
static const uint8_t SCRIPT_WRITEBUF_SIZE = 8;
static const uint8_t BUSY_EXEMPT_MASK = 0x80;
static const uint8_t INDIRECT_OPCODE_FLAG = 0x40;
static const uint8_t FARPTR_OPCODE_FLAG = 0x80;
static const uint8_t CONTROL_EXCODE = 0xC0;
// Constraint: Must be less than VARPTR_START
static const uint8_t INDPTR_END = 0x1C;
// Constraint: Must be greater than INDPTR_END
static const uint8_t VARPTR_START = 0x60;
#pragma pack(push, 1)
struct MotorInterface {
  // 0x00 - Telemetry (Aligned)
  volatile uint8_t stateFlags;  // +0x00
  volatile uint8_t scriptPage;       // +0x01
  volatile uint8_t scriptIdx;        // +0x02
  volatile uint8_t scriptRepCode;    // +0x03
  volatile int32_t currentPosition;  // +0x04
  volatile int32_t targetPosition;   // +0x08
  volatile float speed;              // +0x0C
  // 0x10 - Configuration
  volatile uint8_t settingsFlags;    // +0x10
  volatile uint8_t enableDelayMs;    // +0x11
  volatile uint16_t sleepTimeoutMs;  // +0x12
  volatile float maxSpeed;           // +0x14
  volatile float acceleration;       // +0x18
  // 0x1C - Actions
  volatile int32_t cmdMove;         // +0x1C
  volatile int32_t cmdMoveTo;       // +0x20
  volatile uint32_t cmdDelay;       // +0x24
  volatile uint8_t cmdStop;         // +0x28
  volatile uint8_t cmdScriptClear;  // +0x29
  volatile uint8_t cmdScriptExec[2];// +0x2A
  volatile uint32_t waitEndTime;    // +0x2C
  volatile int32_t cmdMoveRev;      // +0x30
  uint8_t _pad0[0x0C];
};
struct VMStack {
  uint8_t page;
  uint8_t idx;
  uint8_t condArg;
  uint8_t callArg;
};
struct VMContext {
  volatile uint8_t page;
  volatile uint8_t idx;
  volatile uint8_t condArg;
  volatile uint8_t callArg;
  int32_t compArg;
  uint8_t sp;      // Stack Pointer
  uint8_t offset;  // offset for write() callback
  uint8_t count;   // byte count stored in writeBuf for write() callback
  uint8_t exitCode;
  volatile uint8_t scripts[NUM_SCRIPT_PAGES][SCRIPT_PAGE_SIZE];
  VMStack stack[SCRIPT_STACK_SIZE];
  uint8_t writeBuf[SCRIPT_WRITEBUF_SIZE];
  int32_t vars[NUM_SCRIPT_GLOBAL_VARS];
};
#pragma pack(pop)

}
#endif
