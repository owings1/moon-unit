#include <sys/_stdint.h>
#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

#include <Arduino.h>
#include "MoicProtocol.h"
#include "IMotor.h"

namespace Moic {
static const uint8_t MOTOR_BLOCK_SIZE = 0x78;
static const uint8_t SCRIPT_PAGE_SIZE = 0xF8;
static const uint8_t NUM_SCRIPT_PAGES = 12;
static const uint8_t SCRIPT_STACK_SIZE = 0x10;
static const uint8_t SCRIPT_WRITEBUF_SIZE = 8;
// Busy Protected:
// current_position   | offset: 0x04 | span: 4 bytes
// settings_flags     | offset: 0x10 | span: 1 bytes
// max_speed          | offset: 0x14 | span: 4 bytes
// acceleration       | offset: 0x18 | span: 4 bytes
// move               | offset: 0x1c | span: 4 bytes
// move_to            | offset: 0x20 | span: 4 bytes
// delay              | offset: 0x24 | span: 4 bytes
// script_exec        | offset: 0x2a | span: 2 bytes
// move_rev           | offset: 0x3c | span: 4 bytes
static const uint64_t MOTOR_BUSY_MASK = 0xF0000CFFFFF100F0ULL;
// Script Protected:
// current_position   | offset: 0x04 | span: 4 bytes
// settings_flags     | offset: 0x10 | span: 1 bytes
// enable_delay_ms    | offset: 0x11 | span: 1 bytes
// sleep_timeout_ms   | offset: 0x12 | span: 2 bytes
// max_speed          | offset: 0x14 | span: 4 bytes
// acceleration       | offset: 0x18 | span: 4 bytes
// move               | offset: 0x1c | span: 4 bytes
// move_to            | offset: 0x20 | span: 4 bytes
// delay              | offset: 0x24 | span: 4 bytes
// script_exec        | offset: 0x2a | span: 2 bytes
// move_rev           | offset: 0x3c | span: 4 bytes
static const uint64_t SCRIPT_LOCK_MASK = 0xF0000CFFFFFF00F0ULL;
#pragma pack(push, 1)
struct MotorBlock {
  // 0x00 - Telemetry (Aligned)
  volatile uint8_t stateFlags;  // +0x00
  uint8_t _pad0[2];
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
  volatile uint8_t scriptPage;      // +0x2C
  volatile uint8_t scriptIdx;       // +0x2D
  volatile uint8_t cmdCall[2];      // +0x2E
  volatile uint8_t cmdCondCall[4];  // +0x30
  volatile uint8_t cmdCondJump[4];  // +0x34
  volatile uint8_t cmdJump[2];      // +0x38
  uint8_t _pad2[2];
  volatile int32_t cmdMoveRev;      // +0x3C

  // 0x40 - (Future)
  uint8_t _unallocated[0x38];  // +0x40 - 0x77 future
  // 0x78 - Script buffer
  volatile uint8_t scripts[NUM_SCRIPT_PAGES][SCRIPT_PAGE_SIZE];  // 248 bytes. 12 pages = 2976 bytes per motor
  volatile uint32_t _waitEndTime;
  volatile uint8_t scriptStackIdx[SCRIPT_STACK_SIZE];
  volatile uint8_t scriptStackPage[SCRIPT_STACK_SIZE];
  volatile uint8_t scriptStackRhsArg[SCRIPT_STACK_SIZE];
  volatile uint8_t scriptStackCallArg[SCRIPT_STACK_SIZE];
  volatile uint8_t sp;  // Stack Pointer
  volatile uint8_t _internalFlags;
  volatile uint8_t scriptLastRhs;
  volatile uint8_t scriptCallArg;
  // uint8_t _pad3[1];
  volatile uint8_t scriptWriteBuf[SCRIPT_WRITEBUF_SIZE];
};

#pragma pack(pop)
}

namespace MotorState {
void syncMotorSettings(IMotor* m, volatile Moic::MotorBlock& mregs);
void syncMotorState(IMotor* m, volatile Moic::MotorBlock& mregs);
uint8_t enterScript(IMotor* m, volatile Moic::MotorBlock& mregs, uint8_t page, const uint8_t arg);
void exitScript(IMotor* m, volatile Moic::MotorBlock& mregs, const uint8_t code);
bool isMotorBusy(IMotor* m, volatile Moic::MotorBlock& mregs);
bool isPageInStack(IMotor* m, volatile Moic::MotorBlock& mregs, const uint8_t page);
bool isScriptActive(IMotor* m, volatile Moic::MotorBlock& mregs);
}
#endif
