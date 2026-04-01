#include <sys/_stdint.h>
#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

#include <Arduino.h>
#include "MoicProtocol.h"
#include "IMotor.h"

namespace Moic {
static const uint8_t MOTOR_BLOCK_SIZE = 0x40;
static const uint8_t SCRIPT_PAGE_SIZE = 0xF8;
static const uint8_t NUM_SCRIPT_PAGES = 0x04;
static const uint8_t SCRIPT_STACK_SIZE = 0x08;
static const uint8_t SCRIPT_WRITEBUF_SIZE = 4;
static const uint8_t BUSY_EXEMPT_MASK = 0x80;
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
  volatile uint8_t cmdCall[2];      // +0x30
  volatile uint8_t cmdCondCall[4];  // +0x32
  volatile uint8_t cmdCondJump[4];  // +0x36
  volatile uint8_t cmdJump[2];      // +0x3A
  volatile int32_t cmdMoveRev;      // +0x3C
};
struct MotorContext {
  volatile uint8_t _internalFlags;
  volatile uint8_t sp;  // Stack Pointer
  volatile uint8_t scriptLastRhs;
  volatile uint8_t scriptCallArg;
  volatile uint8_t scripts[NUM_SCRIPT_PAGES][SCRIPT_PAGE_SIZE];
  volatile uint8_t scriptStackIdx[SCRIPT_STACK_SIZE];
  volatile uint8_t scriptStackPage[SCRIPT_STACK_SIZE];
  volatile uint8_t scriptStackRhsArg[SCRIPT_STACK_SIZE];
  volatile uint8_t scriptStackCallArg[SCRIPT_STACK_SIZE];
  volatile uint8_t scriptWriteBuf[SCRIPT_WRITEBUF_SIZE];
};

#pragma pack(pop)

}
#endif
