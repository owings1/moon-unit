#include <sys/_stdint.h>
#ifndef MOTOR_ACTIONS_H
#define MOTOR_ACTIONS_H

#include "MoicProtocol.h"
#include "IMotor.h"
#include "MotorState.h"
#include "MotorVM.h"

namespace MotorActions {
uint8_t onCurrentPosition(IMotor* m, volatile Moic::MotorBlock& mregs);
uint8_t onSettingsFlags(IMotor* m, volatile Moic::MotorBlock& mregs);
uint8_t onEnableDelayMs(IMotor* m, volatile Moic::MotorBlock& mregs);
uint8_t onSleepTimeout(IMotor* m, volatile Moic::MotorBlock& mregs);
uint8_t onMaxSpeed(IMotor* m, volatile Moic::MotorBlock& mregs);
uint8_t onAcceleration(IMotor* m, volatile Moic::MotorBlock& mregs);
uint8_t onMove(IMotor* m, volatile Moic::MotorBlock& mregs);
uint8_t onMoveTo(IMotor* m, volatile Moic::MotorBlock& mregs);
uint8_t onDelay(IMotor* m, volatile Moic::MotorBlock& mregs);
uint8_t onStop(IMotor* m, volatile Moic::MotorBlock& mregs);
uint8_t onScriptClear(IMotor* m, volatile Moic::MotorBlock& mregs);
uint8_t onScriptExec(IMotor* m, volatile Moic::MotorBlock& mregs);
uint8_t onCmdCall(IMotor* m, volatile Moic::MotorBlock& mregs);
uint8_t onCondCall(IMotor* m, volatile Moic::MotorBlock& mregs);
uint8_t onCondJump(IMotor* m, volatile Moic::MotorBlock& mregs);
uint8_t onCmdJump(IMotor* m, volatile Moic::MotorBlock& mregs);
uint8_t write(IMotor* m, volatile Moic::MotorBlock& mregs, const uint8_t offset, const uint8_t incoming, const bool enforceBusy, const bool enforceScriptLock);
void tickScript(IMotor* m, volatile Moic::MotorBlock& mregs);
}
struct RegMapping {
  uint8_t offset;
  uint8_t size;
  uint8_t (*handler)(IMotor* m, volatile Moic::MotorBlock& mregs);
};

const RegMapping ACTION_TABLE[] = {
  { 0x04, 4, MotorActions::onCurrentPosition },
  { 0x10, 1, MotorActions::onSettingsFlags },
  { 0x11, 1, MotorActions::onEnableDelayMs },
  { 0x12, 2, MotorActions::onSleepTimeout },
  { 0x14, 4, MotorActions::onMaxSpeed },
  { 0x18, 4, MotorActions::onAcceleration },
  { 0x24, 4, MotorActions::onDelay },
  { 0x1C, 4, MotorActions::onMove },
  { 0x20, 4, MotorActions::onMoveTo },
  { 0x28, 1, MotorActions::onStop },
  { 0x29, 1, MotorActions::onScriptClear },
  { 0x2A, 1, MotorActions::onScriptExec },
  { 0x2E, 2, MotorActions::onCmdCall },
  { 0x30, 4, MotorActions::onCondCall },
  { 0x34, 4, MotorActions::onCondJump },
  { 0x38, 2, MotorActions::onCmdJump },
};

const uint8_t MOTOR_REGS_COUNT = sizeof(ACTION_TABLE) / sizeof(RegMapping);
// Array of 120 pointers (one for every possible struct offset)
static const RegMapping* ACTION_LOOKUP[Moic::MOTOR_BLOCK_SIZE] = { nullptr };
void clearTrigger(volatile uint8_t* buf, uint8_t size);
#endif

