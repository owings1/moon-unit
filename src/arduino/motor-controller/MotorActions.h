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
uint8_t onMoveRev(IMotor* m, volatile Moic::MotorBlock& mregs);
uint8_t write(IMotor* m, volatile Moic::MotorBlock& mregs, const uint8_t offset, const uint8_t incoming, const bool enforceBusy, const bool enforceScriptLock);
void tickScript(IMotor* m, volatile Moic::MotorBlock& mregs);
}
struct RegMapping {
  uint8_t offset;
  uint8_t size;
  uint8_t (*handler)(IMotor* m, volatile Moic::MotorBlock& mregs);
};

const RegMapping ACTION_TABLE[] = {
  { offsetof(Moic::MotorBlock, currentPosition), 4, MotorActions::onCurrentPosition },
  { offsetof(Moic::MotorBlock, settingsFlags), 1, MotorActions::onSettingsFlags },
  { offsetof(Moic::MotorBlock, enableDelayMs), 1, MotorActions::onEnableDelayMs },
  { offsetof(Moic::MotorBlock, sleepTimeoutMs), 2, MotorActions::onSleepTimeout },
  { offsetof(Moic::MotorBlock, maxSpeed), 4, MotorActions::onMaxSpeed },
  { offsetof(Moic::MotorBlock, acceleration), 4, MotorActions::onAcceleration },
  { offsetof(Moic::MotorBlock, cmdMove), 4, MotorActions::onMove },
  { offsetof(Moic::MotorBlock, cmdMoveTo), 4, MotorActions::onMoveTo },
  { offsetof(Moic::MotorBlock, cmdDelay), 4, MotorActions::onDelay },
  { offsetof(Moic::MotorBlock, cmdStop), 1, MotorActions::onStop },
  { offsetof(Moic::MotorBlock, cmdScriptClear), 1, MotorActions::onScriptClear },
  { offsetof(Moic::MotorBlock, cmdScriptExec), 2, MotorActions::onScriptExec },
  { offsetof(Moic::MotorBlock, cmdCall), 2, MotorActions::onCmdCall },
  { offsetof(Moic::MotorBlock, cmdCondCall), 4, MotorActions::onCondCall },
  { offsetof(Moic::MotorBlock, cmdCondJump), 4, MotorActions::onCondJump },
  { offsetof(Moic::MotorBlock, cmdJump), 2, MotorActions::onCmdJump },
  { offsetof(Moic::MotorBlock, cmdMoveRev), 4, MotorActions::onMoveRev },
};

const uint8_t MOTOR_REGS_COUNT = sizeof(ACTION_TABLE) / sizeof(RegMapping);
// Array of 120 pointers (one for every possible struct offset)
static const RegMapping* ACTION_LOOKUP[Moic::MOTOR_BLOCK_SIZE] = { nullptr };
void clearTrigger(volatile uint8_t* buf, uint8_t size);
#endif

