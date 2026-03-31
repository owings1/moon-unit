#include <sys/_stdint.h>
#ifndef MOTOR_ACTIONS_H
#define MOTOR_ACTIONS_H

#include "MoicProtocol.h"
#include "IMotor.h"
#include "MotorState.h"
#include "MotorVM.h"

namespace MotorActions {
typedef uint8_t (*ActionHandler)(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx);
uint8_t onCurrentPosition(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx);
uint8_t onSettingsFlags(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx);
uint8_t onEnableDelayMs(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx);
uint8_t onSleepTimeout(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx);
uint8_t onMaxSpeed(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx);
uint8_t onAcceleration(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx);
uint8_t onMove(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx);
uint8_t onMoveTo(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx);
uint8_t onDelay(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx);
uint8_t onStop(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx);
uint8_t onScriptClear(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx);
uint8_t onScriptExec(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx);
uint8_t onCmdCall(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx);
uint8_t onCondCall(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx);
uint8_t onCondJump(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx);
uint8_t onCmdJump(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx);
uint8_t onMoveRev(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx);
void tickScript(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx);
uint8_t write(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, const uint8_t offset, const uint8_t incoming, const bool enforceBusy, const bool enforceScriptLock);
}
struct RegMapping {
  uint8_t offset;
  uint8_t size;
  MotorActions::ActionHandler handler;
};

const RegMapping ACTION_TABLE[] = {
  { offsetof(Moic::MotorInterface, currentPosition), 4, MotorActions::onCurrentPosition },
  { offsetof(Moic::MotorInterface, settingsFlags), 1, MotorActions::onSettingsFlags },
  { offsetof(Moic::MotorInterface, enableDelayMs), 1, MotorActions::onEnableDelayMs },
  { offsetof(Moic::MotorInterface, sleepTimeoutMs), 2, MotorActions::onSleepTimeout },
  { offsetof(Moic::MotorInterface, maxSpeed), 4, MotorActions::onMaxSpeed },
  { offsetof(Moic::MotorInterface, acceleration), 4, MotorActions::onAcceleration },
  { offsetof(Moic::MotorInterface, cmdMove), 4, MotorActions::onMove },
  { offsetof(Moic::MotorInterface, cmdMoveTo), 4, MotorActions::onMoveTo },
  { offsetof(Moic::MotorInterface, cmdDelay), 4, MotorActions::onDelay },
  { offsetof(Moic::MotorInterface, cmdStop), 1, MotorActions::onStop },
  { offsetof(Moic::MotorInterface, cmdScriptClear), 1, MotorActions::onScriptClear },
  { offsetof(Moic::MotorInterface, cmdScriptExec), 2, MotorActions::onScriptExec },
  { offsetof(Moic::MotorInterface, cmdCall), 2, MotorActions::onCmdCall },
  { offsetof(Moic::MotorInterface, cmdCondCall), 4, MotorActions::onCondCall },
  { offsetof(Moic::MotorInterface, cmdCondJump), 4, MotorActions::onCondJump },
  { offsetof(Moic::MotorInterface, cmdJump), 2, MotorActions::onCmdJump },
  { offsetof(Moic::MotorInterface, cmdMoveRev), 4, MotorActions::onMoveRev },
};

// const uint8_t MOTOR_REGS_COUNT = sizeof(ACTION_TABLE) / sizeof(RegMapping);
// Array of 120 pointers (one for every possible struct offset)
static const RegMapping* ACTION_LOOKUP[Moic::MOTOR_BLOCK_SIZE] = { nullptr };
void clearTrigger(volatile uint8_t* buf, uint8_t size);
#endif
