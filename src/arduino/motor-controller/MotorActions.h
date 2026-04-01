#include <sys/_stdint.h>
#ifndef MOTOR_ACTIONS_H
#define MOTOR_ACTIONS_H

#include "MoicProtocol.h"
#include "IMotor.h"
#include "MotorState.h"
#include "MotorVM.h"
#include "MotorManager.h"

namespace MotorActions {
typedef uint8_t (*ActionHandler)(Moic::ManagedMotor& mm);
uint8_t onCurrentPosition(Moic::ManagedMotor& mm);
uint8_t onSettingsFlags(Moic::ManagedMotor& mm);
uint8_t onEnableDelayMs(Moic::ManagedMotor& mm);
uint8_t onSleepTimeout(Moic::ManagedMotor& mm);
uint8_t onMaxSpeed(Moic::ManagedMotor& mm);
uint8_t onAcceleration(Moic::ManagedMotor& mm);
uint8_t onMove(Moic::ManagedMotor& mm);
uint8_t onMoveTo(Moic::ManagedMotor& mm);
uint8_t onDelay(Moic::ManagedMotor& mm);
uint8_t onStop(Moic::ManagedMotor& mm);
uint8_t onScriptClear(Moic::ManagedMotor& mm);
uint8_t onScriptExec(Moic::ManagedMotor& mm);
uint8_t onCmdCall(Moic::ManagedMotor& mm);
uint8_t onCondCall(Moic::ManagedMotor& mm);
uint8_t onCondJump(Moic::ManagedMotor& mm);
uint8_t onCmdJump(Moic::ManagedMotor& mm);
uint8_t onMoveRev(Moic::ManagedMotor& mm);
struct RegMapping {
  uint8_t offset;
  uint8_t size;
  ActionHandler handler;
};

const RegMapping ACTION_TABLE[] = {
  { offsetof(Moic::MotorInterface, currentPosition), 4, onCurrentPosition },
  { offsetof(Moic::MotorInterface, settingsFlags), 1, onSettingsFlags },
  { offsetof(Moic::MotorInterface, enableDelayMs), 1, onEnableDelayMs },
  { offsetof(Moic::MotorInterface, sleepTimeoutMs), 2, onSleepTimeout },
  { offsetof(Moic::MotorInterface, maxSpeed), 4, onMaxSpeed },
  { offsetof(Moic::MotorInterface, acceleration), 4, onAcceleration },
  { offsetof(Moic::MotorInterface, cmdMove), 4, onMove },
  { offsetof(Moic::MotorInterface, cmdMoveTo), 4, onMoveTo },
  { offsetof(Moic::MotorInterface, cmdDelay), 4, onDelay },
  { offsetof(Moic::MotorInterface, cmdStop), 1, onStop },
  { offsetof(Moic::MotorInterface, cmdScriptClear), 1, onScriptClear },
  { offsetof(Moic::MotorInterface, cmdScriptExec), 2, onScriptExec },
  { offsetof(Moic::MotorInterface, cmdCall), 2, onCmdCall },
  { offsetof(Moic::MotorInterface, cmdCondCall), 4, onCondCall },
  { offsetof(Moic::MotorInterface, cmdCondJump), 4, onCondJump },
  { offsetof(Moic::MotorInterface, cmdJump), 2, onCmdJump },
  { offsetof(Moic::MotorInterface, cmdMoveRev), 4, onMoveRev },
};

// Array of 64 pointers (one for every possible struct offset)
static const RegMapping* ACTION_LOOKUP[Moic::MOTOR_BLOCK_SIZE] = { nullptr };
}
void clearTrigger(volatile uint8_t* buf, uint8_t size);
#endif
