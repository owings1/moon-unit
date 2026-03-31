#include <sys/_stdint.h>

#include "MotorActions.h"

namespace MotorActions {
uint8_t onCurrentPosition(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx) {
  m->setCurrentPosition(mregs.currentPosition);
  mregs.currentPosition = m->currentPosition();
  return Moic::OK;
}
uint8_t onSettingsFlags(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx) {
  m->setSettingsFlags(mregs.settingsFlags);
  mregs.settingsFlags = m->settingsFlags();
  return Moic::OK;
}
uint8_t onEnableDelayMs(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx) {
  m->setEnableDelayMs(mregs.enableDelayMs);
  mregs.enableDelayMs = m->enableDelayMs();
  return Moic::OK;
}
uint8_t onSleepTimeout(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx) {
  m->setSleepTimeoutMs(mregs.sleepTimeoutMs);
  mregs.sleepTimeoutMs = m->sleepTimeoutMs();
  return Moic::OK;
}
uint8_t onMaxSpeed(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx) {
  m->setMaxSpeed(mregs.maxSpeed);
  mregs.maxSpeed = m->maxSpeed();
  return Moic::OK;
}
uint8_t onAcceleration(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx) {
  m->setAcceleration(mregs.acceleration);
  mregs.acceleration = m->acceleration();
  return Moic::OK;
}
uint8_t onMove(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx) {
  const uint8_t code = m->move(mregs.cmdMove) ? Moic::OK : Moic::COMMAND_IGNORED;
  mregs.cmdMove = 0;
  return code;
}
uint8_t onMoveRev(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx) {
  const uint8_t code = m->move(-mregs.cmdMoveRev) ? Moic::OK : Moic::COMMAND_IGNORED;
  mregs.cmdMoveRev = 0;
  return code;
}
uint8_t onMoveTo(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx) {
  const uint8_t code = m->move(mregs.cmdMoveTo - m->currentPosition()) ? Moic::OK : Moic::COMMAND_IGNORED;
  mregs.cmdMoveTo = 0;
  return code;
}
uint8_t onDelay(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx) {
  if (mregs.cmdDelay > 0) {
    mregs.waitEndTime = millis() + mregs.cmdDelay;
    m->setDelayActive(true);
  } else {
    mregs.waitEndTime = 0;
    m->setDelayActive(false);
  }
  mregs.cmdDelay = 0;
  return Moic::OK;
}
uint8_t onStop(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx) {
  uint8_t code = m->stop() ? Moic::OK : Moic::COMMAND_IGNORED;
  if (MotorState::isScriptActive(m, mregs, ctx)) {
    MotorState::exitScript(m, mregs, ctx, Moic::CANCELED);
    code = Moic::OK;
  }
  mregs.stateFlags = m->stateFlags();
  mregs.cmdStop = 0;
  return code;
}
uint8_t onScriptClear(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx) {
  uint8_t code = Moic::OK;
  if (mregs.cmdScriptClear < Moic::NUM_SCRIPT_PAGES) {
    if (MotorState::isPageInStack(m, mregs, ctx, mregs.cmdScriptClear)) {
      // Prevent clearing running script
      code = Moic::MOTOR_BUSY;
    } else {
      memset((void*)ctx.scripts[mregs.cmdScriptClear], 0, Moic::SCRIPT_PAGE_SIZE);
    }
  } else {
    code = Moic::OVERFLOW;
  }
  mregs.cmdScriptClear = 0;
  return code;
}
uint8_t onScriptExec(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx) {
  const uint8_t code = MotorState::enterScript(m, mregs, ctx, mregs.cmdScriptExec[0], mregs.cmdScriptExec[1]);
  clearTrigger(mregs.cmdScriptExec, 2);
  return code;
}
uint8_t onCmdCall(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx) {
  uint8_t code = Moic::OK;
  if (!MotorState::isScriptActive(m, mregs, ctx)) {
    code = Moic::UNKNOWN_COMMAND;
  } else {
    code = MotorVM::call(mregs, ctx, mregs.cmdCall);
  }
  clearTrigger(mregs.cmdCall, 2);
  return code;
}
uint8_t onCmdJump(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx) {
  uint8_t code = Moic::OK;
  if (!MotorState::isScriptActive(m, mregs, ctx)) {
    code = Moic::UNKNOWN_COMMAND;
  } else {
    code = MotorVM::jump(mregs, ctx, mregs.cmdJump);
  }
  clearTrigger(mregs.cmdJump, 2);
  return code;
}
uint8_t onCondCall(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx) {
  uint8_t code = Moic::OK;
  if (!MotorState::isScriptActive(m, mregs, ctx)) {
    code = Moic::UNKNOWN_COMMAND;
  } else {
    code = MotorVM::condCall(mregs, ctx, mregs.cmdCondCall);
  }
  clearTrigger(mregs.cmdCondCall, 4);
  return code;
}
uint8_t onCondJump(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx) {
  uint8_t code = Moic::OK;
  if (!MotorState::isScriptActive(m, mregs, ctx)) {
    code = Moic::UNKNOWN_COMMAND;
  } else {
    code = MotorVM::condJump(mregs, ctx, mregs.cmdCondJump);
  }
  clearTrigger(mregs.cmdCondJump, 4);
  return code;
}

}

void clearTrigger(volatile uint8_t* buf, uint8_t size) {
  for (uint8_t i = 0; i < size; i++) buf[i] = 0;
}
