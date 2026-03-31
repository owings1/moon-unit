#include "MotorState.h"

namespace MotorState {
void syncMotorSettings(IMotor* m, volatile Moic::MotorInterface& mregs) {
  if (mregs.settingsFlags != m->settingsFlags()) {
    mregs.settingsFlags = m->settingsFlags();
  }
  if (mregs.maxSpeed != m->maxSpeed()) {
    mregs.maxSpeed = m->maxSpeed();
  }
  if (mregs.acceleration != m->acceleration()) {
    mregs.acceleration = m->acceleration();
  }
  if (mregs.sleepTimeoutMs != m->sleepTimeoutMs()) {
    mregs.sleepTimeoutMs = m->sleepTimeoutMs();
  }
  if (mregs.enableDelayMs != m->enableDelayMs()) {
    mregs.enableDelayMs = m->enableDelayMs();
  }
}
void syncMotorState(IMotor* m, volatile Moic::MotorInterface& mregs) {
  mregs.stateFlags = m->stateFlags();
  mregs.currentPosition = m->currentPosition();
  mregs.targetPosition = m->targetPosition();
  mregs.speed = m->speed();
  if (mregs.waitEndTime != 0) {
    if (millis() >= mregs.waitEndTime) {
      mregs.waitEndTime = 0;
      m->setDelayActive(false);
    }
  }
}
uint8_t enterScript(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, uint8_t page, const uint8_t arg) {
  if (isMotorBusy(m, mregs) || isScriptActive(m, mregs, ctx)) {
    return Moic::MOTOR_BUSY;
  }
  if (page >= Moic::NUM_SCRIPT_PAGES) {
    return Moic::OVERFLOW;
  }
  mregs.scriptPage = page;
  mregs.scriptIdx = 0;
  mregs.scriptRepCode = Moic::OK;
  ctx.scriptCallArg = arg;
  ctx.scriptLastRhs = 0;
  m->setScriptActive(true);
  ctx._internalFlags |= 1 << Moic::BitIsScriptActive;
  return Moic::OK;
}
void exitScript(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, const uint8_t code) {
  ctx.sp = 0;
  mregs.scriptRepCode = code;
  m->setScriptActive(false);
  ctx._internalFlags &= ~(1 << Moic::BitIsScriptActive);
  mregs.waitEndTime = 0;
  m->setDelayActive(false);
  mregs.stateFlags = m->stateFlags();
}
bool isMotorBusy(IMotor* m, volatile Moic::MotorInterface& mregs) {
  return mregs.waitEndTime != 0 && millis() < mregs.waitEndTime || m->busy();
}
bool isScriptActive(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx) {
  return (ctx._internalFlags >> Moic::BitIsScriptActive) & 1;
}
bool isPageInStack(IMotor* m, volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, const uint8_t page) {
  if (!MotorState::isScriptActive(m, mregs, ctx)) {
    return false;
  }
  if (page == mregs.scriptPage) {
    return true;
  }
  for (uint8_t i = 0; i < ctx.sp; ++i) {
    if (page == ctx.scriptStackPage[i]) {
      return true;
    }
  }
  return false;
}
}