#include "MotorState.h"

namespace MotorState {
void syncMotorSettings(IMotor* m, volatile Moic::MotorBlock& mregs) {
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
void syncMotorState(IMotor* m, volatile Moic::MotorBlock& mregs) {
  mregs.stateFlags = m->stateFlags();
  mregs.currentPosition = m->currentPosition();
  mregs.targetPosition = m->targetPosition();
  mregs.speed = m->speed();
  if (mregs._waitEndTime != 0) {
    if (millis() >= mregs._waitEndTime) {
      mregs._waitEndTime = 0;
      m->setDelayActive(false);
    }
  }
}
uint8_t enterScript(IMotor* m, volatile Moic::MotorBlock& mregs, uint8_t page, const uint8_t arg) {
  if (isMotorBusy(m, mregs) || isScriptActive(m, mregs)) {
    return Moic::MOTOR_BUSY;
  }
  if (page >= Moic::NUM_SCRIPT_PAGES) {
    return Moic::OVERFLOW;
  }
  mregs.scriptPage = page;
  mregs.scriptIdx = 0;
  mregs.scriptRepCode = Moic::OK;
  mregs.scriptCallArg = arg;
  mregs.scriptLastRhs = 0;
  m->setScriptActive(true);
  mregs._internalFlags |= 1 << Moic::BitIsScriptActive;
  return Moic::OK;
}
void exitScript(IMotor* m, volatile Moic::MotorBlock& mregs, const uint8_t code) {
  mregs.sp = 0;
  mregs.scriptRepCode = code;
  m->setScriptActive(false);
  mregs._internalFlags &= ~(1 << Moic::BitIsScriptActive);
  mregs._waitEndTime = 0;
  m->setDelayActive(false);
  mregs.stateFlags = m->stateFlags();
}
bool isMotorBusy(IMotor* m, volatile Moic::MotorBlock& mregs) {
  return mregs._waitEndTime != 0 && millis() < mregs._waitEndTime || m->busy();
}
bool isScriptActive(IMotor* m, volatile Moic::MotorBlock& mregs) {
  return (mregs._internalFlags >> Moic::BitIsScriptActive) & 1;
}
bool isPageInStack(IMotor* m, volatile Moic::MotorBlock& mregs, const uint8_t page) {
  if (!MotorState::isScriptActive(m, mregs)) {
    return false;
  }
  if (page == mregs.scriptPage) {
    return true;
  }
  for (uint8_t i = 0; i < mregs.sp; ++i) {
    if (page == mregs.scriptStackPage[i]) {
      return true;
    }
  }
  return false;
}
}