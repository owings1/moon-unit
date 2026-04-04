#include <sys/_stdint.h>

#include "MotorActions.h"

namespace MotorActions {
uint8_t onCurrentPosition(Moic::ManagedMotor& mm) {
  mm.m->setCurrentPosition(mm.mregs->currentPosition);
  mm.mregs->currentPosition = mm.m->currentPosition();
  return Moic::OK;
}
uint8_t onSettingsFlags(Moic::ManagedMotor& mm) {
  mm.m->setSettingsFlags(mm.mregs->settingsFlags);
  mm.mregs->settingsFlags = mm.m->settingsFlags();
  return Moic::OK;
}
uint8_t onEnableDelayMs(Moic::ManagedMotor& mm) {
  mm.m->setEnableDelayMs(mm.mregs->enableDelayMs);
  mm.mregs->enableDelayMs = mm.m->enableDelayMs();
  return Moic::OK;
}
uint8_t onSleepTimeout(Moic::ManagedMotor& mm) {
  mm.m->setSleepTimeoutMs(mm.mregs->sleepTimeoutMs);
  mm.mregs->sleepTimeoutMs = mm.m->sleepTimeoutMs();
  return Moic::OK;
}
uint8_t onMaxSpeed(Moic::ManagedMotor& mm) {
  mm.m->setMaxSpeed(abs(mm.mregs->maxSpeed));
  mm.mregs->maxSpeed = mm.m->maxSpeed();
  return Moic::OK;
}
uint8_t onAcceleration(Moic::ManagedMotor& mm) {
  mm.m->setAcceleration(abs(mm.mregs->acceleration));
  mm.mregs->acceleration = mm.m->acceleration();
  return Moic::OK;
}
uint8_t onMove(Moic::ManagedMotor& mm) {
  const uint8_t code = mm.m->move(mm.mregs->cmdMove) ? Moic::OK : Moic::COMMAND_IGNORED;
  mm.mregs->cmdMove = 0;
  return code;
}
uint8_t onMoveRev(Moic::ManagedMotor& mm) {
  const uint8_t code = mm.m->move(-mm.mregs->cmdMoveRev) ? Moic::OK : Moic::COMMAND_IGNORED;
  mm.mregs->cmdMoveRev = 0;
  return code;
}
uint8_t onMoveTo(Moic::ManagedMotor& mm) {
  const uint8_t code = mm.m->move(mm.mregs->cmdMoveTo - mm.m->currentPosition()) ? Moic::OK : Moic::COMMAND_IGNORED;
  mm.mregs->cmdMoveTo = 0;
  return code;
}
uint8_t onDelay(Moic::ManagedMotor& mm) {
  const uint32_t delay = mm.mregs->cmdDelay;
  uint32_t forceAt;
  if (delay > 0) {
    const uint32_t endTime = millis() + delay;
    forceAt = endTime + 1;
    mm.mregs->waitEndTime = endTime;
    mm.m->setDelayActive(true);
  } else {
    forceAt = millis() + 1;
    mm.mregs->waitEndTime = 0;
    mm.m->setDelayActive(false);
  }
  if (delay <= Moic::ManagedMotor::FAST_SYNC_MS) {
    mm.setForceStateSyncAt(forceAt);
  }
  mm.mregs->cmdDelay = 0;
  return Moic::OK;
}
uint8_t onStop(Moic::ManagedMotor& mm) {
  uint8_t code = mm.m->stop() ? Moic::OK : Moic::COMMAND_IGNORED;
  if (mm.scriptActive()) {
    mm.exitScript(Moic::CANCELED);
    code = Moic::OK;
  }
  mm.mregs->stateFlags = mm.m->stateFlags();
  mm.mregs->cmdStop = 0;
  return code;
}
uint8_t onScriptClear(Moic::ManagedMotor& mm) {
  uint8_t code = Moic::OK;
  if (mm.mregs->cmdScriptClear < Moic::NUM_SCRIPT_PAGES) {
    if (mm.isPageInStack(mm.mregs->cmdScriptClear)) {
      // Prevent clearing running script
      code = Moic::MOTOR_BUSY;
    } else {
      memset((void*)mm.vmctx->scripts[mm.mregs->cmdScriptClear], 0, Moic::SCRIPT_PAGE_SIZE);
    }
  } else {
    code = Moic::OVERFLOW;
  }
  mm.mregs->cmdScriptClear = 0;
  return code;
}
uint8_t onScriptExec(Moic::ManagedMotor& mm) {
  const uint8_t code = mm.enterScript(mm.mregs->cmdScriptExec[0], mm.mregs->cmdScriptExec[1]);
  clearTrigger(mm.mregs->cmdScriptExec, 2);
  return code;
}

}

void clearTrigger(volatile uint8_t* buf, uint8_t size) {
  for (uint8_t i = 0; i < size; i++) buf[i] = 0;
}
