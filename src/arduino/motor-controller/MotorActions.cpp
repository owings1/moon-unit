#include <sys/_stdint.h>

#include "MotorActions.h"

namespace MotorActions {
uint8_t onCurrentPosition(IMotor* m, volatile Moic::MotorBlock& mregs) {
  m->setCurrentPosition(mregs.currentPosition);
  mregs.currentPosition = m->currentPosition();
  return Moic::OK;
}
uint8_t onSettingsFlags(IMotor* m, volatile Moic::MotorBlock& mregs) {
  m->setSettingsFlags(mregs.settingsFlags);
  mregs.settingsFlags = m->settingsFlags();
  return Moic::OK;
}
uint8_t onEnableDelayMs(IMotor* m, volatile Moic::MotorBlock& mregs) {
  m->setEnableDelayMs(mregs.enableDelayMs);
  mregs.enableDelayMs = m->enableDelayMs();
  return Moic::OK;
}
uint8_t onSleepTimeout(IMotor* m, volatile Moic::MotorBlock& mregs) {
  m->setSleepTimeoutMs(mregs.sleepTimeoutMs);
  mregs.sleepTimeoutMs = m->sleepTimeoutMs();
  return Moic::OK;
}
uint8_t onMaxSpeed(IMotor* m, volatile Moic::MotorBlock& mregs) {
  m->setMaxSpeed(mregs.maxSpeed);
  mregs.maxSpeed = m->maxSpeed();  // Re-sync from hardware
  return Moic::OK;
}
uint8_t onAcceleration(IMotor* m, volatile Moic::MotorBlock& mregs) {
  m->setAcceleration(mregs.acceleration);
  mregs.acceleration = m->acceleration();
  return Moic::OK;
}
uint8_t onMove(IMotor* m, volatile Moic::MotorBlock& mregs) {
  const uint8_t code = m->move(mregs.cmdMove) ? Moic::OK : Moic::COMMAND_IGNORED;
  mregs.cmdMove = 0;
  return code;
}
uint8_t onMoveRev(IMotor* m, volatile Moic::MotorBlock& mregs) {
  const uint8_t code = m->move(-mregs.cmdMoveRev) ? Moic::OK : Moic::COMMAND_IGNORED;
  mregs.cmdMoveRev = 0;
  return code;
}
uint8_t onMoveTo(IMotor* m, volatile Moic::MotorBlock& mregs) {
  const uint8_t code = m->move(mregs.cmdMoveTo - m->currentPosition()) ? Moic::OK : Moic::COMMAND_IGNORED;
  mregs.cmdMoveTo = 0;
  return code;
}
uint8_t onDelay(IMotor* m, volatile Moic::MotorBlock& mregs) {
  if (mregs.cmdDelay > 0) {
    mregs._waitEndTime = millis() + mregs.cmdDelay;
    m->setDelayActive(true);
  } else {
    mregs._waitEndTime = 0;
    m->setDelayActive(false);
  }
  mregs.cmdDelay = 0;
  return Moic::OK;
}
uint8_t onStop(IMotor* m, volatile Moic::MotorBlock& mregs) {
  uint8_t code = m->stop() ? Moic::OK : Moic::COMMAND_IGNORED;
  if (MotorState::isScriptActive(m, mregs)) {
    MotorState::exitScript(m, mregs, Moic::CANCELED);
    code = Moic::OK;
  }
  mregs.stateFlags = m->stateFlags();
  mregs.cmdStop = 0;
  return code;
}
uint8_t onScriptClear(IMotor* m, volatile Moic::MotorBlock& mregs) {
  uint8_t code = Moic::OK;
  if (mregs.cmdScriptClear < Moic::NUM_SCRIPT_PAGES) {
    if (MotorState::isPageInStack(m, mregs, mregs.cmdScriptClear)) {
      // Prevent clearing running script
      code = Moic::MOTOR_BUSY;
    } else {
      memset((void*)mregs.scripts[mregs.cmdScriptClear], 0, Moic::SCRIPT_PAGE_SIZE);
    }
  } else {
    code = Moic::OVERFLOW;
  }
  mregs.cmdScriptClear = 0;
  return code;
}
uint8_t onScriptExec(IMotor* m, volatile Moic::MotorBlock& mregs) {
  const uint8_t code = MotorState::enterScript(m, mregs, mregs.cmdScriptExec, 0);
  mregs.cmdScriptExec = 0;
  return code;
}
uint8_t onCmdCall(IMotor* m, volatile Moic::MotorBlock& mregs) {
  uint8_t code = Moic::OK;
  if (!MotorState::isScriptActive(m, mregs)) {
    code = Moic::UNKNOWN_COMMAND;
  } else {
    code = MotorVM::call(mregs, mregs.cmdCall);
  }
  clearTrigger(mregs.cmdCall, 2);
  return code;
}
uint8_t onCmdJump(IMotor* m, volatile Moic::MotorBlock& mregs) {
  uint8_t code = Moic::OK;
  if (!MotorState::isScriptActive(m, mregs)) {
    code = Moic::UNKNOWN_COMMAND;
  } else {
    code = MotorVM::jump(mregs, mregs.cmdJump);
  }
  clearTrigger(mregs.cmdJump, 2);
  return code;
}
uint8_t onCondCall(IMotor* m, volatile Moic::MotorBlock& mregs) {
  uint8_t code = Moic::OK;
  if (!MotorState::isScriptActive(m, mregs)) {
    code = Moic::UNKNOWN_COMMAND;
  } else {
    code = MotorVM::condCall(mregs, mregs.cmdCondCall);
  }
  clearTrigger(mregs.cmdCondCall, 4);
  return code;
}
uint8_t onCondJump(IMotor* m, volatile Moic::MotorBlock& mregs) {
  uint8_t code = Moic::OK;
  if (!MotorState::isScriptActive(m, mregs)) {
    code = Moic::UNKNOWN_COMMAND;
  } else {
    code = MotorVM::condJump(mregs, mregs.cmdCondJump);
  }
  clearTrigger(mregs.cmdCondJump, 4);
  return code;
}

uint8_t write(IMotor* m, volatile Moic::MotorBlock& mregs, const uint8_t offset, const uint8_t incoming, const bool enforceBusy, const bool enforceScriptLock) {
  static bool initialized = []() {
    for (const auto& entry : ACTION_TABLE) {
      ACTION_LOOKUP[entry.offset + entry.size - 1] = &entry;
    }
    return true;
  }();
  if (offset >= Moic::MOTOR_BLOCK_SIZE) {
    return Moic::UNKNOWN_COMMAND;
  }
  if (
    enforceBusy && ((Moic::MOTOR_BUSY_MASK >> offset) & 1) && MotorState::isMotorBusy(m, mregs) ||
    enforceScriptLock && ((Moic::SCRIPT_LOCK_MASK >> offset) & 1) && MotorState::isScriptActive(m, mregs)
  ) {
    // In case of prior partial write
    MotorState::syncMotorSettings(m, mregs);
    MotorState::syncMotorState(m, mregs);
    return Moic::MOTOR_BUSY;
  }
  // Get a pointer to the start of this specific motor's data in the buffer
  // This translates struct-relative 'offset' to the correct absolute buffer index
  uint8_t* motorData = (uint8_t*)&mregs;
  motorData[offset] = incoming;
  uint8_t repCode = Moic::OK;
  if (offset < Moic::MOTOR_BLOCK_SIZE) {
    const RegMapping* entry = ACTION_LOOKUP[offset];
    if (entry) {
      repCode = entry->handler(m, mregs);
      MotorState::syncMotorState(m, mregs);
    }
  }
  return repCode;
}

void tickScript(IMotor* m, volatile Moic::MotorBlock& mregs) {
  if (!MotorState::isScriptActive(m, mregs) || MotorState::isMotorBusy(m, mregs)) {
    return;
  }
  uint8_t offset, count, code;
  if (MotorVM::processNext(mregs, offset, count, code)) {
    if (count >= Moic::SCRIPT_WRITEBUF_SIZE) {
      MotorState::exitScript(m, mregs, Moic::OTHER_ERROR);
      return;
    }
    for (uint8_t i = 0; i < count; ++i) {
      code = write(m, mregs, offset + i, mregs.scriptWriteBuf[i], true, false);
      if (code != Moic::OK) {
        MotorState::exitScript(m, mregs, code);
        return;
      }
      if (!MotorState::isScriptActive(m, mregs)) {
        return;
      }
    }
  } else {
    MotorState::exitScript(m, mregs, code);
  }
}

}

void clearTrigger(volatile uint8_t* buf, uint8_t size) {
  for (uint8_t i = 0; i < size; i++) buf[i] = 0;
}
