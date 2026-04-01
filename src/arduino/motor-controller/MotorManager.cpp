#include <sys/_stdint.h>
#include "MotorManager.h"
#include "MotorActions.h"

namespace Moic {
ManagedMotor::ManagedMotor(IMotor* m, volatile MotorInterface& mregs, volatile MotorContext& ctx)
  : m(m), mregs(&mregs), ctx(&ctx) {
  static bool initialized = []() {
    for (const auto& entry : MotorActions::ACTION_TABLE) {
      MotorActions::ACTION_LOOKUP[entry.offset + entry.size - 1] = &entry;
    }
    return true;
  }();
}

bool ManagedMotor::scriptActive() {
  return _scriptActive;
}

void ManagedMotor::tick() {
  if (!scriptActive()) {
    return;
  }
  uint8_t offset, count, code;
  if (MotorVM::processNext(*(mregs), *(ctx), offset, count, code)) {
    if (count > SCRIPT_WRITEBUF_SIZE) {
      exitScript(OTHER_ERROR);
      _scriptActive = false;
      return;
    }
    for (uint8_t i = 0; i < count; ++i) {
      code = write(offset + i, ctx->scriptWriteBuf[i], VMEXC);
      if (code != OK) {
        exitScript(code);
        _scriptActive = false;
        return;
      }
      _scriptActive = (ctx->_internalFlags >> BitIsScriptActive) & 1;
      if (!_scriptActive) {
        return;
      }
    }
    _scriptActive = true;
  } else {
    exitScript(code);
    _scriptActive = false;
  }
}

uint8_t ManagedMotor::write(const uint8_t offset, const uint8_t incoming, const uint8_t source) {
  if (offset >= MOTOR_BLOCK_SIZE) {
    return UNKNOWN_COMMAND;
  }
  const uint8_t perms = OFFSET_WRITEMASKS[offset];
  if (!(perms & (1 << source))) {
    return READONLY_ATTRIBUTE;
  }
  if (!(perms & BUSY_EXEMPT_MASK) && busy()) {
    // In case of prior partial write
    syncMotorSettings();
    syncMotorState();
    return MOTOR_BUSY;
  }
  ((uint8_t*)mregs)[offset] = incoming;
  const MotorActions::RegMapping* entry = MotorActions::ACTION_LOOKUP[offset];
  if (entry) {
    const uint8_t code = entry->handler(*(this));
    syncMotorState();
    return code;
  }
  return OK;
}

uint8_t ManagedMotor::enterScript(uint8_t page, const uint8_t arg) {
  if (busy() || scriptActive()) {
    return MOTOR_BUSY;
  }
  if (page >= NUM_SCRIPT_PAGES) {
    return OVERFLOW;
  }
  mregs->scriptPage = page;
  mregs->scriptIdx = 0;
  mregs->scriptRepCode = OK;
  ctx->scriptCallArg = arg;
  ctx->scriptLastRhs = 0;
  m->setScriptActive(true);
  ctx->_internalFlags |= 1 << BitIsScriptActive;
  _scriptActive = true;
  return OK;
}

void ManagedMotor::exitScript(const uint8_t code) {
  ctx->sp = 0;
  mregs->scriptRepCode = code;
  m->setScriptActive(false);
  ctx->_internalFlags &= ~(1 << BitIsScriptActive);
  _scriptActive = false;
  mregs->waitEndTime = 0;
  m->setDelayActive(false);
  mregs->stateFlags = m->stateFlags();
}

bool ManagedMotor::isPageInStack(const uint8_t page) {
  _scriptActive = (ctx->_internalFlags >> BitIsScriptActive) & 1;
  if (!_scriptActive) {
    return false;
  }
  if (page == mregs->scriptPage) {
    return true;
  }
  for (uint8_t i = 0; i < ctx->sp; ++i) {
    if (page == ctx->scriptStackPage[i]) {
      return true;
    }
  }
  return false;
}

bool ManagedMotor::busy() {
  return mregs->waitEndTime != 0 && millis() < mregs->waitEndTime || m->busy();
}

void ManagedMotor::syncMotorSettings() {
  if (mregs->settingsFlags != m->settingsFlags()) {
    mregs->settingsFlags = m->settingsFlags();
  }
  if (mregs->maxSpeed != m->maxSpeed()) {
    mregs->maxSpeed = m->maxSpeed();
  }
  if (mregs->acceleration != m->acceleration()) {
    mregs->acceleration = m->acceleration();
  }
  if (mregs->sleepTimeoutMs != m->sleepTimeoutMs()) {
    mregs->sleepTimeoutMs = m->sleepTimeoutMs();
  }
  if (mregs->enableDelayMs != m->enableDelayMs()) {
    mregs->enableDelayMs = m->enableDelayMs();
  }
}
void ManagedMotor::syncMotorState() {
  mregs->stateFlags = m->stateFlags();
  mregs->currentPosition = m->currentPosition();
  mregs->targetPosition = m->targetPosition();
  mregs->speed = m->speed();
  if (mregs->waitEndTime != 0) {
    if (millis() >= mregs->waitEndTime) {
      mregs->waitEndTime = 0;
      m->setDelayActive(false);
    }
  }
}

};