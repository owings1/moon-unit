#include <sys/_stdint.h>
#include "MotorManager.h"
#include "MotorActions.h"

namespace Moic {
ManagedMotor::ManagedMotor(IMotor* m)
  : m(m), mregs(&_mregs), vmctx(&_vmctx) {
  static bool initialized = []() {
    for (const auto& entry : MotorActions::ACTION_TABLE) {
      MotorActions::ACTION_LOOKUP[entry.offset + entry.size - 1] = &entry;
    }
    return true;
  }();
  memset((void*)&_mregs, 0, sizeof(MotorInterface));
  memset((void*)&_vmctx, 0, sizeof(VMContext));
}

bool ManagedMotor::scriptActive() {
  return _scriptActive;
}

void ManagedMotor::tick() {
  if (!scriptActive() || busy()) {
    return;
  }
  if (!MotorVM::processNext(*(this))) {
    exitScript(vmctx->exitCode);
    return;
  }
  const uint8_t count = vmctx->count;
  if (count > SCRIPT_WRITEBUF_SIZE) {
    exitScript(OTHER_ERROR);
    return;
  }
  const uint8_t offset = vmctx->offset;
  for (uint8_t i = 0; i < count; ++i) {
    uint8_t code = write(offset + i, vmctx->writeBuf[i], VMEXC);
    if (code != OK) {
      exitScript(code);
      return;
    }
    if (!scriptActive()) {
      return;
    }
  }
  syncScriptState();
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
  _scriptActive = true;
  mregs->scriptPage = page;
  mregs->scriptIdx = 0;
  mregs->scriptRepCode = OK;
  vmctx->page = page;
  vmctx->idx = 0;
  vmctx->callArg = arg;
  vmctx->rhsArg = 0;
  vmctx->count = 0;
  vmctx->sp = 0;
  vmctx->exitCode = OK;
  m->setScriptActive(true);
  return OK;
}

void ManagedMotor::exitScript(const uint8_t code) {
  _scriptActive = false;
  vmctx->sp = 0;
  mregs->scriptPage = vmctx->page;
  mregs->scriptIdx = vmctx->idx;
  mregs->scriptRepCode = code;
  m->setScriptActive(false);
  mregs->waitEndTime = 0;
  m->setDelayActive(false);
  mregs->stateFlags = m->stateFlags();
}

bool ManagedMotor::isPageInStack(const uint8_t page) {
  if (!_scriptActive) {
    return false;
  }
  if (page == mregs->scriptPage || page == vmctx->page) {
    return true;
  }
  for (uint8_t i = 0; i < vmctx->sp; ++i) {
    if (page == vmctx->stack[i].page) {
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
void ManagedMotor::syncScriptState() {
  mregs->scriptPage = vmctx->page;
  mregs->scriptIdx = vmctx->idx;
  mregs->scriptRepCode = vmctx->exitCode;
}

};