#include <sys/_stdint.h>
#include "MotorManager.h"
#include "MotorActions.h"
#include "MotorVM.h"

void onMotorNotify(void* ctx) {
  static_cast<Moic::ManagedMotor*>(ctx)->setForceStateSyncAt(millis());
}
namespace Moic {
ManagedMotor::ManagedMotor(IMotor* m)
  : m(m), mregs(&_mregs), vmctx(&_vmctx) {
  static bool initialized = []() {
    for (const auto& entry : MotorActions::ACTION_TABLE) {
      uint8_t tableIdx = &entry - MotorActions::ACTION_TABLE;
      MotorActions::ACTION_LOOKUP[entry.offset + entry.size - 1] = &MotorActions::ACTION_TABLE[tableIdx];
    }
    MotorVM::initStaticTables();
    return true;
  }();
  memset((void*)&_mregs, 0, sizeof(MotorInterface));
  memset((void*)&_vmctx, 0, sizeof(VMContext));
  syncMotorState();
  syncMotorSettings();
  m->setNotify(onMotorNotify, this);
}

bool ManagedMotor::scriptActive() {
  return _scriptActive;
}
void ManagedMotor::syncLockInc() {
  syncLockout++;
}
void ManagedMotor::syncLockDec() {
  if (syncLockout) {
    syncLockout--;
  }
}
void ManagedMotor::setForceStateSyncAt(const uint32_t when) {
  forceStateSyncAt = when;
}
void ManagedMotor::tick() {
  memSyncInterval();
  if (!scriptActive() || _isBusyFast) {
    return;
  }
  if (!MotorVM::tick(*(this))) {
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
    return INVALID_COMMAND;
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
  vmctx->page = page;
  vmctx->idx = 0;
  vmctx->exitCode = OK;
  syncScriptState();
  vmctx->callArg = arg;
  vmctx->condArg = 0;
  vmctx->compArg = 0;
  vmctx->count = 0;
  vmctx->sp = 0;
  m->setScriptActive(true);
  return OK;
}

void ManagedMotor::exitScript(const uint8_t code) {
  _scriptActive = false;
  vmctx->exitCode = code;
  syncScriptState();
  vmctx->sp = 0;
  m->setScriptActive(false);
  mregs->waitEndTime = 0;
  m->setDelayActive(false);
  mregs->stateFlags = m->stateFlags();
}

bool ManagedMotor::isPageInStack(const uint8_t page) {
  if (!scriptActive()) {
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
void ManagedMotor::memSyncInterval() {
  if (syncLockout) {
    return;
  }
  const uint32_t now = millis();
  // 1. Fast Sync (High-priority telemetry)
  if (now - lastFastSync >= FAST_SYNC_MS || forceStateSyncAt && now >= forceStateSyncAt) {
    lastFastSync = now;
    if (now >= forceStateSyncAt) {
      forceStateSyncAt = 0;
    }
    syncMotorState();
  }
  // 2. Slow Sync (Settings/Config) - Every 500ms
  if (now - lastSlowSync >= SLOW_SYNC_MS) {
    lastSlowSync = now;
    syncMotorSettings();
  }
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
  _isBusyFast = busy();
}
void ManagedMotor::syncScriptState() {
  mregs->scriptPage = vmctx->page;
  mregs->scriptIdx = vmctx->idx;
  mregs->scriptRepCode = vmctx->exitCode;
}

};