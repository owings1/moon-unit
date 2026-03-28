#include <cstddef>
#include <sys/_stdint.h>
#include <stdint.h>
#include "I2CMotors.h"
#include "IMotor.h"

I2CMotors::I2CMotors(TwoWire& wire, IMotor** motors, uint8_t count)
  : wire(wire), motors(motors), numMotors(min(MAX_MOTORS, count)) {
  memset((void*) mem.buffer, 0, sizeof(mem.buffer));
  setBootId((uint16_t) millis());
  for (uint8_t mIdx = 0; mIdx < numMotors; ++mIdx) {
    syncMotorSettings(mIdx);
    syncMotorState(mIdx);
  }
}

void I2CMotors::setBootId(uint16_t id) {
  mem.regs.bootId = id;
}

void I2CMotors::update() {
  for (uint8_t i = 0; i < numMotors; ++i) processScript(i);
  memSyncInterval();
}

void I2CMotors::handleRead() {
  if (ptr < MOTOR_BASE_ADDR) {
    wire.write(mem.buffer[ptr]);
  } else {
    const uint8_t mIdx = getMidx(currentPage, ptr);
    if (mIdx >= numMotors) {
      wire.write(UNSET);
    } else if (currentPage < SCRIPT_PAGE_START) {
      if (ptr < TOTAL_BLOCK_SIZE) {
        const uint8_t offset = getStructOffset(ptr);
        uint8_t* motorData = (uint8_t*) &mem.regs.motors[mIdx];
        wire.write(motorData[offset]);
      } else {
        wire.write(UNSET);
      }
    } else if (currentPage < SCRIPT_PAGE_START * (NUM_SCRIPT_PAGES + 1)) {
      const uint8_t sIdx = (currentPage / SCRIPT_PAGE_START) - 1;
      wire.write(mem.regs.motors[mIdx].scripts[sIdx][ptr - MOTOR_BASE_ADDR]);
    } else {
      wire.write(UNSET);
    }
  }
  ptr++;
}

void I2CMotors::handleWrite(int howMany) {
  if (howMany < 1) {
    return;
  }
  masterWriting = true;
  ptr = wire.read();
  howMany--;
  while (howMany > 0 && ptr < 0x100) {
    uint8_t incoming = wire.read();
    if (ptr == PAGE_REGISTER) {
      currentPage = incoming;
      mem.regs.repCode = OK;
    } else if (isWriteable(currentPage, ptr)) {
      if (ptr < MOTOR_BASE_ADDR) {
        mem.buffer[ptr] = incoming;
        mem.regs.repCode = OK;
      } else {
        uint8_t mIdx = getMidx(currentPage, ptr);
        if (mIdx >= numMotors) {
          mem.regs.repCode = INVALID_MOTOR;
        } else if (currentPage < SCRIPT_PAGE_START) {
          uint8_t offset = getStructOffset(ptr);
          mem.regs.repCode = handleMotorWrite(mIdx, offset, incoming, true, true);
        } else if (currentPage < SCRIPT_PAGE_START * (NUM_SCRIPT_PAGES + 1)) {
          uint8_t sIdx = (currentPage / SCRIPT_PAGE_START) - 1;
          auto& mregs = mem.regs.motors[mIdx];
          if (isScriptActive(mIdx) && mregs.scriptPage == sIdx) {
            // Prevent writing to running script
            mem.regs.repCode = MOTOR_BUSY;
          } else {
            mregs.scripts[sIdx][ptr - MOTOR_BASE_ADDR] = incoming;
            mem.regs.repCode = OK;
          }
        } else {
          mem.regs.repCode = UNKNOWN_COMMAND;
        }
      }
    } else {
      mem.regs.repCode = READONLY_ATTRIBUTE;
    }
    ptr++;
    howMany--;
  }
  masterWriting = false;
}

uint8_t I2CMotors::handleMotorWrite(const uint8_t mIdx, const uint8_t offset, const uint8_t incoming, const bool enforceBusy, const bool enforceScriptLock) {
  if (mIdx >= numMotors) {
    return INVALID_MOTOR;
  }
  if (offset >= MOTOR_BLOCK_SIZE) {
    return UNKNOWN_COMMAND;
  }
  auto& m = motors[mIdx];
  auto& mregs = mem.regs.motors[mIdx];
  if (
    enforceBusy && ((MOTOR_BUSY_MASK >> offset) & 1) && isMotorBusy(mIdx) ||
    enforceScriptLock && ((SCRIPT_LOCK_MASK >> offset) & 1) && isScriptActive(mIdx)
  ) {
    // In case of prior partial write
    syncMotorSettings(mIdx);
    syncMotorState(mIdx);
    return MOTOR_BUSY;
  }
  // Get a pointer to the start of this specific motor's data in the buffer
  // This translates struct-relative 'offset' to the correct absolute buffer index
  uint8_t* motorData = (uint8_t*) &mregs;
  const uint8_t previous = motorData[offset];
  motorData[offset] = incoming;
  uint8_t repCode = OK;
  switch (offset) {
    case offsetof(MotorBlock, cmdMove) + 3:
      repCode = m->move(mregs.cmdMove) ? OK : COMMAND_IGNORED;
      mregs.cmdMove = 0;
      syncMotorState(mIdx);
      break;

    case offsetof(MotorBlock, cmdMoveTo) + 3:
      repCode = m->move(mregs.cmdMoveTo - m->currentPosition()) ? OK : COMMAND_IGNORED;
      mregs.cmdMoveTo = 0;
      syncMotorState(mIdx);
      break;

    case offsetof(MotorBlock, cmdStop):
      repCode = m->stop() ? OK : COMMAND_IGNORED;
      if (isScriptActive(mIdx)) {
        exitScript(mIdx, CANCELED);
        repCode = OK;
      }
      mregs.stateFlags = m->stateFlags();
      mregs.cmdStop = 0;
      break;

    case offsetof(MotorBlock, currentPosition) + 3:
      m->setCurrentPosition(mregs.currentPosition);
      mregs.currentPosition = m->currentPosition();
      break;

    case offsetof(MotorBlock, settingsFlags):
      m->setSettingsFlags(mregs.settingsFlags);
      mregs.settingsFlags = m->settingsFlags();
      break;

    case offsetof(MotorBlock, maxSpeed) + 3:
      m->setMaxSpeed(mregs.maxSpeed);
      mregs.maxSpeed = m->maxSpeed();
      break;

    case offsetof(MotorBlock, acceleration) + 3:
      m->setAcceleration(mregs.acceleration);
      mregs.acceleration = m->acceleration();
      break;

    case offsetof(MotorBlock, sleepTimeoutMs) + 1:
      m->setSleepTimeoutMs(mregs.sleepTimeoutMs);
      mregs.sleepTimeoutMs = m->sleepTimeoutMs();
      break;

    case offsetof(MotorBlock, enableDelayMs):
      m->setEnableDelayMs(mregs.enableDelayMs);
      mregs.enableDelayMs = m->enableDelayMs();
      break;

    case offsetof(MotorBlock, cmdDelay) + 3:
      if (mregs.cmdDelay > 0) {
        mregs._waitEndTime = millis() + mregs.cmdDelay;
        m->setDelayActive(true);
      } else {
        mregs._waitEndTime = 0;
        m->setDelayActive(false);
      }
      mregs.cmdDelay = 0;
      break;

    case offsetof(MotorBlock, cmdScriptExec):
      if (incoming < NUM_SCRIPT_PAGES) {
        mregs.scriptPage = incoming;
        mregs.scriptIdx = 0;
        mregs.scriptRepCode = OK;
        m->setScriptActive(true);
        mregs._internalFlags |= 1 << BitIsScriptActive;
        syncMotorState(mIdx);
      } else {
        repCode = OVERFLOW;
      }
      mregs.cmdScriptExec = 0;
      break;

    case offsetof(MotorBlock, cmdScriptClear):
      if (incoming < NUM_SCRIPT_PAGES) {
        if (isPageInStack(mIdx, incoming)) {
          // Prevent clearing running script
          repCode = MOTOR_BUSY;
        } else {
          memset((void*) mregs.scripts[incoming], 0, SCRIPT_PAGE_SIZE);
        }
      } else {
        repCode = OVERFLOW;
      }
      mregs.cmdScriptClear = 0;
      break;

    case offsetof(MotorBlock, cmdCall) + 1:
      if (!isScriptActive(mIdx)) {
        repCode = UNKNOWN_COMMAND;
      } else {
        repCode = scriptStackPush(mIdx, mregs.cmdCall[0], incoming);
      }
      memset((void*) mregs.cmdCall, 0, 2);
      break;

    case offsetof(MotorBlock, cmdJump) + 1:
      if (!isScriptActive(mIdx)) {
        repCode = UNKNOWN_COMMAND;
      } else {
        uint8_t page = mregs.cmdJump[0];
        if (page == UNSET) {
          page = mregs.scriptPage;
        }
        if (page < NUM_SCRIPT_PAGES && incoming < SCRIPT_PAGE_SIZE) {
          mregs.scriptPage = page;
          mregs.scriptIdx = incoming;
        } else {
          repCode = OVERFLOW;
        }
      }
      memset((void*) mregs.cmdJump, 0, 2);
      break;

    case offsetof(MotorBlock, cmdCondCall) + 3:
      if (!isScriptActive(mIdx)) {
        repCode = UNKNOWN_COMMAND;
      } else {
        const uint8_t func = mregs.cmdCondCall[0];
        const uint8_t rhs = mregs.cmdCondCall[1];
        const int8_t result = scriptCondition(mIdx, func, rhs);
        if (result < 0) {
          repCode = UNKNOWN_COMMAND;
        } else if (result > 0) {
          repCode = scriptStackPush(mIdx, mregs.cmdCondCall[2], incoming);
        }
      }
      memset((void*) mregs.cmdCondCall, 0, 4);
      break;

    case offsetof(MotorBlock, cmdCondJump) + 3:
      if (!isScriptActive(mIdx)) {
        repCode = UNKNOWN_COMMAND;
      } else {
        uint8_t page = mregs.cmdCondJump[2];
        if (page == UNSET) {
          page = mregs.scriptPage;
        }
        if (page < NUM_SCRIPT_PAGES && incoming < SCRIPT_PAGE_SIZE) {
          const uint8_t func = mregs.cmdCondJump[0];
          const uint8_t rhs = mregs.cmdCondJump[1];
          const int8_t result = scriptCondition(mIdx, func, rhs);
          if (result < 0) {
            repCode = UNKNOWN_COMMAND;
          } else if (result > 0) {
            mregs.scriptPage = page;
            mregs.scriptIdx = incoming;
          }
        } else {
          repCode = OVERFLOW;
        }
      }
      memset((void*) mregs.cmdCondJump, 0, 4);
      break;
  }
  return repCode;
}

uint8_t I2CMotors::scriptStackPush(const uint8_t mIdx, uint8_t page, const uint8_t sIdx) {
  if (mIdx >= numMotors) {
    return INVALID_MOTOR;
  }
  if (!isScriptActive(mIdx)) {
    return OTHER_ERROR;
  }
  auto& mregs = mem.regs.motors[mIdx];
  if (page == UNSET) {
    page = mregs.scriptPage;
  }
  if (page >= NUM_SCRIPT_PAGES || sIdx >= SCRIPT_PAGE_SIZE || mregs.sp >= SCRIPT_STACK_SIZE) {
    return OVERFLOW;
  }
  mregs.scriptStackIdx[mregs.sp] = mregs.scriptIdx;
  mregs.scriptStackPage[mregs.sp] = mregs.scriptPage;
  mregs.scriptStackRhsArg[mregs.sp] = mregs.scriptLastRhs;
  mregs.sp++;
  mregs.scriptPage = page;
  mregs.scriptIdx = sIdx;
  return OK;
}

bool I2CMotors::isPageInStack(const uint8_t mIdx, const uint8_t page) {
  if (!isScriptActive(mIdx)) {
    return false;
  }
  auto& mregs = mem.regs.motors[mIdx];
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

void I2CMotors::processScript(const uint8_t mIdx) {
  if (!isScriptActive(mIdx) || isMotorBusy(mIdx)) {
    return;
  }
  auto& m = motors[mIdx];
  auto& mregs = mem.regs.motors[mIdx];
  if (mregs.scriptPage >= NUM_SCRIPT_PAGES) {
    exitScript(mIdx, OVERFLOW);
    return;
  }
  if (mregs.scriptIdx >= SCRIPT_PAGE_SIZE) {
    exitScript(mIdx, OVERFLOW);
    return;
  }
  // Get OpCode (Struct Offset)
  volatile uint8_t* scriptBase = mregs.scripts[mregs.scriptPage];
  const uint8_t op = scriptBase[mregs.scriptIdx];
  if (op == OK || op >= USR1) {  // End markers
    // OK 0x00 or UNSET 0xFF are treated as OK 0x00.
    // Anything USR1 0xFA to USR5 0xFE are passed on as is to
    // the scriptRepCode, which can be evaluated in conditional
    // functions with EQL_RETURNCODE_RHS 0x03. A subroutine can
    // use this as a return code to the caller.
    const uint8_t code = (op == OK || op == UNSET) ? OK : op;
    if (mregs.sp > 0) {
      mregs.sp--;
      mregs.scriptRepCode = code;
      mregs.scriptPage = mregs.scriptStackPage[mregs.sp];
      mregs.scriptIdx = mregs.scriptStackIdx[mregs.sp];
      mregs.scriptLastRhs = mregs.scriptStackRhsArg[mregs.sp];
      return;
    }
    exitScript(mIdx, code);
    return;
  }
  const uint8_t dataLen = getOpCodeDataLength(op);
  if (dataLen == 0) {
    exitScript(mIdx, UNKNOWN_COMMAND);
    return;
  }
  const uint8_t totalCmdLen = 1 + dataLen;
  if (mregs.scriptIdx + totalCmdLen >= SCRIPT_PAGE_SIZE) {
    exitScript(mIdx, OTHER_ERROR);
    return;
  }
  // Consume from buffer (Advance BEFORE execution)
  const uint8_t currentCmdStart = mregs.scriptIdx;
  mregs.scriptIdx += totalCmdLen;
  // We feed bytes one-by-one to handleMotorWrite to reuse all logic
  for (uint8_t i = 0; i < dataLen; i++) {
    uint8_t offset = op + i;
    uint8_t incoming = scriptBase[currentCmdStart + i + 1];
    uint8_t code = handleMotorWrite(mIdx, offset, incoming, true, false);
    if (code != OK) {
      exitScript(mIdx, code);
      return;
    }
    if (!isScriptActive(mIdx)) {
      return;
    }
  }
}

void I2CMotors::exitScript(const uint8_t mIdx, const uint8_t code) {
  if (mIdx >= numMotors) {
    return;
  }
  auto& m = motors[mIdx];
  auto& mregs = mem.regs.motors[mIdx];
  mregs.sp = 0;
  mregs.scriptRepCode = code;
  m->setScriptActive(false);
  mregs._internalFlags &= ~(1 << BitIsScriptActive);
  mregs._waitEndTime = 0;
  m->setDelayActive(false);
  mregs.stateFlags = m->stateFlags();
}

int8_t I2CMotors::scriptCondition(const uint8_t mIdx, const uint8_t func, const uint8_t rhs) {
  if (mIdx >= numMotors) {
    return -1;
  }
  // Bit 7 of the function code signifies negation.
  const bool negate = func & 0x80;
  auto& m = motors[mIdx];
  auto& mregs = mem.regs.motors[mIdx];
  bool result = false;
  const uint8_t funId = func & 0x7F;
  switch (funId) {
    case ALWAYS_TRUE:
      result = true;
      break;
    case AND_STATEFLAGS_RHS:
      result = m->stateFlags() & rhs;
      break;
    case AND_SETTINGSFLAGS_RHS:
      result = m->settingsFlags() & rhs;
      break;
    case EQL_RETURNCODE_RHS:
      result = mregs.scriptRepCode == rhs;
      break;
    // The RHS value of the previous condition check can be evaluated
    // with EQL_LASTCONDARG_RHS 0x30 or AND_LASTCONDARG_RHS 0x31. This
    // can be used with ALWAYS_TRUE to pass an argument to a subroutine.
    case EQL_LASTCONDARG_RHS:
      result = mregs.scriptLastRhs == rhs;
      break;
    case AND_LASTCONDARG_RHS:
      result = mregs.scriptLastRhs & rhs;
      break;
    default:
      return -1;
  }
  if (funId < EQL_LASTCONDARG_RHS || funId > AND_LASTCONDARG_RHS) {
    // Only update the register if the operation wasn't a 'Read' of the register
    mregs.scriptLastRhs = rhs;
  }
  return result ^ negate;
}

bool I2CMotors::isMotorBusy(const uint8_t mIdx) {
  if (mIdx >= numMotors) {
    return false;
  }
  auto& m = motors[mIdx];
  auto& mregs = mem.regs.motors[mIdx];
  return mregs._waitEndTime != 0 && millis() < mregs._waitEndTime || m->busy();
}

bool I2CMotors::isScriptActive(const uint8_t mIdx) {
  if (mIdx >= numMotors) {
    return false;
  }
  auto& m = motors[mIdx];
  auto& mregs = mem.regs.motors[mIdx];
  return (mregs._internalFlags >> BitIsScriptActive) & 1;
}

void I2CMotors::memSyncInterval() {
  if (masterWriting) {
    return;
  }
  const uint32_t now = millis();
  // 1. Fast Sync (High-priority telemetry)
  if (now - lastFastSync >= FAST_SYNC_MS) {
    lastFastSync = now;
    for (uint8_t i = 0; i < numMotors; ++i) syncMotorState(i);
  }
  // 2. Slow Sync (Settings/Config) - Every 500ms
  if (now - lastSlowSync >= SLOW_SYNC_MS) {
    lastSlowSync = now;
    for (uint8_t i = 0; i < numMotors; ++i) syncMotorSettings(i);
  }
}

void I2CMotors::syncMotorState(const uint8_t mIdx) {
  if (mIdx >= numMotors) {
    return;
  }
  auto& mregs = mem.regs.motors[mIdx];
  auto& m = motors[mIdx];
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

void I2CMotors::syncMotorSettings(const uint8_t mIdx) {
  if (mIdx >= numMotors) {
    return;
  }
  auto& mregs = mem.regs.motors[mIdx];
  auto& m = motors[mIdx];
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

uint8_t I2CMotors::getMidx(const uint8_t page, const uint8_t ptr) {
  if (page < SCRIPT_PAGE_START) {
    return (page * 2) + (ptr >= LOWER_BLOCK_SIZE);
  }
  return page % SCRIPT_PAGE_START;
}

uint8_t I2CMotors::getStructOffset(const uint8_t ptr) {
  // Motor 2 starts at exactly 0x80
  // Motor 1 starts at exactly 0x08
  return (ptr >= LOWER_BLOCK_SIZE) ? (ptr - LOWER_BLOCK_SIZE) : (ptr - MOTOR_BASE_ADDR);
}

uint8_t I2CMotors::getOpCodeDataLength(const uint8_t offset) {
  switch (offset) {
    case offsetof(MotorBlock, currentPosition):
    case offsetof(MotorBlock, maxSpeed):
    case offsetof(MotorBlock, acceleration):
    case offsetof(MotorBlock, cmdMove):
    case offsetof(MotorBlock, cmdMoveTo):
    case offsetof(MotorBlock, cmdDelay):
    case offsetof(MotorBlock, cmdCondCall):
    case offsetof(MotorBlock, cmdCondJump):
      return 4;
    case offsetof(MotorBlock, sleepTimeoutMs):
    case offsetof(MotorBlock, cmdCall):
    case offsetof(MotorBlock, cmdJump):
      return 2;
    case offsetof(MotorBlock, settingsFlags):
    case offsetof(MotorBlock, enableDelayMs):
    case offsetof(MotorBlock, cmdStop):
      return 1;
    default:
      return 0;
  }
}

bool I2CMotors::isWriteable(const uint8_t page, const uint8_t ptr) {
  // Only the absolute first 8 bytes of the buffer use the DEVICE_WRITE_MASK
  if (ptr < MOTOR_BASE_ADDR) {
    return (DEVICE_WRITE_MASK >> ptr) & 1;
  }
  if (page >= SCRIPT_PAGE_START) {
    return (page % SCRIPT_PAGE_START) < MAX_MOTORS;
  }
  if (ptr >= TOTAL_BLOCK_SIZE) {
    return false;
  }
  const uint8_t offset = getStructOffset(ptr);
  return (MOTOR_WRITE_MASK >> offset) & 1;
}