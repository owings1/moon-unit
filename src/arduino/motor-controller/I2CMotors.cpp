#include <cstddef>
#include <sys/_stdint.h>
#include <stdint.h>
#include "I2CMotors.h"
#include "IMotor.h"

I2CMotors::I2CMotors(TwoWire& wire, IMotor** motors, uint8_t count)
  : wire(wire), motors(motors), numMotors(min(I2C_MAXMOTORS, count)) {
  memset((void*) mem.buffer, 0, sizeof(mem.buffer));
  setBootId((uint16_t) millis());
  syncAll();
}

void I2CMotors::setBootId(uint16_t id) {
  mem.regs.bootId = id;
}

void I2CMotors::update() {
  processScript();
  memSyncInterval();
}

void I2CMotors::handleRead() {
  if (ptr < MOTOR_BASE_ADDR) {
    wire.write(mem.buffer[ptr]);
  } else {
    const uint8_t mIdx = getMidx(currentPage, ptr);
    if (mIdx >= numMotors) {
      wire.write(0xFF);
    } else if (currentPage < 0x10) {
      if (ptr < TOTAL_BLOCK_SIZE) {
        const uint8_t offset = getStructOffset(ptr);
        uint8_t* motorData = (uint8_t*) &mem.regs.motors[mIdx];
        wire.write(motorData[offset]);
      } else {
        wire.write(0xFF);
      }
    } else if (currentPage < 0x20) {
      wire.write(mem.regs.motors[mIdx].script[ptr - MOTOR_BASE_ADDR]);
    } else {
      wire.write(0xFF);
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
        } else if (currentPage < 0x10) {
          uint8_t offset = getStructOffset(ptr);
          mem.regs.repCode = handleMotorWrite(mIdx, offset, incoming, true, true);
        } else if (currentPage < 0x20) {
          mem.regs.motors[mIdx].script[ptr - MOTOR_BASE_ADDR] = incoming;
          mem.regs.repCode = OK;
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
    enforceScriptLock && ((SCRIPT_LOCK_MASK >> offset) & 1) && ((mregs._internalFlags >> BitIsScriptActive) & 1)
  ) {
    // In case of prior partial write
    syncAll(mIdx);
    return MOTOR_BUSY;
  }
  // Get a pointer to the start of this specific motor's data in the buffer
  // This translates struct-relative 'offset' to the correct absolute buffer index
  uint8_t* motorData = (uint8_t*) &mregs;
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
      if ((mregs._internalFlags >> BitIsScriptActive) & 1) {
        exitScript(mIdx, CANCELED);
        repCode = OK;
      }
      mregs.stateFlags = m->stateFlags();
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

    case offsetof(MotorBlock, cmdScriptExec):
      mregs.scriptIdx = 0;
      mregs.scriptRepCode = OK;
      m->setScriptActive(true);
      mregs._internalFlags |= 1 << BitIsScriptActive;
      syncMotorState(mIdx);
      break;

    case offsetof(MotorBlock, cmdScriptClear):
      memset((void*)mregs.script, 0, sizeof(mregs.script));
      mregs.scriptIdx = 0;
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
  }
  return repCode;
}

void I2CMotors::processScript() {
  for (uint8_t i = 0; i < numMotors; ++i) {
    processScript(i);
  }
}

void I2CMotors::processScript(const uint8_t mIdx) {
  if (mIdx >= numMotors) {
    return;
  }
  auto& m = motors[mIdx];
  auto& mregs = mem.regs.motors[mIdx];
  if (!((mregs._internalFlags >> BitIsScriptActive) & 1) || isMotorBusy(mIdx)) {
    return;
  }
  if (mregs.scriptIdx >= sizeof(mregs.script)) {
    exitScript(mIdx, OTHER_ERROR);
    return;
  }
  // 1. Get OpCode (Struct Offset)
  volatile uint8_t* scriptBase = mregs.script;
  const uint8_t op = scriptBase[mregs.scriptIdx];
  if (SCRIPT_CLEAR_ON_READ) {
    scriptBase[mregs.scriptIdx] = 0;
  }
  if (op == 0x00 || op == 0xFF) {  // End markers
    exitScript(mIdx, OK);
    return;
  }
  const uint8_t dataLen = getOpCodeDataLength(op);
  if (dataLen == 0) {
    exitScript(mIdx, UNKNOWN_COMMAND);
    return;
  }
  const uint8_t totalCmdLen = 1 + dataLen;
  if (mregs.scriptIdx + totalCmdLen >= sizeof(mregs.script)) {
    exitScript(mIdx, OTHER_ERROR);
    return;
  }
  // 2. Consume from buffer (Advance BEFORE execution)
  const uint8_t currentCmdStart = mregs.scriptIdx;
  mregs.scriptIdx += totalCmdLen;
  // 3. Extract data and execute
  // We feed bytes one-by-one to handleMotorWrite to reuse all logic
  for (uint8_t i = 0; i < dataLen; i++) {
    uint8_t offset = op + i;  // The target register offset
    uint8_t incoming = scriptBase[currentCmdStart + i + 1];
    if (SCRIPT_CLEAR_ON_READ) {
      scriptBase[currentCmdStart + i + 1] = 0;
    }
    uint8_t code = handleMotorWrite(mIdx, offset, incoming, true, false);
    if (code != OK) {
      exitScript(mIdx, code);
      return;
    }
    if (!((mregs._internalFlags >> BitIsScriptActive) & 1)) {
      return;
    }
    mregs.scriptRepCode = code;
  }
}

void I2CMotors::exitScript(const uint8_t mIdx, const uint8_t code) {
  if (mIdx >= numMotors) {
    return;
  }
  auto& m = motors[mIdx];
  auto& mregs = mem.regs.motors[mIdx];
  mregs.scriptRepCode = code;
  mregs.scriptIdx = 0;
  m->setScriptActive(false);
  mregs._internalFlags &= ~(1 << BitIsScriptActive);
  mregs._waitEndTime = 0;
  m->setDelayActive(false);
  mregs.stateFlags = m->stateFlags();
}

bool I2CMotors::isMotorBusy(const uint8_t mIdx) {
  if (mIdx >= numMotors) {
    return false;
  }
  auto& m = motors[mIdx];
  auto& mregs = mem.regs.motors[mIdx];
  return mregs._waitEndTime != 0 && millis() < mregs._waitEndTime || m->busy();
}

void I2CMotors::memSyncInterval() {
  if (masterWriting) {
    return;
  }
  const uint32_t now = millis();
  // 1. Fast Sync (High-priority telemetry)
  if (now - lastFastSync >= FAST_SYNC_MS) {
    lastFastSync = now;
    syncMotorState();
  }
  // 2. Slow Sync (Settings/Config) - Every 500ms
  if (now - lastSlowSync >= SLOW_SYNC_MS) {
    lastSlowSync = now;
    syncMotorSettings();
  }
}

void I2CMotors::syncAll() {
  syncMotorSettings();
  syncMotorState();
}
void I2CMotors::syncAll(const uint8_t mIdx) {
  syncMotorSettings(mIdx);
  syncMotorState(mIdx);
}

void I2CMotors::syncMotorState() {
  for (uint8_t i = 0; i < numMotors; ++i) {
    syncMotorState(i);
  }
}

void I2CMotors::syncMotorSettings() {
  for (uint8_t i = 0; i < numMotors; ++i) {
    syncMotorSettings(i);
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
  if (page < 0x10) {
    return (page * 2) + (ptr >= 0x80);
  }
  return page % 0x10;
}

uint8_t I2CMotors::getStructOffset(const uint8_t ptr) {
  // Motor 2 starts at exactly 0x80
  // Motor 1 starts at exactly 0x08
  return (ptr >= 0x80) ? (ptr - 0x80) : (ptr - MOTOR_BASE_ADDR);
}

uint8_t I2CMotors::getOpCodeDataLength(const uint8_t offset) {
  switch (offset) {
    case offsetof(MotorBlock, cmdMove):
    case offsetof(MotorBlock, cmdMoveTo):
    case offsetof(MotorBlock, maxSpeed):
    case offsetof(MotorBlock, acceleration):
    case offsetof(MotorBlock, currentPosition):
    case offsetof(MotorBlock, cmdDelay):
      return 4;
    case offsetof(MotorBlock, sleepTimeoutMs):
      return 2;
    case offsetof(MotorBlock, cmdScriptExec):
    case offsetof(MotorBlock, cmdScriptClear):
    case offsetof(MotorBlock, scriptIdx):
    case offsetof(MotorBlock, cmdStop):
    case offsetof(MotorBlock, settingsFlags):
    case offsetof(MotorBlock, enableDelayMs):
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
  if (page >= 0x10) {
    return (page % 0x10) < I2C_MAXMOTORS;
  }
  if (ptr >= TOTAL_BLOCK_SIZE) {
    return false;
  }
  const uint8_t offset = getStructOffset(ptr);
  return (MOTOR_WRITE_MASK >> offset) & 1;
}