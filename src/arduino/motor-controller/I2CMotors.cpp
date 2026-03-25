#include <sys/_stdint.h>
#include <stdint.h>
#include "I2CMotors.h"
#include "IMotor.h"

I2CMotors::I2CMotors(TwoWire& wire, IMotor** motors, uint8_t count)
  : wire(wire), motors(motors), numMotors(count) {
  memset((void*)mem.buffer, 0, sizeof(mem.buffer));
  setBootId((uint16_t)millis());
  syncAll();
}

void I2CMotors::setBootId(uint16_t id) {
  mem.regs.bootId = id;
}

void I2CMotors::begin(uint8_t address, uint32_t freq) {
  wire.setClock(freq);
  wire.begin(address);
}

void I2CMotors::update() {
  memSyncInterval();
}

void I2CMotors::handleRead() {
  if (ptr < MOTOR_BASE_ADDR) {
    wire.write(mem.buffer[ptr]);
  } else if (ptr < TOTAL_BLOCK_SIZE) {
    const uint8_t mIdx = getMidx(currentPage, ptr);
    if (mIdx >= numMotors) {
      wire.write(0xFF);
    } else {
      const uint8_t structOffset = getStructOffset(ptr);
      uint8_t* motorData = (uint8_t*)&mem.regs.motors[mIdx];
      wire.write(motorData[structOffset]);
    }
  } else {
    wire.write(0xFF);
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
    } else if (isWriteable(ptr)) {
      if (ptr < MOTOR_BASE_ADDR) {
        mem.buffer[ptr] = incoming;
      } else {
        uint8_t mIdx = getMidx(currentPage, ptr);
        uint8_t structOffset = getStructOffset(ptr);
        mem.regs.repCode = handleMotorWrite(mIdx, structOffset, incoming);
      }
    } else {
      mem.regs.repCode = READONLY_ATTRIBUTE;
    }
    ptr++;
    howMany--;
  }
  masterWriting = false;
}

bool I2CMotors::isWriteable(const uint8_t ptr) {
  if (ptr >= TOTAL_BLOCK_SIZE) {
    return false;
  }
  // Only the absolute first 8 bytes of the buffer use the DEVICE_WRITE_MASK
  if (ptr < MOTOR_BASE_ADDR) {
    return (DEVICE_WRITE_MASK >> ptr) & 1;
  }
  // Identify structOffset based on which bank we are in
  const uint8_t structOffset = getStructOffset(ptr);
  // Apply the same logic to the calculated structOffset
  if (structOffset >= 0x24) {
    return true;  // Open expansion space
  }
  return (MOTOR_WRITE_MASK >> structOffset) & 1;
}

uint8_t I2CMotors::handleMotorWrite(const uint8_t motorIdx, const uint8_t offset, const uint8_t incoming) {
  if (motorIdx >= numMotors) {
    return INVALID_MOTOR;
  }
  if (offset >= MOTOR_BLOCK_SIZE) {
    return UNKNOWN_COMMAND;
  }
  auto& m = motors[motorIdx];
  if (((MOTOR_BUSY_MASK >> offset) & 1) && m->busy()) {
    // In case of prior partial write
    syncAll(motorIdx);
    return MOTOR_BUSY;
  }
  auto& mregs = mem.regs.motors[motorIdx];
  // Get a pointer to the start of this specific motor's data in the buffer
  // This translates struct-relative 'offset' to the correct absolute buffer index
  uint8_t* motorData = (uint8_t*)&mregs;
  motorData[offset] = incoming;
  uint8_t repCode = OK;
  switch (offset) {
    case offsetof(MotorBlock, cmdMove) + 3:
      repCode = m->move(mregs.cmdMove) ? OK : COMMAND_IGNORED;
      mregs.cmdMove = 0;
      mregs.stateFlags = m->getStateFlags();
      break;

    case offsetof(MotorBlock, cmdStop):
      repCode = m->stop() ? OK : COMMAND_IGNORED;
      mregs.stateFlags = m->getStateFlags();
      break;

    case offsetof(MotorBlock, currentPosition) + 3:
      m->setCurrentPosition(mregs.currentPosition);
      mregs.currentPosition = m->currentPosition();
      break;

    case offsetof(MotorBlock, settingsFlags):
      m->setSettingsFlags(mregs.settingsFlags);
      mregs.settingsFlags = m->getSettingsFlags();
      break;

    case offsetof(MotorBlock, maxSpeed) + 3:
      m->setMaxSpeed(mregs.maxSpeed);
      mregs.maxSpeed = m->maxSpeed();
      break;

    case offsetof(MotorBlock, acceleration) + 3:
      m->setAcceleration(mregs.acceleration);
      mregs.acceleration = m->acceleration();
      break;
  }
  return repCode;
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
void I2CMotors::syncAll(const uint8_t idx) {
  syncMotorSettings(idx);
  syncMotorState(idx);
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

void I2CMotors::syncMotorState(const uint8_t motorIdx) {
  auto& mregs = mem.regs.motors[motorIdx];
  auto& m = motors[motorIdx];
  mregs.stateFlags = m->getStateFlags();
  mregs.currentPosition = m->currentPosition();
  mregs.targetPosition = m->targetPosition();
  mregs.speed = m->speed();
}

void I2CMotors::syncMotorSettings(const uint8_t motorIdx) {
  auto& mregs = mem.regs.motors[motorIdx];
  auto& m = motors[motorIdx];
  if (mregs.settingsFlags != m->getSettingsFlags()) {
    mregs.settingsFlags = m->getSettingsFlags();
  }
  if (mregs.maxSpeed != m->maxSpeed()) {
    mregs.maxSpeed = m->maxSpeed();
  }
  if (mregs.acceleration != m->acceleration()) {
    mregs.acceleration = m->acceleration();
  }
}

uint8_t I2CMotors::getMidx(const uint8_t page, const uint8_t ptr) {
  return (page * 2) + (ptr >= 0x80);
}

uint8_t I2CMotors::getStructOffset(const uint8_t ptr) {
  // Motor 2 starts at exactly 0x80
  // Motor 1 starts at exactly 0x08
  return (ptr >= 0x80) ? (ptr - 0x80) : (ptr - MOTOR_BASE_ADDR);
}