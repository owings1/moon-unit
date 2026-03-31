#include <cstddef>
#include <sys/_stdint.h>
#include <stdint.h>
#include "I2CMotors.h"

I2CMotors::I2CMotors(TwoWire& wire, IMotor** motors, Moic::MotorContext** contexts, uint8_t count)
  : wire(wire), motors(motors),contexts(contexts), numMotors(min(MAX_MOTORS, count)) {
  memset((void*)mem.buffer, 0, sizeof(mem.buffer));
  setBootId((uint16_t)millis());
  for (uint8_t mIdx = 0; mIdx < numMotors; ++mIdx) {
    MotorState::syncMotorSettings(motors[mIdx], mem.regs.motors[mIdx]);
    MotorState::syncMotorState(motors[mIdx], mem.regs.motors[mIdx]);
  }
}

void I2CMotors::setBootId(uint16_t id) {
  mem.regs.bootId = id;
}

void I2CMotors::update() {
  memSyncInterval();
}

void I2CMotors::handleRead() {
  if (ptr < MOTOR_BASE_ADDR) {
    wire.write(mem.buffer[ptr]);
  } else {
    const uint8_t mIdx = getMidx(currentPage, ptr);
    if (mIdx >= numMotors) {
      wire.write(Moic::UNSET);
    } else if (currentPage < SCRIPT_PAGE_START) {
      if (ptr < TOTAL_BLOCK_SIZE) {
        const uint8_t offset = getStructOffset(ptr);
        uint8_t* motorData = (uint8_t*)&mem.regs.motors[mIdx];
        wire.write(motorData[offset]);
      } else {
        wire.write(Moic::UNSET);
      }
    } else if (currentPage < SCRIPT_PAGE_START * (Moic::NUM_SCRIPT_PAGES + 1)) {
      const uint8_t sIdx = (currentPage / SCRIPT_PAGE_START) - 1;
      wire.write(contexts[mIdx]->scripts[sIdx][ptr - MOTOR_BASE_ADDR]);
    } else {
      wire.write(Moic::UNSET);
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
      mem.regs.repCode = Moic::OK;
    } else if (isWriteable(currentPage, ptr)) {
      if (ptr < MOTOR_BASE_ADDR) {
        mem.buffer[ptr] = incoming;
        mem.regs.repCode = Moic::OK;
      } else {
        uint8_t mIdx = getMidx(currentPage, ptr);
        if (mIdx >= numMotors) {
          mem.regs.repCode = Moic::INVALID_MOTOR;
        } else if (currentPage < SCRIPT_PAGE_START) {
          uint8_t offset = getStructOffset(ptr);
          auto& mregs = mem.regs.motors[mIdx];
          auto& ctx = *(contexts[mIdx]);
          mem.regs.repCode = MotorActions::write(motors[mIdx], mregs, ctx, offset, incoming, true, true);

          // Update the fragile gatekeeper
          if (MotorState::isScriptActive(motors[mIdx], mregs, ctx)) {
            _tmp_scriptActiveMask |= (1 << mIdx);
          } else {
            // Also handle manual stops via I2C
            _tmp_scriptActiveMask &= ~(1 << mIdx);
          }

        } else if (currentPage < SCRIPT_PAGE_START * (Moic::NUM_SCRIPT_PAGES + 1)) {
          uint8_t sIdx = (currentPage / SCRIPT_PAGE_START) - 1;
          auto& mregs = mem.regs.motors[mIdx];
          auto& ctx = *(contexts[mIdx]);
          if (MotorState::isScriptActive(motors[mIdx], mregs, ctx) && MotorState::isPageInStack(motors[mIdx], mregs, ctx, sIdx)) {
            // Prevent writing to running script
            mem.regs.repCode = Moic::MOTOR_BUSY;
          } else {
            ctx.scripts[sIdx][ptr - MOTOR_BASE_ADDR] = incoming;
            mem.regs.repCode = Moic::OK;
          }
        } else {
          mem.regs.repCode = Moic::UNKNOWN_COMMAND;
        }
      }
    } else {
      mem.regs.repCode = Moic::READONLY_ATTRIBUTE;
    }
    ptr++;
    howMany--;
  }
  masterWriting = false;
}

uint8_t I2CMotors::getTmpScriptMask() {
  return _tmp_scriptActiveMask;
}
void I2CMotors::clearTmpScriptBit(const uint8_t mIdx) {
  _tmp_scriptActiveMask &= ~(1 << mIdx);
}

void I2CMotors::memSyncInterval() {
  if (masterWriting) {
    return;
  }
  const uint32_t now = millis();
  // 1. Fast Sync (High-priority telemetry)
  if (now - lastFastSync >= FAST_SYNC_MS) {
    lastFastSync = now;
    for (uint8_t mIdx = 0; mIdx < numMotors; ++mIdx) {
      MotorState::syncMotorState(motors[mIdx], mem.regs.motors[mIdx]);
    }
  }
  // 2. Slow Sync (Settings/Config) - Every 500ms
  if (now - lastSlowSync >= SLOW_SYNC_MS) {
    lastSlowSync = now;
    for (uint8_t mIdx = 0; mIdx < numMotors; ++mIdx) {
      MotorState::syncMotorSettings(motors[mIdx], mem.regs.motors[mIdx]);
    }
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