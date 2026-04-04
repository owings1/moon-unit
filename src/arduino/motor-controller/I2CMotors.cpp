#include <cstddef>
#include <sys/_stdint.h>
#include <stdint.h>
#include "I2CMotors.h"

I2CMotors::I2CMotors(TwoWire& wire, Moic::ManagedMotor** mms, uint8_t count)
  : wire(wire), mms(mms), numMotors(min(MAX_MOTORS, count)) {
  memset((void*)&regs, 0, sizeof(regs));
  memset((void*)&perf, 0, sizeof(perf));
  setBootId((uint16_t)millis());

  for (uint8_t mIdx = 0; mIdx < numMotors; ++mIdx) {
    mms[mIdx]->syncMotorState();
    mms[mIdx]->syncMotorSettings();
  }
}
void I2CMotors::observeDelta(const uint32_t delta) {
  if (perfOn) {
    jitmon.observe(delta);
  }
}
bool I2CMotors::updatePerf() {
  if (perfOn && !jitmon.locked) {
    const uint32_t c = jitmon.count;
    const uint32_t m = jitmon.max_val;
    const double mn = jitmon.mean;
    const float sd = (float)jitmon.get_stdev();
    perf.count = c;
    perf.max_jitter = (uint32_t)m;
    perf.avg_jitter = (float)mn;
    perf.stdev_jitter = sd;
    return true;
  }
  return false;
}

void I2CMotors::setBootId(uint16_t id) {
  regs.bootId = id;
}

void I2CMotors::update() {
  memSyncInterval();
}

void I2CMotors::handleRead() {
  wire.write(read(currentPage, ptr++));
}

void I2CMotors::handleWrite(int howMany) {
  if (howMany < 1) {
    return;
  }
  masterWriting = true;
  ptr = wire.read();
  while (--howMany > 0) {
    regs.repCode = write(ptr++, wire.read());
  }
  masterWriting = false;
}

uint8_t I2CMotors::read(const uint8_t page, const uint8_t ptr) {
  if (ptr < MOTOR_BASE_ADDR) {
    return ((uint8_t*)&regs)[ptr];
  }
  if (page < SCRIPT_PAGE_START && ptr >= PERF_BLOCK_START) {
    if (ptr < PERF_BLOCK_END) {
      return ((uint8_t*)&perf)[ptr - PERF_BLOCK_START];
    }
    return Moic::UNSET;
  }
  const uint8_t mIdx = getMidx(page, ptr);
  if (mIdx >= numMotors) {
    return Moic::UNSET;
  }
  if (page < SCRIPT_PAGE_START) {
    if (ptr < TOTAL_BLOCK_SIZE) {
      const uint8_t offset = getStructOffset(ptr);
      return ((uint8_t*)(mms[mIdx]->mregs))[offset];
    }
    return Moic::UNSET;
  }
  if (page < SCRIPT_PAGE_END) {
    const uint8_t sIdx = (page / SCRIPT_PAGE_START) - 1;
    return mms[mIdx]->vmctx->scripts[sIdx][ptr - MOTOR_BASE_ADDR];
  }
  return Moic::UNSET;
}
uint8_t I2CMotors::write(const uint8_t ptr, const uint8_t incoming) {
  if (ptr == PAGE_REGISTER) {
    currentPage = incoming;
    return Moic::OK;
  }
  const uint8_t page = currentPage;
  if (!isWriteable(page, ptr)) {
    return Moic::READONLY_ATTRIBUTE;
  }
  if (ptr < MOTOR_BASE_ADDR) {
    ((uint8_t*)&regs)[ptr] = incoming;
    if (ptr == offsetof(DeviceMap, sysFlags)) {
      const bool newPerfOn = (incoming >> BitPerfEnabled) & 1;
      if (perfOn != newPerfOn) {
        if (newPerfOn) {
          jitmon.reset();
        }
        perfOn = newPerfOn;
      }
    }
    return Moic::OK;
  }
  const uint8_t mIdx = getMidx(page, ptr);
  if (mIdx >= numMotors) {
    return Moic::INVALID_MOTOR;
  }
  if (page < SCRIPT_PAGE_START) {
    const uint8_t offset = getStructOffset(ptr);
    return mms[mIdx]->write(offset, incoming, Moic::BUSIO);
  }
  if (page < SCRIPT_PAGE_END) {
    uint8_t sIdx = (page / SCRIPT_PAGE_START) - 1;
    if (mms[mIdx]->isPageInStack(sIdx)) {
      // Prevent writing to running script
      return Moic::MOTOR_BUSY;
    }
    mms[mIdx]->vmctx->scripts[sIdx][ptr - MOTOR_BASE_ADDR] = incoming;
    return Moic::OK;
  }
  return Moic::UNKNOWN_COMMAND;
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
      mms[mIdx]->syncMotorState();
    }
  }
  // 2. Slow Sync (Settings/Config) - Every 500ms
  if (now - lastSlowSync >= SLOW_SYNC_MS) {
    lastSlowSync = now;
    for (uint8_t mIdx = 0; mIdx < numMotors; ++mIdx) {
      mms[mIdx]->syncMotorSettings();
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
  return (Moic::OFFSET_WRITEMASKS[offset] & (1 << Moic::BUSIO));
}