#include "I2CMotors.h"
#include "Motor.h"

I2CMotors::I2CMotors(TwoWire& wire, Motor* motorPtr, uint8_t motorCount)
  : wire(wire), motors(motorPtr), numMotors(motorCount) {
  memset((void*)mem.buffer, 0, sizeof(mem.buffer));
  setBootId((uint16_t) millis());
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
  if (ptr < sizeof(mem.buffer)) {
    wire.write(mem.buffer[ptr]);
    ptr++;
  } else {
    // Send NUL/FF if master over-reads
    wire.write(0xFF);
  }
}

void I2CMotors::handleWrite(int howMany) {
  if (howMany < 1) {
    return;
  }
  masterWriting = true;
  ptr = wire.read();
  howMany--;
  while (howMany > 0 && ptr < sizeof(mem.buffer)) {
    const uint8_t incoming = wire.read();
    if (isWriteable(ptr)) {
      if (ptr >= MOTOR_BASE_ADDR) {
        uint8_t mIdx = (ptr - MOTOR_BASE_ADDR) / MOTOR_BLOCK_SIZE;
        uint8_t offset = motorOffset(ptr);
        handleMotorWrite(mIdx, offset, incoming);
      } else {
        mem.buffer[ptr] = incoming;
      }
    } else {
      mem.regs.repCode = READONLY_ATTRIBUTE;
    }
    ptr++;
    howMany--;
  }
  masterWriting = false;
}

void I2CMotors::handleMotorWrite(const uint8_t motorIdx, const uint8_t offset, const uint8_t incoming) {
  auto& repCode = mem.regs.repCode;
  if (motorIdx >= numMotors) {
    repCode = INVALID_MOTORID;
    return;
  }
  Motor& m = motors[motorIdx];
  auto& mregs = mem.regs.motors[motorIdx];
  if (((MOTOR_BUSY_MASK >> offset) & 1) && ((m.stateFlags >> Motor::BitIsMoving) & 1)) {
    repCode = MOTOR_BUSY;
    // In case of prior partial write
    syncAll(m);
    return;
  }
  mem.buffer[ptr] = incoming;
  repCode = OK;
  switch (offset) {
    case offsetof(MotorBlock, cmdMove) + 3:
      repCode = m.move(mregs.cmdMove) ? OK : COMMAND_IGNORED;
      mregs.cmdMove = 0;
      mregs.stateFlags = m.stateFlags;
      break;

    case offsetof(MotorBlock, cmdStop):
      repCode = m.stop() ? OK : COMMAND_IGNORED;
      mregs.stateFlags = m.stateFlags;
      break;

    case offsetof(MotorBlock, pos) + 3:
      m.setCurrentPosition(mregs.pos);
      mregs.pos = m.currentPosition();
      break;

    case offsetof(MotorBlock, settingsFlags):
      m.setSettingsFlags(mregs.settingsFlags);
      mregs.settingsFlags = m.settingsFlags;
      break;

    case offsetof(MotorBlock, maxSpeed) + 1:
      m.setMaxSpeed(mregs.maxSpeed);
      mregs.maxSpeed = m.maxSpeed();
      break;

    case offsetof(MotorBlock, acceleration) + 1:
      m.setAcceleration(mregs.acceleration);
      mregs.acceleration = m.acceleration();
      break;
  }
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
  syncMotorState();
  syncMotorSettings();
}

void I2CMotors::syncMotorState() {
  for (uint8_t i = 0; i < numMotors; ++i) {
    syncMotorState(motors[i]);
  }
}

void I2CMotors::syncMotorSettings() {
  for (uint8_t i = 0; i < numMotors; ++i) {
    syncMotorSettings(motors[i]);
  }
}

void I2CMotors::syncAll(Motor& m) {
  syncMotorState(m);
  syncMotorSettings(m);
}

void I2CMotors::syncMotorState(Motor& m) {
  auto& mregs = mem.regs.motors[m.id - 1];
  mregs.stateFlags = m.stateFlags;
  mregs.pos = m.currentPosition();
  mregs.targetPos = m.targetPosition();
  mregs.speed = m.speed();
}

void I2CMotors::syncMotorSettings(Motor& m) {
  auto& mregs = mem.regs.motors[m.id - 1];
  if (mregs.settingsFlags != m.settingsFlags) {
    mregs.settingsFlags = m.settingsFlags;
  }
  if (mregs.maxSpeed != m.maxSpeed()) {
    mregs.maxSpeed = m.maxSpeed();
  }
  if (mregs.acceleration != m.acceleration()) {
    mregs.acceleration = m.acceleration();
  }
}

bool I2CMotors::isWriteable(uint8_t p) {
  if (p < MOTOR_BASE_ADDR) {
    return (DEVICE_WRITE_MASK >> p) & 1;
  }
  const uint8_t offset = motorOffset(p);
  if (offset >= MOTOR_BLOCK_SIZE) {
    return false;
  }
  return (MOTOR_WRITE_MASK >> offset) & 1;
}

uint8_t I2CMotors::motorOffset(const uint8_t p) {
  return (p - MOTOR_BASE_ADDR) % MOTOR_BLOCK_SIZE;
}