#include "MotorManager.h"

namespace Moic {
ManagedMotor::ManagedMotor(IMotor* m, volatile MotorInterface& mregs, volatile MotorContext& ctx)
  : m(m), mregs(&mregs), ctx(&ctx), scriptActive(_scriptActive) {}

void ManagedMotor::tick() {
  if (!scriptActive) {
    return;
  }
  _scriptActive = MotorState::isScriptActive(m, *(mregs), *(ctx));
  if (!_scriptActive || MotorState::isMotorBusy(m, *(mregs))) {
    return;
  }
  uint8_t offset, count, code;
  if (MotorVM::processNext(*(mregs), *(ctx), offset, count, code)) {
    if (count > Moic::SCRIPT_WRITEBUF_SIZE) {
      MotorState::exitScript(m, *(mregs), *(ctx), Moic::OTHER_ERROR);
      _scriptActive = false;
      return;
    }
    for (uint8_t i = 0; i < count; ++i) {
      code = write(offset + i, ctx->scriptWriteBuf[i], true, false);
      if (code != Moic::OK) {
        MotorState::exitScript(m, *(mregs), *(ctx), code);
        _scriptActive = false;
        return;
      }
      if (!MotorState::isScriptActive(m, *(mregs), *(ctx))) {
        _scriptActive = false;
        return;
      }
    }
    _scriptActive = true;
  } else {
    MotorState::exitScript(m, *(mregs), *(ctx), code);
    _scriptActive = false;
  }
}
uint8_t ManagedMotor::write(const uint8_t offset, const uint8_t incoming, const bool enforceBusy, const bool enforceScriptLock) {
  static bool initialized = []() {
    for (const auto& entry : MotorActions::ACTION_TABLE) {
      MotorActions::ACTION_LOOKUP[entry.offset + entry.size - 1] = &entry;
    }
    return true;
  }();
  if (offset >= Moic::MOTOR_BLOCK_SIZE) {
    return Moic::UNKNOWN_COMMAND;
  }
  if (
    enforceBusy && ((Moic::MOTOR_BUSY_MASK >> offset) & 1) && MotorState::isMotorBusy(m, *(mregs)) ||
    enforceScriptLock && ((Moic::SCRIPT_LOCK_MASK >> offset) & 1) && MotorState::isScriptActive(m, *(mregs), *(ctx))
  ) {
    // In case of prior partial write
    MotorState::syncMotorSettings(m, *(mregs));
    MotorState::syncMotorState(m, *(mregs));
    return Moic::MOTOR_BUSY;
  }
  // Get a pointer to the start of this specific motor's data in the buffer
  // This translates struct-relative 'offset' to the correct absolute buffer index
  uint8_t* motorData = (uint8_t*)mregs;
  motorData[offset] = incoming;
  uint8_t repCode = Moic::OK;
  if (offset < Moic::MOTOR_BLOCK_SIZE) {
    const MotorActions::RegMapping* entry = MotorActions::ACTION_LOOKUP[offset];
    if (entry) {
      repCode = entry->handler(m, *(mregs), *(ctx));
      MotorState::syncMotorState(m, *(mregs));
      _scriptActive = MotorState::isScriptActive(m, *(mregs), *(ctx));
    }
  }
  return repCode;
}
};