#include <sys/_stdint.h>
#include "MotorVM.h"

namespace MotorVM {

uint8_t jump(volatile Moic::MotorBlock& mregs, volatile uint8_t* cmdBuf) {
  uint8_t page = cmdBuf[0];
  const uint8_t sIdx = cmdBuf[1];
  if (page == Moic::UNSET) {
    page = mregs.scriptPage;
  }
  if (page >= Moic::NUM_SCRIPT_PAGES || sIdx >= Moic::SCRIPT_PAGE_SIZE) {
    return Moic::OVERFLOW;
  }
  mregs.scriptPage = page;
  mregs.scriptIdx = sIdx;
  return Moic::OK;
}

uint8_t call(volatile Moic::MotorBlock& mregs, volatile uint8_t* cmdBuf) {
  uint8_t page = cmdBuf[0];
  const uint8_t sIdx = cmdBuf[1];
  if (page == Moic::UNSET) {
    page = mregs.scriptPage;
  }
  return scriptStackPush(mregs, page, sIdx);
}

uint8_t condCall(volatile Moic::MotorBlock& mregs, volatile uint8_t* cmdBuf) {
  const uint8_t func = cmdBuf[0];
  const uint8_t rhs = cmdBuf[1];
  const int8_t result = scriptCondition(mregs, func, rhs);
  if (result < 0) {
    return Moic::UNKNOWN_COMMAND;
  }
  if (result > 0) {
    return call(mregs, &cmdBuf[2]);
  }
  return Moic::OK;
}

uint8_t condJump(volatile Moic::MotorBlock& mregs, volatile uint8_t* cmdBuf) {
  const uint8_t func = cmdBuf[0];
  const uint8_t rhs = cmdBuf[1];
  const int8_t result = scriptCondition(mregs, func, rhs);
  if (result < 0) {
    return Moic::UNKNOWN_COMMAND;
  }
  if (result > 0) {
    return jump(mregs, &cmdBuf[2]);
  }
  return Moic::OK;
}


bool processNext(volatile Moic::MotorBlock& mregs, uint8_t& offset, uint8_t& count, uint8_t& exitCode) {
  if (mregs.scriptPage >= Moic::NUM_SCRIPT_PAGES) {
    exitCode = Moic::OVERFLOW;
    return false;
  }
  if (mregs.scriptIdx >= Moic::SCRIPT_PAGE_SIZE) {
    exitCode = Moic::OVERFLOW;
    return false;
  }
  // Get OpCode (Struct Offset)
  volatile uint8_t* scriptBase = mregs.scripts[mregs.scriptPage];
  const uint8_t op = scriptBase[mregs.scriptIdx];
  if (op == Moic::OK || op >= Moic::USR1) {  // End markers
    // OK 0x00 or UNSET 0xFF are treated as OK 0x00.
    // Anything USR1 0xFA to USR5 0xFE are passed on as is to
    // the scriptRepCode, which can be evaluated in conditional
    // functions with EQL_RETURNCODE_RHS 0x03. A subroutine can
    // use this as a return code to the caller.
    const uint8_t code = (op == Moic::OK || op == Moic::UNSET) ? Moic::OK : op;
    if (mregs.sp > 0) {
      scriptStackPop(mregs, code);
      count = 0;
      return true;
    } else {
      exitCode = code;
      return false;
    }
  }
  const bool isIndirect = op & INDIRECT_OPCODE_FLAG;
  const uint8_t directOp = op & ~INDIRECT_OPCODE_FLAG;
  const uint8_t dataLen = getOpCodeDataLength(directOp);
  if (dataLen == 0) {
    exitCode = Moic::UNKNOWN_COMMAND;
    return false;
  }
  if (dataLen >= Moic::SCRIPT_WRITEBUF_SIZE) {
    exitCode = Moic::OTHER_ERROR;
    return false;
  }
  const uint8_t totalCmdLen = 1 + (isIndirect ? 1 : dataLen);
  if (mregs.scriptIdx + totalCmdLen >= Moic::SCRIPT_PAGE_SIZE) {
    exitCode = Moic::OTHER_ERROR;
    return false;
  }
  // Consume from buffer (Advance BEFORE execution)
  const uint8_t currentCmdStart = mregs.scriptIdx;
  mregs.scriptIdx += totalCmdLen;
  // We feed bytes one-by-one to handleMotorWrite to reuse all logic
  for (uint8_t i = 0; i < dataLen; i++) {
    uint8_t incoming;
    if (isIndirect) {
      uint8_t ptrOffset = scriptBase[currentCmdStart + 1];
      if (ptrOffset > 0x1B) {
        exitCode = Moic::UNINVITED_POINTER;
        return false;
      }
      incoming = ((uint8_t*)&mregs)[ptrOffset + i];
    } else {
      incoming = scriptBase[currentCmdStart + i + 1];
    }
    mregs.scriptWriteBuf[i] = incoming;
  }
  count = dataLen;
  offset = directOp;
  return true;
}

}

void scriptStackPop(volatile Moic::MotorBlock& mregs, const uint8_t code) {
  if (mregs.sp <= 0) {
    return;
  }
  mregs.sp--;
  mregs.scriptRepCode = code;
  mregs.scriptPage = mregs.scriptStackPage[mregs.sp];
  mregs.scriptIdx = mregs.scriptStackIdx[mregs.sp];
  mregs.scriptLastRhs = mregs.scriptStackRhsArg[mregs.sp];
}

uint8_t scriptStackPush(volatile Moic::MotorBlock& mregs, uint8_t page, const uint8_t sIdx) {
  if (page == Moic::UNSET) {
    page = mregs.scriptPage;
  }
  if (page >= Moic::NUM_SCRIPT_PAGES || sIdx >= Moic::SCRIPT_PAGE_SIZE || mregs.sp >= Moic::SCRIPT_STACK_SIZE) {
    return Moic::OVERFLOW;
  }
  mregs.scriptStackIdx[mregs.sp] = mregs.scriptIdx;
  mregs.scriptStackPage[mregs.sp] = mregs.scriptPage;
  mregs.scriptStackRhsArg[mregs.sp] = mregs.scriptLastRhs;
  mregs.sp++;
  mregs.scriptPage = page;
  mregs.scriptIdx = sIdx;
  return Moic::OK;
}

int8_t scriptCondition(volatile Moic::MotorBlock& mregs, const uint8_t func, const uint8_t rhs) {
  // Bit 7 of the function code signifies negation.
  const bool negate = func & 0x80;
  bool result = false;
  const uint8_t funId = func & 0x7F;
  switch (funId) {
    case Moic::ALWAYS_TRUE:
      result = true;
      break;
    case Moic::AND_STATEFLAGS_RHS:
      result = mregs.stateFlags & rhs;
      break;
    case Moic::AND_SETTINGSFLAGS_RHS:
      result = mregs.settingsFlags & rhs;
      break;
    case Moic::EQL_RETURNCODE_RHS:
      result = mregs.scriptRepCode == rhs;
      break;
    // The RHS value of the previous condition check can be evaluated
    // with EQL_LASTCONDARG_RHS 0x30 or AND_LASTCONDARG_RHS 0x31. This
    // can be used with ALWAYS_TRUE to pass an argument to a subroutine.
    case Moic::EQL_LASTCONDARG_RHS:
      result = mregs.scriptLastRhs == rhs;
      break;
    case Moic::AND_LASTCONDARG_RHS:
      result = mregs.scriptLastRhs & rhs;
      break;
    default:
      return -1;
  }
  if (funId < Moic::EQL_LASTCONDARG_RHS || funId > Moic::AND_LASTCONDARG_RHS) {
    // Only update the register if the operation wasn't a 'Read' of the register
    mregs.scriptLastRhs = rhs;
  }
  return result ^ negate;
}
uint8_t getOpCodeDataLength(const uint8_t offset) {
  switch (offset) {
    case offsetof(Moic::MotorBlock, currentPosition):
    case offsetof(Moic::MotorBlock, maxSpeed):
    case offsetof(Moic::MotorBlock, acceleration):
    case offsetof(Moic::MotorBlock, cmdMove):
    case offsetof(Moic::MotorBlock, cmdMoveTo):
    case offsetof(Moic::MotorBlock, cmdDelay):
    case offsetof(Moic::MotorBlock, cmdCondCall):
    case offsetof(Moic::MotorBlock, cmdCondJump):
      return 4;
    case offsetof(Moic::MotorBlock, sleepTimeoutMs):
    case offsetof(Moic::MotorBlock, cmdCall):
    case offsetof(Moic::MotorBlock, cmdJump):
      return 2;
    case offsetof(Moic::MotorBlock, settingsFlags):
    case offsetof(Moic::MotorBlock, enableDelayMs):
    case offsetof(Moic::MotorBlock, cmdStop):
      return 1;
    default:
      return 0;
  }
}
