#include <sys/_stdint.h>
#include "MotorVM.h"

namespace MotorVM {

uint8_t jump(volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, volatile uint8_t* cmdBuf) {
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

uint8_t call(volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, volatile uint8_t* cmdBuf) {
  uint8_t page = cmdBuf[0];
  const uint8_t sIdx = cmdBuf[1];
  if (page == Moic::UNSET) {
    page = mregs.scriptPage;
  }
  return scriptStackPush(mregs, ctx, page, sIdx);
}

uint8_t condCall(volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, volatile uint8_t* cmdBuf) {
  const uint8_t func = cmdBuf[0];
  const uint8_t rhs = cmdBuf[1];
  const int8_t result = scriptCondition(mregs, ctx, func, rhs);
  if (result < 0) {
    return Moic::UNKNOWN_COMMAND;
  }
  if (result > 0) {
    return call(mregs, ctx, &cmdBuf[2]);
  }
  return Moic::OK;
}

uint8_t condJump(volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, volatile uint8_t* cmdBuf) {
  const uint8_t func = cmdBuf[0];
  const uint8_t rhs = cmdBuf[1];
  const int8_t result = scriptCondition(mregs, ctx, func, rhs);
  if (result < 0) {
    return Moic::UNKNOWN_COMMAND;
  }
  if (result > 0) {
    return jump(mregs, ctx, &cmdBuf[2]);
  }
  return Moic::OK;
}

bool processNext(volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, uint8_t& offset, uint8_t& count, uint8_t& exitCode) {
  const uint8_t scriptPage = mregs.scriptPage;
  const uint8_t startIdx = mregs.scriptIdx;
  if (scriptPage >= Moic::NUM_SCRIPT_PAGES) {
    exitCode = Moic::OVERFLOW;
    return false;
  }
  if (startIdx >= Moic::SCRIPT_PAGE_SIZE) {
    exitCode = Moic::OVERFLOW;
    return false;
  }
  // Get OpCode (Struct Offset)
  volatile uint8_t* scriptBase = ctx.scripts[scriptPage];
  const uint8_t op = scriptBase[startIdx];
  if (op == Moic::OK || op >= Moic::USR1) {  // End markers
    // OK 0x00 or UNSET 0xFF are treated as OK 0x00.
    // Anything USR1 0xFA to USR5 0xFE are passed on as is to
    // the scriptRepCode, which can be evaluated in conditional
    // functions with EQL_RETURNCODE_RHS 0x03. A subroutine can
    // use this as a return code to the caller.
    const uint8_t code = (op == Moic::OK || op == Moic::UNSET) ? Moic::OK : op;
    if (ctx.sp > 0) {
      scriptStackPop(mregs, ctx, code);
      count = 0;
      return true;
    } else {
      exitCode = code;
      return false;
    }
  }
  const bool isFar = op & FARPTR_OPCODE_FLAG;
  const bool isInd = op & INDIRECT_OPCODE_FLAG;
  const uint8_t directOp = op & ~(INDIRECT_OPCODE_FLAG | FARPTR_OPCODE_FLAG);
  const uint8_t dataLen = getOpCodeDataLength(directOp);
  if (dataLen == 0) {
    exitCode = Moic::UNKNOWN_COMMAND;
    return false;
  }
  if (dataLen > Moic::SCRIPT_WRITEBUF_SIZE) {
    exitCode = Moic::OTHER_ERROR;
    return false;
  }
  const uint8_t operandLen = isFar ? 2 : (isInd ? 1 : dataLen);
  const uint8_t totalCmdLen = 1 + operandLen;
  if (startIdx + totalCmdLen >= Moic::SCRIPT_PAGE_SIZE) {
    exitCode = Moic::OVERFLOW;
    return false;
  }
  // Consume from buffer (Advance BEFORE execution)
  mregs.scriptIdx += totalCmdLen;
  uint8_t ptrOffset, farPage;
  if (isFar) {
    farPage = scriptBase[startIdx + 1];
    ptrOffset = scriptBase[startIdx + 2];
    if (farPage >= Moic::NUM_SCRIPT_PAGES || ((uint16_t)ptrOffset) + dataLen >= Moic::SCRIPT_PAGE_SIZE) {
      exitCode = Moic::OVERFLOW;
      return false;
    }
  } else if (isInd) {
    ptrOffset = scriptBase[startIdx + 1];
    if (((uint16_t)ptrOffset) + dataLen > 0x1B) {
      exitCode = Moic::UNINVITED_POINTER;
      return false;
    }
  }
  // We feed bytes one-by-one to handleMotorWrite to reuse all logic
  for (uint8_t i = 0; i < dataLen; i++) {
    uint8_t incoming;
    if (isFar) {
      incoming = ctx.scripts[farPage][ptrOffset + i];
    } else if (isInd) {
      incoming = ((uint8_t*)&mregs)[ptrOffset + i];
    } else {
      incoming = scriptBase[startIdx + i + 1];
    }
    ctx.scriptWriteBuf[i] = incoming;
  }
  count = dataLen;
  offset = directOp;
  return true;
}

}

void scriptStackPop(volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, const uint8_t code) {
  if (ctx.sp <= 0) {
    return;
  }
  ctx.sp--;
  mregs.scriptRepCode = code;
  mregs.scriptPage = ctx.scriptStackPage[ctx.sp];
  mregs.scriptIdx = ctx.scriptStackIdx[ctx.sp];
  ctx.scriptLastRhs = ctx.scriptStackRhsArg[ctx.sp];
  ctx.scriptCallArg = ctx.scriptStackCallArg[ctx.sp];
}

uint8_t scriptStackPush(volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, uint8_t page, const uint8_t sIdx) {
  if (page == Moic::UNSET) {
    page = mregs.scriptPage;
  }
  if (page >= Moic::NUM_SCRIPT_PAGES || sIdx >= Moic::SCRIPT_PAGE_SIZE || ctx.sp >= Moic::SCRIPT_STACK_SIZE) {
    return Moic::OVERFLOW;
  }
  ctx.scriptStackIdx[ctx.sp] = mregs.scriptIdx;
  ctx.scriptStackPage[ctx.sp] = mregs.scriptPage;
  ctx.scriptStackRhsArg[ctx.sp] = ctx.scriptLastRhs;
  ctx.scriptStackCallArg[ctx.sp] = ctx.scriptCallArg;
  ctx.sp++;
  mregs.scriptPage = page;
  mregs.scriptIdx = sIdx;
  ctx.scriptCallArg = ctx.scriptLastRhs;
  return Moic::OK;
}

int8_t scriptCondition(volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, const uint8_t func, const uint8_t rhs) {
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
      result = ctx.scriptLastRhs == rhs;
      break;
    case Moic::AND_LASTCONDARG_RHS:
      result = ctx.scriptLastRhs & rhs;
      break;
    // The CALLARG is set to the last RHS value when the stack is pushed
    // and remains immutable for that context, until it is restored to its
    // previous value when the stack is popped.
    case Moic::EQL_CALLARG_RHS:
      result = ctx.scriptCallArg == rhs;
      break;
    case Moic::AND_CALLARG_RHS:
      result = ctx.scriptCallArg & rhs;
      break;
    default:
      return -1;
  }
  if (funId < Moic::EQL_LASTCONDARG_RHS || funId > Moic::AND_LASTCONDARG_RHS) {
    // Only update the register if the operation wasn't a 'Read' of the register
    ctx.scriptLastRhs = rhs;
  }
  return result ^ negate;
}
uint8_t getOpCodeDataLength(const uint8_t offset) {
  switch (offset) {
    case offsetof(Moic::MotorInterface, currentPosition):
    case offsetof(Moic::MotorInterface, maxSpeed):
    case offsetof(Moic::MotorInterface, acceleration):
    case offsetof(Moic::MotorInterface, cmdMove):
    case offsetof(Moic::MotorInterface, cmdMoveRev):
    case offsetof(Moic::MotorInterface, cmdMoveTo):
    case offsetof(Moic::MotorInterface, cmdDelay):
    case offsetof(Moic::MotorInterface, cmdCondCall):
    case offsetof(Moic::MotorInterface, cmdCondJump):
      return 4;
    case offsetof(Moic::MotorInterface, sleepTimeoutMs):
    case offsetof(Moic::MotorInterface, cmdCall):
    case offsetof(Moic::MotorInterface, cmdJump):
      return 2;
    case offsetof(Moic::MotorInterface, settingsFlags):
    case offsetof(Moic::MotorInterface, enableDelayMs):
    case offsetof(Moic::MotorInterface, cmdStop):
      return 1;
    default:
      return 0;
  }
}
