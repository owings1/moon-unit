#include <sys/_stdint.h>
#include "MotorVM.h"

namespace MotorVM {

uint8_t jump(Moic::ManagedMotor& mm, volatile uint8_t* cmdBuf) {
  auto& vmctx = *(mm.vmctx);
  uint8_t page = cmdBuf[0];
  const uint8_t sIdx = cmdBuf[1];
  if (page == Moic::UNSET) {
    page = vmctx.page;
  }
  if (page >= Moic::NUM_SCRIPT_PAGES || sIdx >= Moic::SCRIPT_PAGE_SIZE) {
    return Moic::OVERFLOW;
  }
  vmctx.page = page;
  vmctx.idx = sIdx;
  return Moic::OK;
}

uint8_t call(Moic::ManagedMotor& mm, volatile uint8_t* cmdBuf) {
  uint8_t page = cmdBuf[0];
  const uint8_t sIdx = cmdBuf[1];
  if (page == Moic::UNSET) {
    page = mm.vmctx->page;
  }
  return scriptStackPush(mm, page, sIdx);
}

uint8_t condCall(Moic::ManagedMotor& mm, volatile uint8_t* cmdBuf) {
  const uint8_t func = cmdBuf[0];
  const uint8_t rhs = cmdBuf[1];
  const int8_t result = scriptCondition(mm, func, rhs);
  if (result < 0) {
    return Moic::UNKNOWN_COMMAND;
  }
  if (result > 0) {
    return call(mm, &cmdBuf[2]);
  }
  return Moic::OK;
}

uint8_t condJump(Moic::ManagedMotor& mm, volatile uint8_t* cmdBuf) {
  const uint8_t func = cmdBuf[0];
  const uint8_t rhs = cmdBuf[1];
  const int8_t result = scriptCondition(mm, func, rhs);
  if (result < 0) {
    return Moic::UNKNOWN_COMMAND;
  }
  if (result > 0) {
    return jump(mm, &cmdBuf[2]);
  }
  return Moic::OK;
}

bool processNext(Moic::ManagedMotor& mm) {
  auto& vmctx = *(mm.vmctx);
  const uint8_t scriptPage = vmctx.page;
  const uint8_t startIdx = vmctx.idx;
  if (scriptPage >= Moic::NUM_SCRIPT_PAGES) {
    vmctx.exitCode = Moic::OVERFLOW;
    return false;
  }
  if (startIdx >= Moic::SCRIPT_PAGE_SIZE) {
    vmctx.exitCode = Moic::OVERFLOW;
    return false;
  }
  // Get OpCode (Struct Offset)
  volatile uint8_t* scriptBase = vmctx.scripts[scriptPage];
  const uint8_t op = scriptBase[startIdx];
  if (op == Moic::OK || op >= Moic::USR1) {  // End markers
    // OK 0x00 or UNSET 0xFF are treated as OK 0x00.
    // Anything USR1 0xFA to USR5 0xFE are passed on as is to
    // the scriptRepCode, which can be evaluated in conditional
    // functions with EQL_RETURNCODE_RHS 0x03. A subroutine can
    // use this as a return code to the caller.
    const uint8_t code = (op == Moic::OK || op == Moic::UNSET) ? Moic::OK : op;
    if (vmctx.sp > 0) {
      scriptStackPop(mm, code);
      vmctx.count = 0;
      return true;
    } else {
      vmctx.exitCode = code;
      return false;
    }
  }
  const bool isFar = op & FARPTR_OPCODE_FLAG;
  const bool isInd = op & INDIRECT_OPCODE_FLAG;
  const uint8_t directOp = op & ~(INDIRECT_OPCODE_FLAG | FARPTR_OPCODE_FLAG);
  const uint8_t dataLen = getOpCodeDataLength(directOp);
  if (dataLen == 0) {
    vmctx.exitCode = Moic::UNKNOWN_COMMAND;
    return false;
  }
  if (dataLen > Moic::SCRIPT_WRITEBUF_SIZE) {
    vmctx.exitCode = Moic::OTHER_ERROR;
    return false;
  }
  const uint8_t operandLen = isFar ? 2 : (isInd ? 1 : dataLen);
  const uint8_t totalCmdLen = 1 + operandLen;
  if (startIdx + totalCmdLen >= Moic::SCRIPT_PAGE_SIZE) {
    vmctx.exitCode = Moic::OVERFLOW;
    return false;
  }
  // Consume from buffer (Advance BEFORE execution)
  vmctx.idx += totalCmdLen;
  uint8_t ptrOffset, farPage;
  if (isFar) {
    farPage = scriptBase[startIdx + 1];
    ptrOffset = scriptBase[startIdx + 2];
    if (farPage >= Moic::NUM_SCRIPT_PAGES || ((uint16_t)ptrOffset) + dataLen >= Moic::SCRIPT_PAGE_SIZE) {
      vmctx.exitCode = Moic::OVERFLOW;
      return false;
    }
  } else if (isInd) {
    ptrOffset = scriptBase[startIdx + 1];
    if (((uint16_t)ptrOffset) + dataLen > 0x1B) {
      vmctx.exitCode = Moic::UNINVITED_POINTER;
      return false;
    }
  }
  // We feed bytes one-by-one to handleMotorWrite to reuse all logic
  for (uint8_t i = 0; i < dataLen; i++) {
    uint8_t incoming;
    if (isFar) {
      incoming = vmctx.scripts[farPage][ptrOffset + i];
    } else if (isInd) {
      incoming = ((uint8_t*)mm.mregs)[ptrOffset + i];
    } else {
      incoming = scriptBase[startIdx + i + 1];
    }
    vmctx.writeBuf[i] = incoming;
  }
  vmctx.count = dataLen;
  vmctx.offset = directOp;
  return true;
}

}

void scriptStackPop(Moic::ManagedMotor& mm, const uint8_t code) {
  auto& vmctx = *(mm.vmctx);
  if (vmctx.sp <= 0) {
    return;
  }
  vmctx.sp--;
  vmctx.exitCode = code;
  auto& stack = vmctx.stack[vmctx.sp];
  vmctx.page = stack.page;
  vmctx.idx = stack.idx;
  vmctx.rhsArg = stack.rhsArg;
  vmctx.callArg = stack.callArg;
}

uint8_t scriptStackPush(Moic::ManagedMotor& mm, uint8_t page, const uint8_t sIdx) {
  auto& vmctx = *(mm.vmctx);
  if (page == Moic::UNSET) {
    page = vmctx.page;
  }
  if (page >= Moic::NUM_SCRIPT_PAGES || sIdx >= Moic::SCRIPT_PAGE_SIZE || vmctx.sp >= Moic::SCRIPT_STACK_SIZE) {
    return Moic::OVERFLOW;
  }
  auto& stack = vmctx.stack[vmctx.sp];
  stack.page = vmctx.page;
  stack.idx = vmctx.idx;
  stack.rhsArg = vmctx.rhsArg;
  stack.callArg = vmctx.callArg;
  vmctx.sp++;
  vmctx.page = page;
  vmctx.idx = sIdx;
  vmctx.callArg = vmctx.rhsArg;
  return Moic::OK;
}

int8_t scriptCondition(Moic::ManagedMotor& mm, const uint8_t func, const uint8_t rhs) {
  auto& vmctx = *(mm.vmctx);
  // Bit 7 of the function code signifies negation.
  const bool negate = func & 0x80;
  bool result = false;
  const uint8_t funId = func & 0x7F;
  switch (funId) {
    case Moic::ALWAYS_TRUE:
      result = true;
      break;
    case Moic::AND_STATEFLAGS_RHS:
      result = mm.mregs->stateFlags & rhs;
      break;
    case Moic::AND_SETTINGSFLAGS_RHS:
      result = mm.mregs->settingsFlags & rhs;
      break;
    case Moic::EQL_RETURNCODE_RHS:
      result = vmctx.exitCode == rhs;
      break;
    // The RHS value of the previous condition check can be evaluated
    // with EQL_LASTCONDARG_RHS 0x30 or AND_LASTCONDARG_RHS 0x31. This
    // can be used with ALWAYS_TRUE to pass an argument to a subroutine.
    case Moic::EQL_LASTCONDARG_RHS:
      result = vmctx.rhsArg == rhs;
      break;
    case Moic::AND_LASTCONDARG_RHS:
      result = vmctx.rhsArg & rhs;
      break;
    // The CALLARG is set to the last RHS value when the stack is pushed
    // and remains immutable for that context, until it is restored to its
    // previous value when the stack is popped.
    case Moic::EQL_CALLARG_RHS:
      result = vmctx.callArg == rhs;
      break;
    case Moic::AND_CALLARG_RHS:
      result = vmctx.callArg & rhs;
      break;
    default:
      return -1;
  }
  if (funId < Moic::EQL_LASTCONDARG_RHS || funId > Moic::AND_LASTCONDARG_RHS) {
    // Only update the register if the operation wasn't a 'Read' of the register
    vmctx.rhsArg = rhs;
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
