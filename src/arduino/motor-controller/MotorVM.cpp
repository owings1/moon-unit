#include <sys/_stdint.h>
#include "MotorVM.h"

namespace MotorVM {

bool tick(Moic::ManagedMotor& mm) {
  auto& vmctx = *(mm.vmctx);
  const uint8_t page = vmctx.page;
  uint8_t idx = vmctx.idx;
  if (page >= Moic::NUM_SCRIPT_PAGES) {
    vmctx.exitCode = Moic::OVERFLOW;
    return false;
  }
  if (idx >= Moic::SCRIPT_PAGE_SIZE) {
    vmctx.exitCode = Moic::OVERFLOW;
    return false;
  }
  // Get OpCode (Struct Offset)
  volatile uint8_t* script = vmctx.scripts[page];
  uint8_t op = script[idx];
  if (op == Moic::OK || op >= Moic::USR1) {  // End markers
    // OK 0x00 or UNSET 0xFF are treated as OK 0x00.
    // Anything USR1 0xFA to USR5 0xFE are passed on as is to
    // the scriptRepCode, which can be evaluated in conditional
    // functions with EQL_RETURNCODE_RHS 0x03. A subroutine can
    // use this as a return code to the caller.
    const uint8_t code = (op == Moic::OK || op == Moic::UNSET) ? Moic::OK : op;
    if (vmctx.sp > 0) {
      popStack(mm, code);
      vmctx.count = 0;
      return true;
    } else {
      vmctx.exitCode = code;
      return false;
    }
  }
  // Control escape code for execution flow constructs such as loops.
  const bool isCtl = op == CONTROL_EXCODE;
  if (isCtl) {
    if (((uint16_t)idx) + 1 >= Moic::SCRIPT_PAGE_SIZE) {
      vmctx.exitCode = Moic::OVERFLOW;
      return false;
    }
    idx += 1;
    op = script[idx];
    vmctx.idx += 1;
  }
  const bool isFar = op & FARPTR_OPCODE_FLAG;
  const bool isInd = op & INDIRECT_OPCODE_FLAG;
  if (isFar && isInd) {
    // Behavior undefined when both flags are set, because the upper range
    // overlaps with USR return codes, and the 0xC0 case would reduce to 0x00 OK.
    // So we explicitly reject both flags to avoid ambiguity. This allows us to
    // use literal 0xC0 as the control escape code CONTROL_EXCODE.
    vmctx.exitCode = Moic::INVALID_OPFLAG;
    return false;
  }
  const uint8_t directOp = op & ~(INDIRECT_OPCODE_FLAG | FARPTR_OPCODE_FLAG);
  const uint8_t dataLen = getOpCodeDataLength(directOp, isCtl);
  if (dataLen == 0) {
    if (isCtl) {
      vmctx.exitCode = Moic::UNKNOWN_CTLOP;
    } else {
      vmctx.exitCode = Moic::UNKNOWN_COMMAND;
    }
    return false;
  }
  if (dataLen > Moic::SCRIPT_WRITEBUF_SIZE) {
    vmctx.exitCode = Moic::OTHER_ERROR;
    return false;
  }
  const uint8_t operandLen = isFar ? 2 : (isInd ? 1 : dataLen);
  const uint8_t totalCmdLen = 1 + operandLen;
  if (((uint16_t)idx) + totalCmdLen >= Moic::SCRIPT_PAGE_SIZE) {
    vmctx.exitCode = Moic::OVERFLOW;
    return false;
  }
  // Consume from buffer (Advance BEFORE execution)
  vmctx.idx += totalCmdLen;
  uint8_t ptrOffset, farPage;
  if (isFar) {
    farPage = script[idx + 1];
    ptrOffset = script[idx + 2];
    if (farPage >= Moic::NUM_SCRIPT_PAGES || ((uint16_t)ptrOffset) + dataLen >= Moic::SCRIPT_PAGE_SIZE) {
      vmctx.exitCode = Moic::OVERFLOW;
      return false;
    }
  } else if (isInd) {
    ptrOffset = script[idx + 1];
    if (((uint16_t)ptrOffset) + dataLen > 0x1B) {
      vmctx.exitCode = Moic::UNINVITED_POINTER;
      return false;
    }
  }
  for (uint8_t i = 0; i < dataLen; i++) {
    uint8_t incoming;
    if (isFar) {
      incoming = vmctx.scripts[farPage][ptrOffset + i];
    } else if (isInd) {
      incoming = ((uint8_t*)mm.mregs)[ptrOffset + i];
    } else {
      incoming = script[idx + i + 1];
    }
    vmctx.writeBuf[i] = incoming;
  }
  if (isCtl) {
    vmctx.count = 0;
    const uint8_t code = processControl(mm, op);
    if (code != Moic::OK) {
      vmctx.exitCode = code;
      return false;
    }
  } else {
    vmctx.count = dataLen;
    vmctx.offset = directOp;
  }
  return true;
}

}
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
  return pushStack(mm, page, sIdx);
}

uint8_t condCall(Moic::ManagedMotor& mm, volatile uint8_t* cmdBuf) {
  const uint8_t func = cmdBuf[0];
  const uint8_t rhs = cmdBuf[1];
  const int8_t result = condition(mm, func, rhs);
  if (result < 0) {
    return Moic::INVALID_FUNID;
  }
  if (result > 0) {
    return call(mm, &cmdBuf[2]);
  }
  return Moic::OK;
}

uint8_t condJump(Moic::ManagedMotor& mm, volatile uint8_t* cmdBuf) {
  const uint8_t func = cmdBuf[0];
  const uint8_t rhs = cmdBuf[1];
  const int8_t result = condition(mm, func, rhs);
  if (result < 0) {
    return Moic::INVALID_FUNID;
  }
  if (result > 0) {
    return jump(mm, &cmdBuf[2]);
  }
  return Moic::OK;
}

uint8_t processControl(Moic::ManagedMotor& mm, const uint8_t ctlop) {
  switch (ctlop) {
    case Moic::CALL:
      return call(mm, mm.vmctx->writeBuf);
    case Moic::COND_CALL:
      return condCall(mm, mm.vmctx->writeBuf);
    case Moic::COND_JUMP:
      return condJump(mm, mm.vmctx->writeBuf);
    case Moic::JUMP:
      return jump(mm, mm.vmctx->writeBuf);
    default:
      return Moic::UNKNOWN_CTLOP;
  }
}

void popStack(Moic::ManagedMotor& mm, const uint8_t code) {
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

uint8_t pushStack(Moic::ManagedMotor& mm, uint8_t page, const uint8_t sIdx) {
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

int8_t condition(Moic::ManagedMotor& mm, const uint8_t func, const uint8_t rhs) {
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
uint8_t getOpCodeDataLength(const uint8_t offset, const bool isCtl) {
  if (isCtl) {
    switch (offset) {
      case Moic::COND_CALL:
      case Moic::COND_JUMP:
        return 4;
      case Moic::CALL:
      case Moic::JUMP:
        return 2;
      default:
        return 0;
    }
  } else {
    switch (offset) {
      case offsetof(Moic::MotorInterface, currentPosition):
      case offsetof(Moic::MotorInterface, maxSpeed):
      case offsetof(Moic::MotorInterface, acceleration):
      case offsetof(Moic::MotorInterface, cmdMove):
      case offsetof(Moic::MotorInterface, cmdMoveRev):
      case offsetof(Moic::MotorInterface, cmdMoveTo):
      case offsetof(Moic::MotorInterface, cmdDelay):
        return 4;
      case offsetof(Moic::MotorInterface, sleepTimeoutMs):
        return 2;
      case offsetof(Moic::MotorInterface, settingsFlags):
      case offsetof(Moic::MotorInterface, enableDelayMs):
      case offsetof(Moic::MotorInterface, cmdStop):
        return 1;
      default:
        return 0;
    }
  }
}
