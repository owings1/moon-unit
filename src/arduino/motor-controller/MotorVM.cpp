#include <sys/_stdint.h>
#include "MotorVM.h"

namespace MotorVM {

bool tick(Moic::ManagedMotor& mm) {
  auto& vmctx = *(mm.vmctx);
  if (vmctx.page >= Moic::NUM_SCRIPT_PAGES || vmctx.idx >= Moic::SCRIPT_PAGE_SIZE) {
    vmctx.exitCode = Moic::OVERFLOW;
    return false;
  }
  uint8_t idx = vmctx.idx;
  uint8_t op = vmctx.scripts[vmctx.page][idx];
  // 1. Terminal / Return Logic
  if (op == Moic::OK || op >= Moic::USR1) {
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
  // 2. Control Escape Handling
  const bool isCtl = (op == CONTROL_EXCODE);
  if (isCtl) {
    if (idx + 1 >= Moic::SCRIPT_PAGE_SIZE) {
      vmctx.exitCode = Moic::OVERFLOW;
      return false;
    }
    // Skip 0xC0 and get the actual control opcode
    idx++;
    op = vmctx.scripts[vmctx.page][idx];
  }
  // 3. Decoding & Flag Validation
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
  // Safety: Only SET_REG is allowed to use Pointer Modes.
  // CALL, JUMP, etc. MUST be literals.
  if (isCtl && (isFar || isInd) && directOp != Moic::SET_REG) {
    vmctx.exitCode = Moic::INVALID_OPFLAG;
    return false;
  }
  const uint8_t dataLen = getOpCodeDataLength(directOp, isCtl);
  if (dataLen == 0 || dataLen > Moic::SCRIPT_WRITEBUF_SIZE) {
    vmctx.exitCode = isCtl ? Moic::UNKNOWN_CTLOP : Moic::UNKNOWN_COMMAND;
    return false;
  }
  // 4. Operand Resolution
  const uint8_t operandSize = isFar ? 2 : (isInd ? 1 : dataLen);
  if ((uint16_t)idx + 1 + operandSize >= Moic::SCRIPT_PAGE_SIZE) {
    vmctx.exitCode = Moic::OVERFLOW;
    return false;
  }
  const uint8_t pfxLen = isCtl ? getCtlPfxLen(directOp) : 0;
  const uint8_t resErr = resolveOperands(mm, op, dataLen, pfxLen, idx);
  if (resErr != Moic::OK) {
    vmctx.exitCode = resErr;
    return false;
  }
  // 5. PC Advancement
  // If we used a control escape, we must skip the escape byte too
  const uint8_t instructionSize = isCtl ? (2 + operandSize) : (1 + operandSize);
  vmctx.idx += instructionSize;
  // 6. Execution
  if (isCtl) {
    vmctx.count = 0;
    const uint8_t ctlErr = processControl(mm, op);
    if (ctlErr != Moic::OK) {
      vmctx.exitCode = ctlErr;
      return false;
    }
  } else {
    vmctx.count = dataLen;
    vmctx.offset = directOp;
  }
  return true;
}

}

uint8_t resolveOperands(Moic::ManagedMotor& mm, const uint8_t op, const uint8_t dataLen, const uint8_t pfxLen, const uint8_t idx) {
  auto& vmctx = *(mm.vmctx);
  volatile uint8_t* script = vmctx.scripts[vmctx.page];
  const bool isFar = op & MotorVM::FARPTR_OPCODE_FLAG;
  const bool isInd = op & MotorVM::INDIRECT_OPCODE_FLAG;
  if (isFar) {
    const uint8_t farPage = script[idx + 1 + pfxLen];
    const uint8_t ptrOffset = script[idx + 2 + pfxLen];
    if (farPage >= Moic::NUM_SCRIPT_PAGES || (uint16_t)ptrOffset + (dataLen - pfxLen) > Moic::SCRIPT_PAGE_SIZE) {
      return Moic::OVERFLOW;
    }
    // We still resolve the FULL dataLen into writeBuf. 
    // For SET_REG, dataLen is 5. We want writeBuf[0] to be RegIdx, 
    // and writeBuf[1..4] to be the resolved data.
    vmctx.writeBuf[0] = script[idx + 1]; // Store the RegIdx
    for (uint8_t i = 0; i < (dataLen - pfxLen); i++) {
      vmctx.writeBuf[i + pfxLen] = vmctx.scripts[farPage][ptrOffset + i];
    }
  } else if (isInd) {
    const uint8_t ptrOffset = script[idx + 1 + pfxLen];
    if ((uint16_t)ptrOffset + (dataLen - pfxLen) > 0x1B) {
      return Moic::UNINVITED_POINTER;
    }
    vmctx.writeBuf[0] = script[idx + 1];
    for (uint8_t i = 0; i < (dataLen - pfxLen); i++) {
      vmctx.writeBuf[i + pfxLen] = ((uint8_t*)mm.mregs)[ptrOffset + i];
    }
  } else {
    for (uint8_t i = 0; i < dataLen; i++) {
      vmctx.writeBuf[i] = script[idx + i + 1];
    }
  }
  return Moic::OK;
}

uint8_t setReg(Moic::ManagedMotor& mm, volatile uint8_t* cmdBuf) {
  auto& vmctx = *(mm.vmctx);
  uint8_t regIdx = cmdBuf[0];
  if (regIdx >= Moic::NUM_SCRIPT_GLOBAL_REGS) {
    return Moic::OVERFLOW;
  }
  // Get the address of the target register and treat it as 4 bytes
  uint8_t* dest = (uint8_t*)&(vmctx.regs[regIdx]);
  for (uint8_t i = 0; i < 4; ++i) {
    dest[i] = cmdBuf[i + 1];
  }
  return Moic::OK;
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
    case Moic::SET_REG:
      return setReg(mm, mm.vmctx->writeBuf);
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
uint8_t getCtlPfxLen(const uint8_t ctlop) {
  switch (ctlop) {
    case Moic::SET_REG:
      return 1;
    default:
      return 0;
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
      case Moic::SET_REG:
        return 5;
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
