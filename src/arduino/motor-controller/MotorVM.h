#include <sys/_stdint.h>
#ifndef MOTOR_VM_H
#define MOTOR_VM_H

#include <Arduino.h>
#include "MoicProtocol.h"
#include "MotorState.h"

namespace MotorVM {
static const uint8_t INDIRECT_OPCODE_FLAG = 0x40;
static const uint8_t FARPTR_OPCODE_FLAG = 0x80;
uint8_t jump(volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, volatile uint8_t* cmdBuf);
uint8_t call(volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, volatile uint8_t* cmdBuf);
uint8_t condCall(volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, volatile uint8_t* cmdBuf);
uint8_t condJump(volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, volatile uint8_t* cmdBuf);
bool processNext(volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, uint8_t& offset, uint8_t& count, uint8_t& exitCode);
}
int8_t scriptCondition(volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, const uint8_t func, const uint8_t rhs);
uint8_t scriptStackPush(volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, uint8_t page, const uint8_t sIdx);
void scriptStackPop(volatile Moic::MotorInterface& mregs, volatile Moic::MotorContext& ctx, const uint8_t code);
uint8_t getOpCodeDataLength(const uint8_t offset);
#endif
