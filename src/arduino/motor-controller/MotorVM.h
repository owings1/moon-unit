#include <sys/_stdint.h>
#ifndef MOTOR_VM_H
#define MOTOR_VM_H

#include <Arduino.h>
#include "MoicProtocol.h"
#include "MotorState.h"

namespace MotorVM {
static const uint8_t INDIRECT_OPCODE_FLAG = 0x40;
uint8_t jump(volatile Moic::MotorBlock& mregs, volatile uint8_t* cmdBuf);
uint8_t call(volatile Moic::MotorBlock& mregs, volatile uint8_t* cmdBuf);
uint8_t condCall(volatile Moic::MotorBlock& mregs, volatile uint8_t* cmdBuf);
uint8_t condJump(volatile Moic::MotorBlock& mregs, volatile uint8_t* cmdBuf);
bool processNext(volatile Moic::MotorBlock& mregs, uint8_t& offset, uint8_t& count, uint8_t& exitCode);
}
uint8_t scriptStackPush(volatile Moic::MotorBlock& mregs, uint8_t page, const uint8_t sIdx);
void scriptStackPop(volatile Moic::MotorBlock& mregs, const uint8_t code);
int8_t scriptCondition(volatile Moic::MotorBlock& mregs, const uint8_t func, const uint8_t rhs);
uint8_t getOpCodeDataLength(const uint8_t offset);
#endif
