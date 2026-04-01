#include <sys/_stdint.h>
#ifndef MOTOR_VM_H
#define MOTOR_VM_H

#include <Arduino.h>
#include "MoicProtocol.h"
#include "MotorState.h"
#include "MotorManager.h"

namespace MotorVM {
static const uint8_t INDIRECT_OPCODE_FLAG = 0x40;
static const uint8_t FARPTR_OPCODE_FLAG = 0x80;

uint8_t jump(Moic::ManagedMotor& mm, volatile uint8_t* cmdBuf);
uint8_t call(Moic::ManagedMotor& mm, volatile uint8_t* cmdBuf);
uint8_t condCall(Moic::ManagedMotor& mm, volatile uint8_t* cmdBuf);
uint8_t condJump(Moic::ManagedMotor& mm, volatile uint8_t* cmdBuf);
bool processNext(Moic::ManagedMotor& mm);
}
int8_t scriptCondition(Moic::ManagedMotor& mm, const uint8_t func, const uint8_t rhs);
uint8_t scriptStackPush(Moic::ManagedMotor& mm, uint8_t page, const uint8_t sIdx);
void scriptStackPop(Moic::ManagedMotor& mm, const uint8_t code);
uint8_t getOpCodeDataLength(const uint8_t offset);
#endif
