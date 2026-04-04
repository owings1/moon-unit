#include <sys/_stdint.h>
#ifndef MOTOR_VM_H
#define MOTOR_VM_H

#include <Arduino.h>
#include "MoicProtocol.h"
#include "MotorState.h"
#include "MotorManager.h"

namespace MotorVM {

bool tick(Moic::ManagedMotor& mm);
}
uint8_t resolveOperands(Moic::ManagedMotor& mm, const uint8_t op, const uint8_t dataLen, const uint8_t pfxLen, const uint8_t idx);
uint8_t setVar(Moic::ManagedMotor& mm, volatile uint8_t* cmdBuf);
uint8_t jump(Moic::ManagedMotor& mm, volatile uint8_t* cmdBuf);
uint8_t call(Moic::ManagedMotor& mm, volatile uint8_t* cmdBuf);
uint8_t condCall(Moic::ManagedMotor& mm, volatile uint8_t* cmdBuf);
uint8_t condJump(Moic::ManagedMotor& mm, volatile uint8_t* cmdBuf);
int8_t condition(Moic::ManagedMotor& mm, const uint8_t func, const uint8_t rhs);
uint8_t pushStack(Moic::ManagedMotor& mm, uint8_t page, const uint8_t sIdx);
void popStack(Moic::ManagedMotor& mm, const uint8_t code);
uint8_t getOpCodeDataLength(const uint8_t offset, const bool isCtl);
uint8_t processControl(Moic::ManagedMotor& mm, const uint8_t ctlop);
uint8_t getCtlPfxLen(const uint8_t ctlop);
#endif
