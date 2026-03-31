#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H
#include <stddef.h>

#include <Arduino.h>
#include "MoicProtocol.h"
#include "IMotor.h"
#include "MotorState.h"
#include "MotorActions.h"

namespace Moic {

class ManagedMotor {
public:
  ManagedMotor(IMotor* m, volatile MotorInterface& mregs, volatile MotorContext& ctx);
  uint8_t write(const uint8_t offset, const uint8_t incoming, const bool enforceBusy, const bool enforceScriptLock);
  void tick();
  IMotor* m;
  volatile MotorInterface* mregs;
  volatile MotorContext* ctx;
  const bool& scriptActive;
private:
  bool _scriptActive = false;
};

};

#endif
