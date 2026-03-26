#include "Arduino.h"
#include <sys/_stdint.h>
#include <stdint.h>
#include "Motor.h"
#include "MotorWrapper.h"
#include <AccelStepper.h>

#ifndef LIMIT_TRIPPED
#define LIMIT_TRIPPED HIGH
#endif
#ifndef MOTOR_ON
#define MOTOR_ON LOW
#endif

#ifndef checkbit
#define checkbit(flags, bit) ((flags >> bit) & 1)
#endif
#define checkstate(bit) checkbit(_stateFlags, bit)
#define checksetting(bit) checkbit(_settingsFlags, bit)

Motor::Motor(AccelStepper& stepper, Motor::Pins pins)
  : pins(pins),
    MotorWrapper(stepper) {
}

void Motor::begin() {
  if (pins.enable != NOPIN) {
    pinMode(pins.enable, OUTPUT);
    digitalWrite(pins.enable, 1 - MOTOR_ON);
  }
  if (pins.limit_cw != NOPIN) {
    pinMode(pins.limit_cw, LIMIT_TRIPPED == HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);
  }
  if (pins.limit_acw != NOPIN) {
    pinMode(pins.limit_acw, LIMIT_TRIPPED == HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);
  }
  lastActionTime = millis();
}

bool Motor::run() {
  if (stepper.distanceToGo() != 0) {
    _runActive();
    return true;
  }
  if (checkstate(BitIsMoving)) {
    _updateIdle();
  } else if (checkstate(BitIsActive)) {
    _checkSleep();
  }
  return false;
}

bool Motor::move(const int32_t value) {
  if (canMove(value)) {
    stepper.move(value);
    if (value != 0) {
      _enable();
    }
    return true;
  }
  return false;
}

bool Motor::stop() {
  if (checkstate(BitIsStopping) || !checkstate(BitIsMoving)) {
    // skip duplicate or unnecessary action
    return false;
  }
  _stateFlags |= 1 << BitIsStopping;
  _accelerationSaved = stepper.acceleration();
  stepper.setAcceleration(STOP_DECELERATION);
  stepper.stop();
  return true;
}

bool Motor::busy() {
  return stepper.isRunning() || checkstate(BitIsMoving);
}

void Motor::setCurrentPosition(const int32_t value) {
  stepper.setCurrentPosition(value);
  _stateFlags |= 1 << BitIsManualPos;
}

void Motor::setSettingsFlags(const uint8_t value) {
  _settingsFlags = value & SETTINGS_FLAGS_MASK;
}

void Motor::setScriptActive(const bool value) {
  if (value) {
    _stateFlags |= 1 << BitIsScriptActive;
  } else {
    _stateFlags &= ~(1 << BitIsScriptActive);
  }
}
void Motor::setDelayActive(const bool value) {
  if (value) {
    _stateFlags |= 1 << BitIsDelayActive;
  } else {
    _stateFlags &= ~(1 << BitIsDelayActive);
  }
}

void Motor::readLimitSwitches() {
  uint8_t newFlags = 0;
  if (pins.limit_cw != NOPIN) {
    newFlags |= (digitalRead(pins.limit_cw) == LIMIT_TRIPPED) << BitIsLimitCw;
  }
  if (pins.limit_acw != NOPIN) {
    newFlags |= (digitalRead(pins.limit_acw) == LIMIT_TRIPPED) << BitIsLimitAcw;
  }
  _stateFlags = (_stateFlags & ~LIMIT_SWITCHES_MASK) | newFlags;
}

bool Motor::canMove(const int32_t direction) {
  // the direction is just a positive/negative direction reference.
  if (!checksetting(BitLimitsEnabled)) {
    return true;
  }
  return !((_stateFlags >> (direction > 0 ? BitIsLimitCw : BitIsLimitAcw)) & 1);
}

void Motor::_runActive() {
  readLimitSwitches();
  if (millis() > enabledAt + _enableDelayMs) {
    // this will move at most one step
    stepper.run();
    if (!canMove(stepper.distanceToGo())) {
      stop();
    }
  }
  lastActionTime = millis();
  _stateFlags |= 1 << BitIsMoving;
}

void Motor::_updateIdle() {
  readLimitSwitches();
  if (checkstate(BitIsStopping) && stepper.acceleration() != _accelerationSaved) {
    stepper.setAcceleration(_accelerationSaved);
  }
  _stateFlags &= ~((1 << BitIsMoving) | (1 << BitIsStopping));
  _checkSleep();
}

void Motor::_enable() {
  if (!checkstate(BitIsActive)) {
    if (pins.enable != NOPIN) {
      digitalWrite(pins.enable, MOTOR_ON);
    } else {
      stepper.enableOutputs();
    }
    _stateFlags |= 1 << BitIsActive;
    enabledAt = millis();
  }
  lastActionTime = millis();
}

void Motor::_disable() {
  if (checkstate(BitIsActive)) {
    if (pins.enable != NOPIN) {
      digitalWrite(pins.enable, 1 - MOTOR_ON);
    } else {
      stepper.disableOutputs();
    }
    _stateFlags &= ~(1 << BitIsActive);
    enabledAt = 0;
  }
}

void Motor::_checkSleep() {
  if (checksetting(BitSleepEnabled) && checkstate(BitIsActive) && !checkstate(BitIsMoving) && millis() - lastActionTime > _sleepTimeoutMs) {
    _disable();
  }
}
