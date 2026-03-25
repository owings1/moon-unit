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
#ifndef ENABLE_DELAY_MS
#define ENABLE_DELAY_MS 2
#endif
#ifndef MOTOR_SLEEP_TIMEOUT_MS
#define MOTOR_SLEEP_TIMEOUT_MS 2000
#endif

#define checkbit(flags, bit) ((flags >> bit) & 1)

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
  if (checkbit(stateFlags, BitIsMoving)) {
    _updateIdle();
  } else if (checkbit(stateFlags, BitIsActive)) {
    _checkSleep();
  }
  return false;
}

bool Motor::move(const int32_t value) {
  if (canMove(value)) {
    stepper.move(value);
    _enable();
    return true;
  }
  return false;
}

bool Motor::stop() {
  if (checkbit(stateFlags, BitIsStopping) || !checkbit(stateFlags, BitIsMoving)) {
    // skip duplicate or unnecessary action
    // D_println("ignoring stop");
    return false;
  }
  stateFlags |= 1 << BitIsStopping;
  _accelerationSaved = stepper.acceleration();
  stepper.setAcceleration(STOP_DECELERATION);
  stepper.stop();
  return true;
}

bool Motor::busy() {
  return checkbit(stateFlags, BitIsMoving);
}

bool Motor::scriptActive() {
  return checkbit(stateFlags, BitIsScriptActive);
}
void Motor::setCurrentPosition(const int32_t value) {
  stepper.setCurrentPosition(value);
  stateFlags |= 1 << BitIsManualPos;
}

void Motor::setSettingsFlags(const uint8_t value) {
  settingsFlags = value & SETTINGS_FLAGS_MASK;
}

void Motor::setScriptActive(const bool value) {
  if (value) {
    stateFlags |= 1 << BitIsScriptActive;
  } else {
    stateFlags &= ~(1 << BitIsScriptActive);
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
  stateFlags = (stateFlags & ~LIMIT_SWITCHES_MASK) | newFlags;
}

uint8_t Motor::getStateFlags() {
  return stateFlags;
}

bool Motor::canMove(const int32_t direction) {
  // the direction is just a positive/negative direction reference.
  if (!checkbit(settingsFlags, BitLimitsEnabled)) {
    return true;
  }
  return !((stateFlags >> (direction > 0 ? BitIsLimitCw : BitIsLimitAcw)) & 1);
}

void Motor::_runActive() {
  readLimitSwitches();
  if (millis() > enabledAt + ENABLE_DELAY_MS) {
    // this will move at most one step
    stepper.run();
    if (!canMove(stepper.distanceToGo())) {
      stop();
    }
  }
  lastActionTime = millis();
  stateFlags |= 1 << BitIsMoving;
}

void Motor::_updateIdle() {
  readLimitSwitches();
  if (checkbit(stateFlags, BitIsStopping) && stepper.acceleration() != _accelerationSaved) {
    stepper.setAcceleration(_accelerationSaved);
  }
  stateFlags &= ~((1 << BitIsMoving) | (1 << BitIsStopping));
  _checkSleep();
}

void Motor::_enable() {
  if (!checkbit(stateFlags, BitIsActive)) {
    if (pins.enable != NOPIN) {
      digitalWrite(pins.enable, MOTOR_ON);
    }
    // stepper.enableOutputs();
    stateFlags |= 1 << BitIsActive;
    enabledAt = millis();
  }
  lastActionTime = millis();
}

void Motor::_disable() {
  if (checkbit(stateFlags, BitIsActive)) {
    if (pins.enable != NOPIN) {
      digitalWrite(pins.enable, 1 - MOTOR_ON);
    }
    // stepper.disableOutputs();
    stateFlags &= ~(1 << BitIsActive);
    enabledAt = 0;
  }
}

void Motor::_checkSleep() {
  if (checkbit(stateFlags, BitIsActive) && !checkbit(stateFlags, BitIsMoving) && millis() - lastActionTime > MOTOR_SLEEP_TIMEOUT_MS) {
    _disable();
  }
}
