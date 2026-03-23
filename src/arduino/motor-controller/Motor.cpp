#include <stdint.h>
#include "Motor.h"
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

uint8_t _lastid = 0;

const uint8_t LIMIT_SWITCHES_MASK = (1 << Motor::BitIsLimitCw) | (1 << Motor::BitIsLimitAcw);
const uint8_t SETTINGS_FLAGS_MASK = (1 << Motor::BitLimitsEnabled);

Motor::Motor(AccelStepper stepper, Motor::Pins pins)
  : 
    id(++_lastid),
    pins(pins),
    stateFlags(_stateFlags),
    settingsFlags(_settingsFlags),
    stepper(stepper)
{
  setMaxSpeed(2000);
  setAcceleration(50000);
}

void Motor::begin() {
  if (pins.enable) {
    pinMode(pins.enable, OUTPUT);
    digitalWrite(pins.enable, 1 - MOTOR_ON);
  }
  if (pins.limit_cw) {
    pinMode(pins.limit_cw, LIMIT_TRIPPED == HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);
  }
  if (pins.limit_acw) {
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

bool Motor::move(const int32_t howMuch) {
  if (canMove(howMuch)) {
    stepper.move(howMuch);
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
  _stateFlags |= 1 << BitIsStopping;
  _accelerationSaved = stepper.acceleration();
  stepper.setAcceleration(STOP_DECELERATION);
  stepper.stop();
  return true;
}

void Motor::setCurrentPosition(const int32_t value) {
  stepper.setCurrentPosition(value);
  _stateFlags |= 1 << BitIsManualPos;
}

void Motor::setMaxSpeed(const uint16_t value) {
  stepper.setMaxSpeed(value);
}

void Motor::setAcceleration(const uint16_t value) {
  stepper.setAcceleration(value);
}

void Motor::setSettingsFlags(const uint8_t value) {
  _settingsFlags = value & SETTINGS_FLAGS_MASK;
}

void Motor::readLimitSwitches() {
  _stateFlags = (
    (stateFlags & ~LIMIT_SWITCHES_MASK) |
    ((pins.limit_cw  ? (digitalRead(pins.limit_cw) == LIMIT_TRIPPED) : 0) << BitIsLimitCw) |
    ((pins.limit_acw ? (digitalRead(pins.limit_acw) == LIMIT_TRIPPED) : 0) << BitIsLimitAcw)
  );
}

int32_t Motor::currentPosition() {
  return stepper.currentPosition();
}
int32_t Motor::targetPosition() {
  return stepper.targetPosition();
}
uint16_t Motor::maxSpeed() {
  return stepper.maxSpeed();
}
uint16_t Motor::acceleration() {
  return stepper.acceleration();
}
uint16_t Motor::speed() {
  return fabs(stepper.speed());
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
  _stateFlags |= 1 << BitIsMoving;
}

void Motor::_updateIdle() {
  readLimitSwitches();
  if (checkbit(stateFlags, BitIsStopping) && stepper.acceleration() != _accelerationSaved) {
    stepper.setAcceleration(_accelerationSaved);
  }
  _stateFlags &= ~((1 << BitIsMoving) | (1 << BitIsStopping));
  _checkSleep();
}

void Motor::_enable() {
  if (!checkbit(stateFlags, BitIsActive)) {
    if (pins.enable) {
      digitalWrite(pins.enable, MOTOR_ON);
    }
    _stateFlags |= 1 << BitIsActive;
    enabledAt = millis();
  }
  lastActionTime = millis();
}

void Motor::_disable() {
  if (checkbit(stateFlags, BitIsActive)) {
    if (pins.enable) {
      digitalWrite(pins.enable, 1 - MOTOR_ON);
    }
    _stateFlags &= ~(1 << BitIsActive);
    enabledAt = 0;
  }
}

void Motor::_checkSleep() {
  if (checkbit(stateFlags, BitIsActive) && !checkbit(stateFlags, BitIsMoving) && millis() - lastActionTime > MOTOR_SLEEP_TIMEOUT_MS) {
    _disable();
  }
}

