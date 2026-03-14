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

Motor::Motor(Motor::Pins pins)
  : _settings({}),
    _state({}),
    id(++_lastid),
    pins(pins),
    settings(_settings),
    state(_state),
    _stepper(AccelStepper(AccelStepper::FULL2WIRE, pins.step, pins.dir))
{
  _stepper.setMaxSpeed(settings.values.maxSpeed);
  _stepper.setAcceleration(settings.values.acceleration);
}

void Motor::begin() {
  pinMode(pins.step, OUTPUT);
  pinMode(pins.dir, OUTPUT);
  pinMode(pins.enable, OUTPUT);
  pinMode(pins.limit_cw, LIMIT_TRIPPED == HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);
  pinMode(pins.limit_acw, LIMIT_TRIPPED == HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);
  digitalWrite(pins.enable, 1 - MOTOR_ON);
  _state.values.lastActionTime = millis();
}

boolean Motor::runIfNeeded() {
  if (_stepper.distanceToGo() != 0) {
    _runActive();
    return true;
  }
  if (checkbit(state.values.flags, BitIsMoving)) {
    _updateIdle();
  } else if (checkbit(state.values.flags, BitIsActive)) {
    _checkSleep();
  }
  return false;
}

boolean Motor::canMove(const int32_t direction) {
  // the direction is just a positive/negative direction reference.
  if (!checkbit(settings.values.flags, BitLimitsEnabled)) {
    return true;
  }
  return !((state.values.flags >> (direction > 0 ? BitIsLimitCw : BitIsLimitAcw)) & 1);
}

boolean Motor::move(const int32_t howMuch) {
  if (canMove(howMuch)) {
    _stepper.move(howMuch);
    _enable();
    return true;
  }
  return false;
}

boolean Motor::stop() {
  if (checkbit(state.values.flags, BitIsStopping) || !checkbit(state.values.flags, BitIsMoving)) {
    // skip duplicate or unnecessary action
    // D_println("ignoring stop");
    return false;
  }
  _state.values.flags |= 1 << BitIsStopping;
  _stepper.setAcceleration(STOP_DECELERATION);
  _stepper.stop();
  return true;
}

void Motor::setPosition(const int32_t value) {
  _stepper.setCurrentPosition(value);
  _state.values.pos = _stepper.currentPosition();
  _state.values.targetPos = _stepper.targetPosition();
  _state.values.speed = fabs(_stepper.speed());
  _state.values.flags |= 1 << BitIsManualPos;
}

void Motor::setMaxSpeed(const uint16_t value) {
  if (_settings.values.maxSpeed == value) {
    return;
  }
  _settings.values.maxSpeed = value;
  _stepper.setMaxSpeed(settings.values.maxSpeed);
}

void Motor::setAcceleration(const uint16_t value) {
  if (_settings.values.acceleration == value) {
    return;
  }
  _settings.values.acceleration = value;
  _stepper.setAcceleration(settings.values.acceleration);
}

void Motor::setSettingsFlags(const uint8_t value) {
  _settings.values.flags = value & SETTINGS_FLAGS_MASK;
}

void Motor::readLimitSwitches() {
  _state.values.flags = (
    (state.values.flags & ~LIMIT_SWITCHES_MASK) |
    ((digitalRead(pins.limit_cw) == LIMIT_TRIPPED) << BitIsLimitCw) |
    ((digitalRead(pins.limit_acw) == LIMIT_TRIPPED) << BitIsLimitAcw)
  );
}

void Motor::_runActive() {
  readLimitSwitches();
  if (millis() > state.values.enabledAt + ENABLE_DELAY_MS) {
    // this will move at most one step
    _stepper.run();
    if (!canMove(_stepper.distanceToGo())) {
      stop();
    }
  }
  _state.values.lastActionTime = millis();
  _state.values.flags |= 1 << BitIsMoving;
  _state.values.pos = _stepper.currentPosition();
  _state.values.targetPos = _stepper.targetPosition();
  _state.values.speed = fabs(_stepper.speed());
}

void Motor::_updateIdle() {
  readLimitSwitches();
  _stepper.setAcceleration(settings.values.acceleration);
  _state.values.flags &= ~((1 << BitIsMoving) | (1 << BitIsStopping));
  _state.values.pos = _stepper.currentPosition();
  _state.values.targetPos = _stepper.targetPosition();
  _state.values.speed = fabs(_stepper.speed());
  _checkSleep();
}

void Motor::_enable() {
  if (!checkbit(state.values.flags, BitIsActive)) {
    digitalWrite(pins.enable, MOTOR_ON);
    _state.values.flags |= 1 << BitIsActive;
    _state.values.enabledAt = millis();
  }
  _state.values.lastActionTime = millis();
}

void Motor::_disable() {
  if (checkbit(state.values.flags, BitIsActive)) {
    digitalWrite(pins.enable, 1 - MOTOR_ON);
    _state.values.flags &= ~(1 << BitIsActive);
    _state.values.enabledAt = 0;
  }
}

void Motor::_checkSleep() {
  if (checkbit(state.values.flags, BitIsActive) && !checkbit(state.values.flags, BitIsMoving) && millis() - state.values.lastActionTime > MOTOR_SLEEP_TIMEOUT_MS) {
    _disable();
  }
}

