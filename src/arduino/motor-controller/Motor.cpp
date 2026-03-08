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

uint8_t _lastid = 0;

const uint16_t LIMIT_SWITCHES_MASK = (1 << Motor::BitIsLimitCw) | (1 << Motor::BitIsLimitAcw);
const uint16_t HOME_END_PASS_MASK = (1 << Motor::BitIsHoming) | (1 << Motor::BitIsEnding) | (1 << Motor::BitIsBacking) | (1 << Motor::BitIsForwarding);
const unsigned long DEG_NULL = 1000.00;

Motor::Motor(
  Motor::Pins pins,
  Motor::Parameters parameters)
  : _params(parameters),
    _settings({}),
    _state({}),
    id(++_lastid),
    pins(pins),
    parameters(_params),
    settings(_settings),
    state(_state),
    _stepper(AccelStepper(AccelStepper::FULL2WIRE, pins.step, pins.dir))
{
  _params.defaultSpeed = min(_params.defaultSpeed, _params.absMaxSpeed);
  _settings.homingSpeed = min(_settings.homingSpeed, _params.absMaxSpeed);
  _settings.maxSpeed = _params.defaultSpeed;
  _settings.acceleration = _params.maxAcceleration;
  _stepper.setMaxSpeed(_settings.maxSpeed);
  _stepper.setAcceleration(_settings.acceleration);
}

void Motor::begin() {
  pinMode(pins.step, OUTPUT);
  pinMode(pins.dir, OUTPUT);
  pinMode(pins.enable, OUTPUT);
  pinMode(pins.limit_cw, LIMIT_TRIPPED == HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);
  pinMode(pins.limit_acw, LIMIT_TRIPPED == HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);
  digitalWrite(pins.enable, 1 - MOTOR_ON);
  _state.lastActionTime = millis();
}

boolean Motor::runIfNeeded() {
  if (_stepper.distanceToGo() != 0) {
    _runActive();
    return true;
  }
  if ((state.flags >> BitIsMoving) & 1) {
    _updateIdle();
  } else if ((state.flags >> BitIsActive) & 1) {
    _checkSleep();
  }
  return false;
}

boolean Motor::canMove(const long direction) {
  // the direction is just a positive/negative direction reference.
  if (!((settings.flags >> BitLimitsEnabled) & 1)) {
    return true;
  }
  return !((state.flags >> (direction > 0 ? BitIsLimitCw : BitIsLimitAcw)) & 1);
}

boolean Motor::move(const long howMuch) {
  if (canMove(howMuch)) {
    _stepper.move(howMuch);
    _enable();
    return true;
  }
  return false;
}

boolean Motor::moveHome() {
  if (!((settings.flags >> BitLimitsEnabled) & 1) || (state.flags & HOME_END_PASS_MASK)) {
    // D_println("ignore moveHome");
    return false;
  }
  overrideMaxSpeed(settings.homingSpeed);
  overrideAcceleration(parameters.maxAcceleration);
  if (isHome()) {
    // move back just a little
    // D_println("moving back");
    _state.flags |= 1 << BitIsBacking;
    move(_degtos(1.5));
    // homing will recommence after backing is complete
  } else {
    // D_println("moving home");
    _state.flags |= 1 << BitIsHoming;
    move(-_getOverLimitStepsToMove());
  }
  return true;
}

boolean Motor::moveEnd() {
  if (!((settings.flags >> BitLimitsEnabled) & 1) || (state.flags & HOME_END_PASS_MASK)) {
    // D_println("ignore moveEnd");
    return false;
  }
  overrideMaxSpeed(settings.homingSpeed);
  overrideAcceleration(parameters.maxAcceleration);
  if (isEnd()) {
    // move forward just a little
    // D_println("moving forward");
    _state.flags |= 1 << BitIsForwarding;
    move(-_degtos(1.5));
    // ending will recommence after forwarding is complete
  } else {
    // D_println("moving end");
    _state.flags |= 1 << BitIsEnding;
    move(_getOverLimitStepsToMove());
  }
  return true;
}

boolean Motor::stop() {
  return _stop(true);
}

boolean Motor::isHome() {
  return ((settings.flags >> BitLimitsEnabled) & 1) && ((state.flags >> BitIsLimitAcw) & 1);
}

boolean Motor::isEnd() {
  return ((settings.flags >> BitLimitsEnabled )& 1) && ((state.flags >> BitIsLimitCw) & 1);
}

void Motor::setMaxSpeed(unsigned long value) {
  _settings.maxSpeed = min(value, parameters.absMaxSpeed);
  _stepper.setMaxSpeed(_settings.maxSpeed);
}

void Motor::setHomingSpeed(unsigned long value) {
  _settings.homingSpeed = min(value, parameters.absMaxSpeed);
}

void Motor::setAcceleration(unsigned long value) {
  _settings.acceleration = min(value, parameters.maxAcceleration);
  _stepper.setAcceleration(settings.acceleration);
}

void Motor::overrideMaxSpeed(unsigned long value) {
  value = min(value, parameters.absMaxSpeed);
  if (settings.maxSpeed != value) {
    if (!state.oldMaxSpeed) {
      _state.oldMaxSpeed = settings.maxSpeed;
    }
    setMaxSpeed(value);
  }
}

void Motor::overrideAcceleration(unsigned long value) {
  value = min(value, parameters.maxAcceleration);
  if (settings.acceleration != value) {
    if (!state.oldAcceleration) {
      _state.oldAcceleration = settings.acceleration;
    }
    setAcceleration(value);
  }
}

void Motor::restoreMaxSpeed() {
  if (state.oldMaxSpeed) {
    setMaxSpeed(state.oldMaxSpeed);
    _state.oldMaxSpeed = 0;
  }
}

void Motor::restoreAcceleration() {
  if (state.oldAcceleration) {
    setAcceleration(state.oldAcceleration);
    _state.oldAcceleration = 0;
  }
}

void Motor::setLimitSwitchEnablement(const boolean value) {
  auto& flags = _settings.flags;
  if ((boolean)((flags >> BitLimitsEnabled) & 1) != value) {
    flags = (flags & ~(1 << BitLimitsEnabled)) | (value << BitLimitsEnabled);
  }
}

void Motor::readLimitSwitches() {
  _state.flags = (
    (_state.flags & ~LIMIT_SWITCHES_MASK) |
    ((digitalRead(pins.limit_cw) == LIMIT_TRIPPED) << BitIsLimitCw) |
    ((digitalRead(pins.limit_acw) == LIMIT_TRIPPED) << BitIsLimitAcw)
  );
}

void Motor::_runActive() {
  readLimitSwitches();
  if (millis() > state.enabledAt + ENABLE_DELAY_MS) {
    // this will move at most one step
    _stepper.run();
    if (!canMove(_stepper.distanceToGo())) {
      _stop(false);
    }
  }
  _state.lastActionTime = millis();
  auto& flags = _state.flags;
  flags |= 1 << BitIsMoving;
  if ((flags >> BitHasHomed) & 1) {
    _state.pos = _stepper.currentPosition();
    _state.targetPos = _stepper.targetPosition();
  }
}

void Motor::_updateIdle() {
  restoreAcceleration();
  restoreMaxSpeed();
  auto& flags = _state.flags;
  if ((flags >> BitIsBacking) & 1) {
    // we have finished backing for home
    // D_println("finished backing");
    flags ^= 1 << BitIsBacking;
    moveHome();
    return;
  }
  if ((flags >> BitIsForwarding) & 1) {
    // we have finished forwarding for end
    // D_println("finished forwarding");
    flags ^= 1 << BitIsForwarding;
    moveEnd();
    return;
  }
  flags &= ~((1 << BitIsHoming) | (1 << BitIsEnding) | (1 << BitIsMoving));
  if ((flags >> BitIsStopping) & 1) {
    // we have finished stopping
    // D_println("finished stopping");
    flags ^= 1 << BitIsStopping;
    if ((flags >> BitIsForceStop) & 1) {
      flags ^= 1 << BitIsForceStop;
    } else {
      // we have reached a limit switch, see if we are home
      if (isHome()) {
        flags |= 1 << BitHasHomed;
        // _state.hasHomed = true;
        _stepper.setCurrentPosition(0);
        _stepper.setMaxSpeed(settings.maxSpeed);
        _state.pos = _stepper.currentPosition();
      } else if (isEnd() && ((flags >> BitHasHomed) & 1)) {
        // store the known max position
        _state.posMax = _stepper.currentPosition();
      }
    }
  }
  _state.targetPos = _state.pos;
  _checkSleep();
}

boolean Motor::_stop(const boolean force) {
  auto& flags = _state.flags;
  if (((flags >> BitIsStopping) & 1) || !((flags >> BitIsMoving) & 1)) {
    // skip duplicate or unnecessary action
    // D_println("ignoring stop");
    return false;
  }
  flags = (flags & ~(1 << BitIsForceStop)) | (force << BitIsForceStop) | (1 << BitIsStopping);
  overrideAcceleration(parameters.maxAcceleration);
  _stepper.stop();
  return true;
}

void Motor::_enable() {
  auto& flags = _state.flags;
  if (!((flags >> BitIsActive) & 1)) {
    digitalWrite(pins.enable, MOTOR_ON);
    flags |= 1 << BitIsActive;
    _state.enabledAt = millis();
  }
  _state.lastActionTime = millis();
}

void Motor::_disable() {
  auto& flags = _state.flags;
  if ((flags >> BitIsActive) & 1) {
    digitalWrite(pins.enable, 1 - MOTOR_ON);
    flags &= ~(1 << BitIsActive);
    _state.enabledAt = 0;
  }
}

void Motor::_checkSleep() {
  auto& flags = state.flags;
  if (((flags >> BitIsActive )& 1) && !((flags >> BitIsMoving) & 1) && millis() - state.lastActionTime > MOTOR_SLEEP_TIMEOUT_MS) {
    _disable();
  }
}


long Motor::_degtos(const float howMuch) {
  return (howMuch * parameters.millistepsPerDegree) / 1000;
}

unsigned long Motor::_getOverLimitStepsToMove() {
  float degreesToMove = parameters.maxDegrees;
  const float mposDegrees = ((state.flags >> BitHasHomed) & 1) ? (state.pos * 1000) / parameters.millistepsPerDegree : DEG_NULL;
  // if we know position, don't way overshoot
  if (mposDegrees != DEG_NULL && mposDegrees > 0) {
    degreesToMove = mposDegrees + 10;
  }
  return _degtos(degreesToMove);
}
