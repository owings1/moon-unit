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

Motor::Motor(Motor::Pins pins)
  : _settings({}),
    _state({}),
    id(++_lastid),
    pins(pins),
    settings(_settings),
    state(_state),
    _stepper(AccelStepper(AccelStepper::FULL2WIRE, pins.step, pins.dir))
{
  // setLimitSwitchEnablement(true);
  // setAbsMaxSpeed(5000);
  // setMaxAcceleration(50000);
  // setMaxSpeed(2000);
  // setDefaultSpeed(2000);
  // setHomingSpeed(4000);
  // setAcceleration(50000);
  // setMaxSteps(5000);
  // setBackingSteps(1000);
  _stepper.setMaxSpeed(_settings.values.defaultSpeed);
  _stepper.setAcceleration(_settings.values.acceleration);
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
  if ((state.values.flags >> BitIsMoving) & 1) {
    _updateIdle();
  } else if ((state.values.flags >> BitIsActive) & 1) {
    _checkSleep();
  }
  return false;
}

boolean Motor::canMove(const int32_t direction) {
  // the direction is just a positive/negative direction reference.
  if (!((settings.values.flags >> BitLimitsEnabled) & 1)) {
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

boolean Motor::moveHome() {
  if (!((settings.values.flags >> BitLimitsEnabled) & 1) || (state.values.flags & HOME_END_PASS_MASK)) {
    // D_println("ignore moveHome");
    return false;
  }
  overrideMaxSpeed(settings.values.homingSpeed);
  overrideAcceleration(settings.values.maxAcceleration);
  if (isHome()) {
    // move back just a little
    // D_println("moving back");
    _state.values.flags |= 1 << BitIsBacking;
    move(settings.values.backingSteps);
    // homing will recommence after backing is complete
  } else {
    // D_println("moving home");
    _state.values.flags |= 1 << BitIsHoming;
    move(-settings.values.maxSteps);
  }
  return true;
}

boolean Motor::moveEnd() {
  if (!((settings.values.flags >> BitLimitsEnabled) & 1) || (state.values.flags & HOME_END_PASS_MASK)) {
    // D_println("ignore moveEnd");
    return false;
  }
  overrideMaxSpeed(settings.values.homingSpeed);
  overrideAcceleration(settings.values.maxAcceleration);
  if (isEnd()) {
    // move forward just a little
    // D_println("moving forward");
    _state.values.flags |= 1 << BitIsForwarding;
    move(-settings.values.backingSteps);
    // ending will recommence after forwarding is complete
  } else {
    // D_println("moving end");
    _state.values.flags |= 1 << BitIsEnding;
    move(settings.values.maxSteps);
  }
  return true;
}

boolean Motor::stop() {
  return _stop(true);
}

boolean Motor::isHome() {
  return ((settings.values.flags >> BitLimitsEnabled) & 1) && ((state.values.flags >> BitIsLimitAcw) & 1);
}

boolean Motor::isEnd() {
  return ((settings.values.flags >> BitLimitsEnabled )& 1) && ((state.values.flags >> BitIsLimitCw) & 1);
}

void Motor::setPosition(int32_t value) {
  _stepper.setCurrentPosition(value);
  _state.values.pos = _stepper.currentPosition();
  _state.values.flags |= 1 << BitIsManualPos;
}

void Motor::setDefaultSpeed(uint16_t value) {
  _settings.values.defaultSpeed = value > settings.values.absMaxSpeed ? settings.values.absMaxSpeed : value;
}

void Motor::setHomingSpeed(uint16_t value) {
  _settings.values.homingSpeed = value > settings.values.absMaxSpeed ? settings.values.absMaxSpeed : value;
}

void Motor::setMaxSpeed(uint16_t value) {
  _settings.values.maxSpeed = value > settings.values.absMaxSpeed ? settings.values.absMaxSpeed : value;
  _stepper.setMaxSpeed(_settings.values.maxSpeed);
}

void Motor::setAbsMaxSpeed(uint16_t value) {
  _settings.values.absMaxSpeed = value > SYS_MAX_SPEED ? SYS_MAX_SPEED : value;
}

void Motor::overrideMaxSpeed(uint16_t value) {
  value = value > settings.values.absMaxSpeed ? settings.values.absMaxSpeed : value;
  if (settings.values.maxSpeed != value) {
    if (!state.values.oldMaxSpeed) {
      _state.values.oldMaxSpeed = settings.values.maxSpeed;
    }
    setMaxSpeed(value);
  }
}

void Motor::restoreMaxSpeed() {
  if (state.values.oldMaxSpeed) {
    setMaxSpeed(state.values.oldMaxSpeed);
    _state.values.oldMaxSpeed = 0;
  }
}

void Motor::setAcceleration(uint16_t value) {
  _settings.values.acceleration = value > settings.values.maxAcceleration ? settings.values.maxAcceleration : value;
  _stepper.setAcceleration(settings.values.acceleration);
}

void Motor::setMaxAcceleration(uint16_t value) {
  _settings.values.maxAcceleration = value > SYS_MAX_ACCELERATION ? SYS_MAX_ACCELERATION : value;
}

void Motor::overrideAcceleration(uint16_t value) {
  value = value > settings.values.maxAcceleration ? settings.values.maxAcceleration : value;
  if (settings.values.acceleration != value) {
    if (!state.values.oldAcceleration) {
      _state.values.oldAcceleration = settings.values.acceleration;
    }
    setAcceleration(value);
  }
}

void Motor::restoreAcceleration() {
  if (state.values.oldAcceleration) {
    setAcceleration(state.values.oldAcceleration);
    _state.values.oldAcceleration = 0;
  }
}

void Motor::setMaxSteps(uint32_t value) {
  _settings.values.maxSteps = value;
}

void Motor::setBackingSteps(uint16_t value) {
  _settings.values.backingSteps = value;
}

void Motor::setLimitSwitchEnablement(const boolean value) {
  auto& flags = _settings.values.flags;
  if ((boolean)((flags >> BitLimitsEnabled) & 1) != value) {
    flags = (flags & ~(1 << BitLimitsEnabled)) | (value << BitLimitsEnabled);
  }
}

void Motor::readLimitSwitches() {
  _state.values.flags = (
    (_state.values.flags & ~LIMIT_SWITCHES_MASK) |
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
      _stop(false);
    }
  }
  _state.values.lastActionTime = millis();
  _state.values.flags |= 1 << BitIsMoving;
  // auto& flags = _state.values.flags;
  // flags |= 1 << BitIsMoving;
  if (((_state.values.flags >> BitHasHomed) & 1) || ((_state.values.flags >> BitIsManualPos) & 1)) {
    _state.values.pos = _stepper.currentPosition();
    _state.values.targetPos = _stepper.targetPosition();
  }
}

void Motor::_updateIdle() {
  restoreAcceleration();
  restoreMaxSpeed();
  // auto& flags = _state.values.flags;
  if ((_state.values.flags >> BitIsBacking) & 1) {
    // we have finished backing for home
    // D_println("finished backing");
    _state.values.flags ^= 1 << BitIsBacking;
    moveHome();
    return;
  }
  if ((_state.values.flags >> BitIsForwarding) & 1) {
    // we have finished forwarding for end
    // D_println("finished forwarding");
    _state.values.flags ^= 1 << BitIsForwarding;
    moveEnd();
    return;
  }
  _state.values.flags &= ~((1 << BitIsHoming) | (1 << BitIsEnding) | (1 << BitIsMoving));
  if ((_state.values.flags >> BitIsStopping) & 1) {
    // we have finished stopping
    // D_println("finished stopping");
    _state.values.flags ^= 1 << BitIsStopping;
    if ((_state.values.flags >> BitIsForceStop) & 1) {
      _state.values.flags ^= 1 << BitIsForceStop;
    } else {
      // we have reached a limit switch, see if we are home
      if (isHome()) {
        _state.values.flags |= 1 << BitHasHomed;
        // _state.values.hasHomed = true;
        _stepper.setCurrentPosition(0);
        _stepper.setMaxSpeed(settings.values.maxSpeed);
        _state.values.pos = _stepper.currentPosition();
      } else if (isEnd() && ((_state.values.flags >> BitHasHomed) & 1)) {
        // store the known max position
        _state.values.posMax = _stepper.currentPosition();
      }
    }
  }
  _state.values.targetPos = _state.values.pos;
  _checkSleep();
}

boolean Motor::_stop(const boolean force) {
  // auto& flags = _state.values.flags;
  if (((_state.values.flags >> BitIsStopping) & 1) || !((_state.values.flags >> BitIsMoving) & 1)) {
    // skip duplicate or unnecessary action
    // D_println("ignoring stop");
    return false;
  }
  _state.values.flags = (_state.values.flags & ~(1 << BitIsForceStop)) | (force << BitIsForceStop) | (1 << BitIsStopping);
  overrideAcceleration(settings.values.maxAcceleration);
  _stepper.stop();
  return true;
}

void Motor::_enable() {
  // auto& flags = _state.values.flags;
  if (!((_state.values.flags >> BitIsActive) & 1)) {
    digitalWrite(pins.enable, MOTOR_ON);
    _state.values.flags |= 1 << BitIsActive;
    _state.values.enabledAt = millis();
  }
  _state.values.lastActionTime = millis();
}

void Motor::_disable() {
  // auto& flags = _state.values.flags;
  if ((_state.values.flags >> BitIsActive) & 1) {
    digitalWrite(pins.enable, 1 - MOTOR_ON);
    _state.values.flags &= ~(1 << BitIsActive);
    _state.values.enabledAt = 0;
  }
}

void Motor::_checkSleep() {
  // auto& flags = state.values.flags;
  if (((_state.values.flags >> BitIsActive )& 1) && !((_state.values.flags >> BitIsMoving) & 1) && millis() - state.values.lastActionTime > MOTOR_SLEEP_TIMEOUT_MS) {
    _disable();
  }
}

