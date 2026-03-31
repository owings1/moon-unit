#include <sys/_stdint.h>
#ifndef MOTOR_WRAPPER_H
#define MOTOR_WRAPPER_H
#include "IMotor.h"
#include <AccelStepper.h>

class MotorWrapper : public IMotor {

public:
  MotorWrapper(AccelStepper& s) : stepper(s) {}
  bool move(int32_t value) override { stepper.move(value); return true; }
  bool stop() override { stepper.stop(); return true; }
  void setCurrentPosition(int32_t value) override { stepper.setCurrentPosition(value); }
  void setMaxSpeed(float value) override { stepper.setMaxSpeed(value); }
  void setAcceleration(float value) override { stepper.setAcceleration(value); }
  void setSettingsFlags(uint8_t value) override { _settingsFlags = value; }
  void setScriptActive(bool value) override { }
  void setDelayActive(bool value) override { }
  void setSleepTimeoutMs(uint16_t value) override { _sleepTimeoutMs = value; }
  void setEnableDelayMs(uint8_t value) override { _enableDelayMs = value; }
  uint8_t stateFlags() override { return _stateFlags; }
  uint8_t settingsFlags() override { return _settingsFlags; }
  int32_t currentPosition() override { return stepper.currentPosition(); }
  int32_t targetPosition() override { return stepper.targetPosition(); }
  float speed() override { return (float) abs(stepper.speed()); }
  float maxSpeed() override { return (float) stepper.maxSpeed(); }
  float acceleration() override { return (float) stepper.acceleration(); }
  uint16_t sleepTimeoutMs() override { return _sleepTimeoutMs; }
  uint8_t enableDelayMs() override { return _enableDelayMs; }
  bool busy() override { return stepper.isRunning(); }

protected:
  AccelStepper& stepper;
  uint8_t _settingsFlags = 0;
  uint8_t _stateFlags = 0;
  uint16_t _sleepTimeoutMs = 2000;
  uint8_t _enableDelayMs = 2;

};

#endif