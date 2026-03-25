#ifndef MOTOR_WRAPPER_H
#define MOTOR_WRAPPER_H
#include "IMotor.h"
#include <AccelStepper.h>

class MotorWrapper : public IMotor {

public:
  MotorWrapper(AccelStepper& s) : stepper(s) {}
  bool move(int32_t value) override { stepper.move(value); return true; }
  bool stop() override { stepper.stop(); return true; }
  bool scriptActive() override { return false; }
  void setCurrentPosition(int32_t value) override { stepper.setCurrentPosition(value); }
  void setMaxSpeed(float value) override { stepper.setMaxSpeed(value); }
  void setAcceleration(float value) override { stepper.setAcceleration(value); }
  void setSettingsFlags(uint8_t value) override { settingsFlags = value; }
  void setScriptActive(bool value) override { }
  uint8_t getStateFlags() override { return 0x00; }
  int32_t currentPosition() override { return (int32_t) stepper.currentPosition(); }
  int32_t targetPosition() override { return (int32_t) stepper.targetPosition(); }
  float speed() override { return (float) abs(stepper.speed()); }
  uint8_t getSettingsFlags() override { return settingsFlags; }
  float maxSpeed() override { return (float) stepper.maxSpeed(); }
  float acceleration() override { return (float) stepper.acceleration(); }   
  bool busy() override { return stepper.isRunning(); }

protected:
  AccelStepper& stepper;
  volatile uint8_t settingsFlags = 0;
};

#endif