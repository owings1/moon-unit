#include <stdint.h>
#include <sys/_types.h>
#ifndef Motor_h
#define Motor_h

#include <Arduino.h>
#include <AccelStepper.h>

class Motor {

public:
  static const uint16_t STOP_DECELERATION = 0xFFFF;
  typedef enum {
    // limit switch states
    BitIsLimitCw = 0,
    BitIsLimitAcw = 1,
    // is the stepper motor engaged
    BitIsActive = 2,
    // is the motor moving
    BitIsMoving = 3,
    BitIsStopping = 4,
    // pos was manually set
    BitIsManualPos = 5,
  } StateFlagBit;

  typedef enum {
    BitLimitsEnabled = 0,
  } SettingsFlagBit;

  struct Pins {
    uint8_t enable = 0;
    uint8_t limit_cw = 0;
    uint8_t limit_acw = 0;
  };

  Motor(AccelStepper stepper, Motor::Pins pins);

  const uint8_t id;
  const Motor::Pins pins;
  const volatile uint8_t &stateFlags;
  const volatile uint8_t &settingsFlags;

  void begin();

  bool run();
  bool move(const int32_t howMuch);
  bool stop();

  void setCurrentPosition(const int32_t value);
  void setMaxSpeed(const uint16_t value);
  void setAcceleration(const uint16_t value);
  void setSettingsFlags(const uint8_t value);
  void readLimitSwitches();
  int32_t currentPosition();
  int32_t targetPosition();
  uint16_t maxSpeed();
  uint16_t acceleration();
  uint16_t speed();
protected:
  // for delaying after enabling motor
  volatile uint32_t enabledAt;
  // for checking motor sleep
  volatile uint32_t lastActionTime;
  bool canMove(const int32_t direction);
  AccelStepper stepper;

private:
  volatile uint8_t _stateFlags;
  volatile uint8_t _settingsFlags;
  volatile uint16_t _accelerationSaved;
  void _runActive();
  void _updateIdle();
  void _enable();
  void _disable();
  void _checkSleep();
};

#endif