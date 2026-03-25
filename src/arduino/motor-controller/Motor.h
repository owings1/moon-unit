#include <stdint.h>
#include <sys/_types.h>
#ifndef Motor_h
#define Motor_h
#include "IMotor.h"
#include "MotorWrapper.h"

#include <Arduino.h>
#include <AccelStepper.h>

class Motor : public MotorWrapper {

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
    uint8_t enable = NOPIN;
    uint8_t limit_cw = NOPIN;
    uint8_t limit_acw = NOPIN;
  };

  Motor(AccelStepper&, Motor::Pins);

  const Motor::Pins pins;

  bool move(const int32_t) override;
  bool stop() override;
  bool busy() override;

  void setCurrentPosition(const int32_t) override;
  void setSettingsFlags(const uint8_t) override;
  uint8_t getStateFlags() override;

  void begin();
  void readLimitSwitches();
  bool run();

protected:
  const uint8_t LIMIT_SWITCHES_MASK = (1 << BitIsLimitCw) | (1 << BitIsLimitAcw);
  const uint8_t SETTINGS_FLAGS_MASK = (1 << BitLimitsEnabled);
  // for delaying after enabling motor
  volatile uint32_t enabledAt;
  // for checking motor sleep
  volatile uint32_t lastActionTime;
  bool canMove(const int32_t);
  volatile uint8_t stateFlags;

private:
  volatile float _accelerationSaved;
  void _runActive();
  void _updateIdle();
  void _enable();
  void _disable();
  void _checkSleep();
};

#endif