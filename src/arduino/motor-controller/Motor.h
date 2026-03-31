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
    BitIsLimitCw = 0,
    BitIsLimitAcw = 1,
    BitIsActive = 2,
    BitIsMoving = 3,
    BitIsStopping = 4,
    BitIsManualPos = 5,
    BitIsScriptActive = 6,
    BitIsDelayActive = 7,
  } StateFlagBit;

  typedef enum {
    BitLimitsEnabled = 0,
    BitSleepEnabled = 1,
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
  void setScriptActive(const bool) override;
  void setDelayActive(const bool) override;
  // bool scriptActive() override;

  void begin();
  void readLimitSwitches();
  bool run();

protected:
  static const uint8_t LIMIT_SWITCHES_MASK = (1 << BitIsLimitCw) | (1 << BitIsLimitAcw);
  static const uint8_t SETTINGS_FLAGS_MASK = (1 << BitLimitsEnabled) | (1 << BitSleepEnabled);
  // for delaying after enabling motor
  volatile uint32_t enabledAt;
  // for checking motor sleep
  volatile uint32_t lastActionTime;
  bool canMove(const int32_t);

private:
  volatile float _accelerationSaved;
  void _runActive();
  void _updateIdle();
  void _enable();
  void _disable();
  void _checkSleep();
};

#endif