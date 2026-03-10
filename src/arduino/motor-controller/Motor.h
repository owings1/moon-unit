#include <stdint.h>
#include <sys/_types.h>
#ifndef Motor_h
#define Motor_h

#include <Arduino.h>
#include <AccelStepper.h>

class Motor {

public:
  static const uint32_t POS_NULL = 10000000UL;
  static const uint16_t SYS_MAX_SPEED = 0xffff;
  static const uint16_t SYS_MAX_ACCELERATION = 0xffff;
  typedef enum {
    // limit switch states
    BitIsLimitCw = 0,
    BitIsLimitAcw = 1,
    // is the stepper motor engaged
    BitIsActive = 2,
    // is the motor moving
    BitIsMoving = 3,
    // // whether homing has occurred, and thus positions are meaningful
    // BitHasHomed = 4,
    // // flag to indicate homing in progress
    // BitIsHoming = 5,
    // // flag to indicate ending in progress
    // BitIsEnding = 6,
    // // when a motor is stopped by a command instead of naturally from limit
    // BitIsForceStop = 7,
    // flag to reset acceleration to oldAcceleration after motors are finished
    // running, for smooth stop on limits.  
    BitIsStopping = 8,
    // // flag for when we are backing up for homing purposes, so that immediately
    // // after we can re-initiate homing.
    // BitIsBacking = 9,
    // // as above for ending purposes
    // BitIsForwarding = 10,
    // pos was manually set
    BitIsManualPos = 11,
  } StateFlagBit;

  typedef enum {
    BitLimitsEnabled = 0,
  } SettingsFlagBit;

  struct Pins {
    uint8_t dir;
    uint8_t step;
    uint8_t enable;
    uint8_t limit_cw;
    uint8_t limit_acw;
  };

  struct __attribute__((packed)) Settings {
    volatile uint8_t flags = 0x0 | (1 << BitLimitsEnabled);
    // volatile uint16_t defaultSpeed = 2000;
    // volatile uint16_t homingSpeed = 4000;
    volatile uint16_t maxSpeed = 2000;
    volatile uint16_t absMaxSpeed = 5000;
    volatile uint16_t acceleration = 50000;
    volatile uint16_t maxAcceleration = 50000;
    // volatile uint32_t maxSteps = 5000;
    // volatile uint16_t backingSteps = 1000;
  };

  union SettingsUnion {
    Settings values = {};
    byte buf[sizeof(Settings)];
  };

  struct __attribute__((packed)) State {
    volatile uint16_t flags;
    int32_t pos = POS_NULL;
    int32_t targetPos = POS_NULL;
    // measured effective range after calibration
    // uint32_t posMax;
    // for delaying after enabling motor
    volatile uint32_t enabledAt;
    // for checking motor sleep
    volatile uint32_t lastActionTime;
    // for temporarily overriding acceleration during stopping.
    volatile uint16_t oldAcceleration;
    // for temporarily overriding max speed during timing.
    volatile uint16_t oldMaxSpeed;
  };

  union StateUnion {
    State values = {};
    byte buf[sizeof(State)];
  };

  Motor(Motor::Pins pins);

  const uint8_t id;
  const Motor::Pins pins;
  const Motor::SettingsUnion &settings;
  const Motor::StateUnion &state;

  void begin();

  boolean runIfNeeded();

  boolean canMove(const int32_t direction);
  boolean move(const int32_t howMuch);
  // boolean moveHome();
  // boolean moveEnd();
  boolean stop();
  // boolean isHome();
  // boolean isEnd();

  void setPosition(int32_t value);
  // void setDefaultSpeed(uint16_t value);
  // void setHomingSpeed(uint16_t value);
  void setMaxSpeed(uint16_t value);
  void setAbsMaxSpeed(uint16_t value);
  void overrideMaxSpeed(uint16_t value);
  void restoreMaxSpeed();
  void setAcceleration(uint16_t value);
  void setMaxAcceleration(uint16_t value);
  void overrideAcceleration(uint16_t value);
  void restoreAcceleration();
  // void setMaxSteps(uint32_t value);
  // void setBackingSteps(uint16_t value);
  void setLimitSwitchEnablement(const boolean value);
  void readLimitSwitches();

private:
  // Motor::Parameters _params;
  Motor::SettingsUnion _settings;
  Motor::StateUnion _state;
  AccelStepper _stepper;
  void _runActive();
  void _updateIdle();
  boolean _stop(const boolean force);
  void _enable();
  void _disable();
  void _checkSleep();
};

#endif