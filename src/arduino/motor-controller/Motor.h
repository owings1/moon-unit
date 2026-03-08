#include <stdint.h>
#include <sys/_types.h>
#ifndef Motor_h
#define Motor_h

#include <Arduino.h>
#include <AccelStepper.h>

class Motor {

public:
  static const unsigned long POS_NULL = 10000000UL;
  typedef enum {
    // limit switch states
    BitIsLimitCw = 0,
    BitIsLimitAcw = 1,
    // is the stepper motor engaged
    BitIsActive = 2,
    // is the motor moving
    BitIsMoving = 3,
    // whether homing has occurred, and thus positions are meaningful
    BitHasHomed = 4,
    // flag to indicate homing in progress
    BitIsHoming = 5,
    // flag to indicate ending in progress
    BitIsEnding = 6,
    // when a motor is stopped by a command instead of naturally from limit
    BitIsForceStop = 7,
    // flag to reset acceleration to oldAcceleration after motors are finished
    // running, for smooth stop on limits.  
    BitIsStopping = 8,
    // flag for when we are backing up for homing purposes, so that immediately
    // after we can re-initiate homing.
    BitIsBacking = 9,
    // as above for ending purposes
    BitIsForwarding = 10,
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

  struct Parameters {
    unsigned long millistepsPerDegree;
    unsigned int maxDegrees;
    unsigned long absMaxSpeed = 5000;
    unsigned long defaultSpeed = 2000;
    unsigned long maxAcceleration = 50000;
  };

  struct Settings {
    volatile uint8_t flags = 0x0 | (1 << BitLimitsEnabled);
    volatile unsigned long homingSpeed = 4000;
    volatile unsigned long acceleration;
    volatile unsigned long maxSpeed;
  };

  struct State {
    volatile uint16_t flags;
    long pos = POS_NULL;
    long targetPos = POS_NULL;
    // measured effective range after calibration
    unsigned long posMax;
    // for delaying after enabling motor
    volatile unsigned long enabledAt;
    // for checking motor sleep
    volatile unsigned long lastActionTime;
    // for temporarily overriding acceleration during stopping.
    volatile unsigned long oldAcceleration;
    // for temporarily overriding max speed during timing.
    volatile unsigned long oldMaxSpeed;
  };

  Motor(Motor::Pins pins, Motor::Parameters parameters);

  const uint8_t id;
  const Motor::Pins pins;
  const Motor::Parameters &parameters;
  const Motor::Settings &settings;
  const Motor::State &state;

  void begin();

  boolean runIfNeeded();

  boolean canMove(const long direction);
  boolean move(const long howMuch);
  boolean moveHome();
  boolean moveEnd();
  boolean stop();
  boolean isHome();
  boolean isEnd();

  void setMaxSpeed(unsigned long value);
  void setHomingSpeed(unsigned long value);
  void setAcceleration(unsigned long value);
  void overrideMaxSpeed(unsigned long value);
  void overrideAcceleration(unsigned long value);
  void restoreMaxSpeed();
  void restoreAcceleration();
  void setLimitSwitchEnablement(const boolean value);
  void readLimitSwitches();

private:
  Motor::Parameters _params;
  Motor::Settings _settings;
  Motor::State _state;
  AccelStepper _stepper;
  void _runActive();
  void _updateIdle();
  boolean _stop(const boolean force);
  void _enable();
  void _disable();
  void _checkSleep();
  long _degtos(const float howMuch);
  unsigned long _getOverLimitStepsToMove();
};

#endif