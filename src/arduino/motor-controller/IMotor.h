#include <sys/_stdint.h>
#ifndef IMOTOR_h
#define IMOTOR_h
typedef void (*MotorNotifyCallback)(void* context);
class IMotor {
public:
  virtual ~IMotor() {}
  virtual bool move(int32_t) = 0;
  virtual bool stop() = 0;
  virtual bool busy() = 0;
  virtual void setCurrentPosition(int32_t) = 0;
  virtual void setMaxSpeed(float) = 0;
  virtual void setAcceleration(float) = 0;
  virtual void setSettingsFlags(uint8_t) = 0;
  virtual void setScriptActive(bool) = 0;
  virtual void setDelayActive(bool) = 0;
  virtual void setSleepTimeoutMs(uint16_t) = 0;
  virtual void setEnableDelayMs(uint8_t) = 0;
  virtual void setNotify(MotorNotifyCallback, void*) = 0;
  virtual uint8_t stateFlags() = 0;
  virtual int32_t currentPosition() = 0;
  virtual int32_t targetPosition() = 0;
  virtual float speed() = 0;
  virtual uint8_t settingsFlags() = 0;
  virtual float maxSpeed() = 0;
  virtual float acceleration() = 0;
  virtual uint16_t sleepTimeoutMs() = 0;
  virtual uint8_t enableDelayMs() = 0;
};
#endif
