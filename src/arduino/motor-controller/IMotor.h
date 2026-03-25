#ifndef IMOTOR_h
#define IMOTOR_h

class IMotor {
public:
  virtual ~IMotor() {}
  virtual bool move(int32_t) = 0;
  virtual bool stop() = 0;
  virtual bool busy() = 0;
  virtual bool scriptActive() = 0;
  virtual void setCurrentPosition(int32_t) = 0;
  virtual void setMaxSpeed(float) = 0;
  virtual void setAcceleration(float) = 0;
  virtual void setSettingsFlags(uint8_t) = 0;
  virtual void setScriptActive(bool) = 0;
  virtual uint8_t getStateFlags() = 0;
  virtual int32_t currentPosition() = 0;
  virtual int32_t targetPosition() = 0;
  virtual float speed() = 0;
  virtual uint8_t getSettingsFlags() = 0;
  virtual float maxSpeed() = 0;
  virtual float acceleration() = 0;
};
#endif
