#include <Wire.h>
#include <AccelStepper.h>
#include "Motor.h"
#include "I2CMotors.h"
#include "IMotor.h"
#include "MotorState.h"


#include "MotorManager.h"

#define I2C_MAIN Wire
#define SDA_MAIN D14
#define SCL_MAIN D13
#define I2C_FREQ 400000
#define I2C_ADDRESS 0x09
#define BAUD_RATE 9600L

AccelStepper s1(AccelStepper::FULL2WIRE, D2, D1, NOPIN, NOPIN);
AccelStepper s2(AccelStepper::FULL2WIRE, D8, D7, NOPIN, NOPIN);
Motor m1(s1, { D0, D16, D15 });
Motor m2(s2, { D9, D17, D18 });
Motor* motors[] = { &m1, &m2 };
Moic::ManagedMotor mm1(&m1);
Moic::ManagedMotor mm2(&m2);
Moic::ManagedMotor* mms[] = { &mm1, &mm2 };
I2CMotors mainI2cMotors = I2CMotors(I2C_MAIN, mms, 2);


void setup() {
  for (auto& m : motors) m->begin();
}
// void loop() { for (auto& m : motors) m->run(); }
void loop() {
  static uint32_t lastMicros = 0;

  uint32_t now = micros();
  uint32_t delta = now - lastMicros;
  bool isrun = false;
  for (auto& m : motors) {
    isrun = m->run() || isrun;
  }
  if (!isrun) {
    for (auto& m : motors) m->readLimitSwitches();
  }
  if (lastMicros > 0) {
    mainI2cMotors.observeDelta(delta);
  }
  lastMicros = now;
}
void mainRequestEvent() {
  mainI2cMotors.handleRead();
}
void mainReceiveEvent(int howMany) {
  mainI2cMotors.handleWrite(howMany);
}

void setup1() {
  mainI2cMotors.setBootId(rp2040.hwrand32());
  I2C_MAIN.setSDA(SDA_MAIN);
  I2C_MAIN.setSCL(SCL_MAIN);
  I2C_MAIN.setClock(I2C_FREQ);
  I2C_MAIN.onRequest(mainRequestEvent);
  I2C_MAIN.onReceive(mainReceiveEvent);
  I2C_MAIN.begin(I2C_ADDRESS);
  // Serial.begin(BAUD_RATE);
}

#include "MotorActions.h"

void loop1() {
  mainI2cMotors.update();
  // delay(0x01);
  // 2. THE PROTO-MANAGER (Orchestration)
  // We only run this every 1ms to keep Core 0 happy
  static uint32_t lastPerfSync = 0;
  static uint32_t lastVM = 0;
  const int now = millis();
  if (now - lastVM >= 1) {
    lastVM = now;

    for (uint8_t i = 0; i < 2; ++i) {
      if (mms[i]->scriptActive()) {
        mms[i]->tick();
      }
    }
  }
  if (now - lastPerfSync >= 500) {
    if (mainI2cMotors.updatePerf()) {
      lastPerfSync = now;
    }
  }
}
