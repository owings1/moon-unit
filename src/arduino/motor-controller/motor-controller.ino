#include <Wire.h>
#include <AccelStepper.h>
#include "Motor.h"
#include "I2CMotors.h"

#define I2C_MAIN Wire
#define SDA_MAIN D14
#define SCL_MAIN D13
#define I2C_FREQ 400000
#define I2C_ADDRESS 0x09
#define BAUD_RATE 9600L

Motor motors[] = {
  Motor(AccelStepper(AccelStepper::FULL2WIRE, D2, D1), {D0, D16, D15 }),
  Motor(AccelStepper(AccelStepper::FULL2WIRE, D8, D7), {D9, D17, D18 })
};

I2CMotors mainI2cMotors = I2CMotors(I2C_MAIN, motors, sizeof(motors) / sizeof(Motor));

void setup() { for (auto& m : motors) m.begin(); }
void loop() { for (auto& m : motors) m.run(); }
void mainRequestEvent() { mainI2cMotors.handleRead(); }
void mainReceiveEvent(int howMany) { mainI2cMotors.handleWrite(howMany); }

void setup1() {
  mainI2cMotors.setBootId(rp2040.hwrand32());
  I2C_MAIN.setSDA(SDA_MAIN);
  I2C_MAIN.setSCL(SCL_MAIN);
  I2C_MAIN.setClock(I2C_FREQ);
  I2C_MAIN.onRequest(mainRequestEvent);
  I2C_MAIN.onReceive(mainReceiveEvent);
  I2C_MAIN.begin(I2C_ADDRESS);
  Serial.begin(BAUD_RATE);
}

void loop1() {
  for (auto& m : motors) m.readLimitSwitches();
  mainI2cMotors.update();
}
