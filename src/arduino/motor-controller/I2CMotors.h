#include <sys/_stdint.h>
#ifndef I2C_MOTORS_H
#define I2C_MOTORS_H
#include <stddef.h>
#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>
#include "IMotor.h"
#include "MoicProtocol.h"
#include "MotorState.h"
#include "MotorActions.h"
#include "MotorManager.h"

class I2CMotors {
public:
  static const uint8_t PAGE_REGISTER = 0x04;
  static const uint8_t MOTOR_BASE_ADDR = 0x08;
  static const uint8_t LOWER_BLOCK_SIZE = Moic::MOTOR_BLOCK_SIZE + MOTOR_BASE_ADDR;
  static const uint8_t TOTAL_BLOCK_SIZE = Moic::MOTOR_BLOCK_SIZE * 2 + MOTOR_BASE_ADDR;
  static const uint8_t SCRIPT_PAGE_START = 0x10;
  static const uint8_t MAX_MOTORS = 2;

  // Writable: 0x01 (sysFlags), 0x04 (PageReg)
  static const uint8_t DEVICE_WRITE_MASK = 0x12;
  static const uint16_t FAST_SYNC_MS = 20;
  static const uint16_t SLOW_SYNC_MS = 500;


#pragma pack(push, 1)
  struct DeviceMap {
    volatile uint8_t repCode;   // 0x00
    volatile uint8_t sysFlags;  // 0x01
    volatile uint16_t bootId;   // 0x02
    uint8_t _pad[4];
  };
#pragma pack(pop)

  I2CMotors(TwoWire& wire, Moic::ManagedMotor** mms, uint8_t count);

  void setBootId(uint16_t id);
  void update();

  void handleRead();
  void handleWrite(int howMany);

  const uint8_t numMotors;
private:
  volatile DeviceMap regs;
  Moic::ManagedMotor** mms;
  TwoWire& wire;
  volatile uint8_t currentPage = 0;
  volatile uint8_t ptr = 0;
  volatile bool masterWriting = false;
  uint32_t lastFastSync = 0;
  uint32_t lastSlowSync = 0;
  void memSyncInterval();
  static bool isWriteable(const uint8_t page, const uint8_t ptr);
  static uint8_t getMidx(const uint8_t page, const uint8_t ptr);
  static uint8_t getStructOffset(const uint8_t ptr);
};

#endif
