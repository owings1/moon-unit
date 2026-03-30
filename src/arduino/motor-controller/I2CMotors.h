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
#include "MotorVM.h"

class I2CMotors {
public:
  static const uint8_t PAGE_REGISTER = 0x04;
  static const uint8_t MOTOR_BASE_ADDR = 0x08;
  static const uint8_t LOWER_BLOCK_SIZE = Moic::MOTOR_BLOCK_SIZE + MOTOR_BASE_ADDR;
  static const uint8_t TOTAL_BLOCK_SIZE = Moic::MOTOR_BLOCK_SIZE * 2 + MOTOR_BASE_ADDR;
  static const uint8_t SCRIPT_PAGE_START = 0x10;
  static const uint8_t MAX_MOTORS = 4;

  // Writable: 0x01 (sysFlags), 0x04 (PageReg)
  static const uint8_t DEVICE_WRITE_MASK = 0x12;
  // Writeable:
  // position           | offset: 0x04 | span: 4 bytes
  // settings_flags     | offset: 0x10 | span: 1 bytes
  // enable_delay_ms    | offset: 0x11 | span: 1 bytes
  // sleep_timeout_ms   | offset: 0x12 | span: 2 bytes
  // max_speed          | offset: 0x14 | span: 4 bytes
  // acceleration       | offset: 0x18 | span: 4 bytes
  // move               | offset: 0x1c | span: 4 bytes
  // move_to            | offset: 0x20 | span: 4 bytes
  // delay              | offset: 0x24 | span: 4 bytes
  // stop               | offset: 0x28 | span: 1 bytes
  // script_clear       | offset: 0x29 | span: 1 bytes
  // script_exec        | offset: 0x2a | span: 1 bytes
  static const uint64_t MOTOR_WRITE_MASK = 0x07FFFFFF00F0ULL;
  static const uint16_t FAST_SYNC_MS = 20;
  static const uint16_t SLOW_SYNC_MS = 500;


#pragma pack(push, 1)

  struct DeviceMap {
    volatile uint8_t repCode;   // 0x00
    volatile uint8_t sysFlags;  // 0x01
    volatile uint16_t bootId;   // 0x02
    uint8_t _pad[4];
    Moic::MotorBlock motors[MAX_MOTORS];
  };
#pragma pack(pop)

  union RegisterMap {
    DeviceMap regs;
    uint8_t buffer[sizeof(DeviceMap)];
  };

  I2CMotors(TwoWire& wire, IMotor** motors, uint8_t count);

  void setBootId(uint16_t id);
  void update();

  void handleRead();
  void handleWrite(int howMany);
  const uint8_t numMotors;

private:
  TwoWire& wire;
  IMotor** motors;
  volatile uint8_t currentPage = 0;
  volatile RegisterMap mem;
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
