#include <sys/_stdint.h>
#include <stdint.h>
#ifndef I2C_MOTORS_H
#define I2C_MOTORS_H
#include <Arduino.h>
#include <Wire.h>
#include <stddef.h>
#include "IMotor.h"

class I2CMotors {
public:
  static const uint8_t PAGE_REGISTER = 0x04;
  static const uint8_t MOTOR_BASE_ADDR = 0x08;
  static const uint8_t MOTOR_BLOCK_SIZE = 0x78;
  static const uint8_t TOTAL_BLOCK_SIZE = MOTOR_BLOCK_SIZE * 2 + MOTOR_BASE_ADDR;

  // Writable: 0x01 (sysFlags), 0x04 (PageReg)
  static const uint8_t DEVICE_WRITE_MASK = 0x12;
  // Writable: pos, settingsFlags, maxSpeed, accel, cmdMove, cmdStop
  // Binary 1 at bits: 4-7, 16, 20-23, 24-27, 28-31, 32
  static const uint64_t MOTOR_WRITE_MASK = 0x0001FFF100F0ULL;
  // Busy Protected: pos, maxSpeed, accel, cmdMove
  // Bits: 4-7, 20-23, 24-27, 28-31
  static const uint64_t MOTOR_BUSY_MASK = 0x0000FFF100F0ULL;

  static const uint16_t FAST_SYNC_MS = 20;
  static const uint16_t SLOW_SYNC_MS = 500;

  typedef enum {
    OK = 0x00,
    OTHER_ERROR = 0x07,
    MOTOR_BUSY = 0x1F,
    UNKNOWN_COMMAND = 0x2C,
    INVALID_MOTOR = 0x2D,
    COMMAND_IGNORED = 0x2E,
    READONLY_ATTRIBUTE = 0x30,
    UNSET = 0xFF,
  } ResCode;

#pragma pack(push, 1)
  struct MotorBlock {
    // 0x00 - Telemetry (Aligned)
    volatile uint8_t stateFlags;      // +0x00
    uint8_t _pad0[3];
    volatile int32_t currentPosition; // +0x04
    volatile int32_t targetPosition;  // +0x08
    volatile float speed;             // +0x0C
    // 0x10 - Configuration
    volatile uint8_t settingsFlags;   // +0x10
    uint8_t _pad1[3];
    volatile float maxSpeed;          // +0x14
    volatile float acceleration;      // +0x18
    // 0x1C - Actions
    volatile int32_t cmdMove;         // +0x1C
    volatile uint8_t cmdStop;         // +0x20
    uint8_t _pad2[3];
    // 0x24 - (Future)
    uint8_t _morestuff[0x54];         // +0x24 - 0x77 Pad to 120 (for 0x78 total size)
  };

  struct DeviceMap {
    volatile uint8_t repCode;   // 0x00
    volatile uint8_t sysFlags;  // 0x01
    volatile uint16_t bootId;   // 0x02
    uint8_t _pad[4];
    MotorBlock motors[4];  // Support up to 4
  };
#pragma pack(pop)

  union RegisterMap {
    DeviceMap regs;
    uint8_t buffer[sizeof(DeviceMap)];
  };

  I2CMotors(TwoWire& wire, IMotor** motors, uint8_t count);

  void begin(uint8_t address, uint32_t freq);
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
  uint8_t handleMotorWrite(const uint8_t, const uint8_t, const uint8_t);
  void memSyncInterval();
  void syncAll();
  void syncMotorSettings();
  void syncMotorState();
  void syncAll(const uint8_t);
  void syncMotorSettings(const uint8_t);
  void syncMotorState(const uint8_t);
  static bool isWriteable(const uint8_t);
  static uint8_t getMidx(const uint8_t, const uint8_t);
  static uint8_t getStructOffset(const uint8_t);
};

#endif
