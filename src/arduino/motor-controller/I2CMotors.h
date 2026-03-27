#ifndef I2C_MOTORS_H
#define I2C_MOTORS_H
#include <stddef.h>
#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>
#include "IMotor.h"

class I2CMotors {
public:
  static const uint8_t PAGE_REGISTER = 0x04;
  static const uint8_t MOTOR_BASE_ADDR = 0x08;
  static const uint8_t MOTOR_BLOCK_SIZE = 0x78;
  static const uint8_t TOTAL_BLOCK_SIZE = MOTOR_BLOCK_SIZE * 2 + MOTOR_BASE_ADDR;
  static const uint8_t SCRIPT_PAGE_SIZE = 0xF8;

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
  // Busy Protected:
  // position           | offset: 0x04 | span: 4 bytes
  // settings_flags     | offset: 0x10 | span: 1 bytes
  // max_speed          | offset: 0x14 | span: 4 bytes
  // acceleration       | offset: 0x18 | span: 4 bytes
  // move               | offset: 0x1c | span: 4 bytes
  // move_to            | offset: 0x20 | span: 4 bytes
  // delay              | offset: 0x24 | span: 4 bytes
  // script_exec        | offset: 0x2a | span: 1 bytes
  static const uint64_t MOTOR_BUSY_MASK = 0x04FFFFF100F0ULL;
  // Script Protected:
  // position           | offset: 0x04 | span: 4 bytes
  // settings_flags     | offset: 0x10 | span: 1 bytes
  // enable_delay_ms    | offset: 0x11 | span: 1 bytes
  // sleep_timeout_ms   | offset: 0x12 | span: 2 bytes
  // max_speed          | offset: 0x14 | span: 4 bytes
  // acceleration       | offset: 0x18 | span: 4 bytes
  // move               | offset: 0x1c | span: 4 bytes
  // move_to            | offset: 0x20 | span: 4 bytes
  // delay              | offset: 0x24 | span: 4 bytes
  // script_exec        | offset: 0x2a | span: 1 bytes
  static const uint64_t SCRIPT_LOCK_MASK = 0x04FFFFFF00F0ULL;
  static const uint16_t FAST_SYNC_MS = 20;
  static const uint16_t SLOW_SYNC_MS = 500;
  static const uint8_t NUM_SCRIPT_PAGES = 4;
  static const uint8_t SCRIPT_STACK_SIZE = 8;
  static const uint8_t MAX_MOTORS = 4;

  typedef enum {
    OK = 0x00,
    OTHER_ERROR = 0x07,
    MOTOR_BUSY = 0x1F,
    CANCELED = 0x20,
    UNKNOWN_COMMAND = 0x2C,
    INVALID_MOTOR = 0x2D,
    COMMAND_IGNORED = 0x2E,
    READONLY_ATTRIBUTE = 0x30,
    OVERFLOW = 0x31,
    UNSET = 0xFF,
  } ResCode;

  typedef enum {
    AND_STFLGS_RHS = 0x10,
    NAND_STFLGS_RHS = 0x11,
  } FunId;

#pragma pack(push, 1)
  struct MotorBlock {
    // 0x00 - Telemetry (Aligned)
    volatile uint8_t stateFlags;       // +0x00
    uint8_t _pad0[2];
    volatile uint8_t scriptRepCode;    // +0x03
    volatile int32_t currentPosition;  // +0x04
    volatile int32_t targetPosition;   // +0x08
    volatile float speed;              // +0x0C
    // 0x10 - Configuration
    volatile uint8_t settingsFlags;    // +0x10
    volatile uint8_t enableDelayMs;    // +0x11
    volatile uint16_t sleepTimeoutMs;  // +0x12
    volatile float maxSpeed;           // +0x14
    volatile float acceleration;       // +0x18
    // 0x1C - Actions
    volatile int32_t cmdMove;          // +0x1C
    volatile int32_t cmdMoveTo;        // +0x20
    volatile uint32_t cmdDelay;        // +0x24
    volatile uint8_t cmdStop;          // +0x28
    volatile uint8_t cmdScriptClear;   // +0x29
    volatile uint8_t cmdScriptExec;    // +0x2A
    uint8_t _pad1[1];
    volatile uint8_t scriptPage;       // +0x2C
    volatile uint8_t scriptIdx;        // +0x2D
    volatile uint8_t cmdCall[2];       // +0x2E
    volatile uint8_t cmdCondCall[4];   // +0x30
    volatile uint8_t cmdCondJump[4];   // +0x34
    volatile uint8_t cmdJump[2];       // +0x38
    uint8_t _pad2[2];

    // 0x3C - (Future)
    uint8_t _unallocated[0x3C];        // +0x3C - 0x77 future
    // 0x78 - Script buffer
    volatile uint8_t scripts[NUM_SCRIPT_PAGES][SCRIPT_PAGE_SIZE]; // 248 bytes. 4 pages = 992 bytes per motor
    volatile uint32_t _waitEndTime;
    volatile uint8_t scriptStackIdx[SCRIPT_STACK_SIZE];
    volatile uint8_t scriptStackPage[SCRIPT_STACK_SIZE];
    volatile uint8_t sp; // Stack Pointer
    volatile uint8_t _internalFlags;
  };

  struct DeviceMap {
    volatile uint8_t repCode;   // 0x00
    volatile uint8_t sysFlags;  // 0x01
    volatile uint16_t bootId;   // 0x02
    uint8_t _pad[4];
    MotorBlock motors[MAX_MOTORS];
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
  typedef enum {
    BitIsScriptActive = 0,
  } InternalFlagBit;
  TwoWire& wire;
  IMotor** motors;
  volatile uint8_t currentPage = 0;
  volatile RegisterMap mem;
  volatile uint8_t ptr = 0;
  volatile bool masterWriting = false;
  uint32_t lastFastSync = 0;
  uint32_t lastSlowSync = 0;
  uint8_t handleMotorWrite(const uint8_t mIdx, const uint8_t offset, const uint8_t incoming, const bool enforceBusy, const bool enforceScriptLock);
  void processScript();
  void processScript(const uint8_t mIdx);
  void exitScript(const uint8_t mIdx, const uint8_t code);
  void memSyncInterval();
  void syncAll();
  void syncMotorSettings();
  void syncMotorState();
  void syncAll(const uint8_t mIdx);
  void syncMotorSettings(const uint8_t mIdx);
  void syncMotorState(const uint8_t mIdx);
  bool isMotorBusy(const uint8_t mIdx);
  bool isScriptActive(const uint8_t mIdx);
  int8_t scriptCondition(const uint8_t mIdx, const uint8_t func, const uint8_t rhs);
  uint8_t scriptStackPush(const uint8_t mIdx, uint8_t page, const uint8_t sIdx);
  static bool isWriteable(const uint8_t page, const uint8_t ptr);
  static uint8_t getMidx(const uint8_t page, const uint8_t ptr);
  static uint8_t getStructOffset(const uint8_t ptr);
  static uint8_t getOpCodeDataLength(const uint8_t offset);
};

#endif
