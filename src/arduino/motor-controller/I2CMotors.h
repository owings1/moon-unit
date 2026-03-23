#ifndef I2C_MOTORS_H
#define I2C_MOTORS_H
#include <Arduino.h>
#include <Wire.h>
#include <stddef.h>
#include <AccelStepper.h>
#include "Motor.h"

/*
================================================================================
MOTOR CONTROLLER - I2C REGISTER MAP (32-BYTE BLOCKS)
================================================================================
ADDR   | SIZE | NAME             | TYPE  | ACCESS | DESCRIPTION
--------------------------------------------------------------------------------
0x00   | 1    | REP_CODE         | uint8 | R      | Result of last write/cmd
0x01   | 1    | SYS_FLAGS        | uint8 | R/W    | Global system status/control
0x02   | 2    | BOOT_ID          | uint16| R      | Random session ID (Reset detect)
0x04   | 28   | _RESERVED        | -     | -      | Padding to 0x20
--------------------------------------------------------------------------------
MOTOR BLOCK (BASE + OFFSET)
--------------------------------------------------------------------------------
+0x00  | 1    | STATE_FLAGS      | uint8 | R      | [Bit 0]
+0x01  | 4    | POS              | int32 | R/W    | [Bits 1-4] Coord Sync
+0x05  | 4    | TARGET_POS       | int32 | R      | [Bits 5-8] Internal Target
+0x09  | 2    | CUR_SPEED        | uint16| R      | [Bits 9-10] Current Speed
+0x0B  | 1    | SETTINGS_FLAGS   | uint8 | R/W    | [Bit 11] Config
+0x0C  | 2    | MAX_SPEED        | uint16| R/W    | [Bits 12-13] Config
+0x0E  | 2    | ACCELERATION     | uint16| R/W    | [Bits 14-15] Config
+0x10  | 4    | CMD_MOVE         | int32 | W      | [Bits 16-19] Trigger Move
+0x14  | 1    | CMD_STOP         | uint8 | W      | [Bit 20] Trigger Stop
+0x15  | 11   | _RESERVED        | -     | -      | Padding to 0x20
================================================================================
BITMASK CONSTANTS (HEX)
================================================================================
DEVICE_WRITE_MASK = 0x0000003E  // Bits: 1 (sysFlags)
MOTOR_WRITE_MASK  = 0x001FF81E  // Bits: 1-4, 11, 12-13, 14-15, 16-19, 20
MOTOR_BUSY_MASK   = 0x000FEF1E  // Bits: 1-4, 12-13, 14-15, 16-19
================================================================================

*/
class I2CMotors {
public:
  static const uint8_t MOTOR_BASE_ADDR = 0x20;
  static const uint8_t MOTOR_BLOCK_SIZE = 0x20;
  static const uint32_t DEVICE_WRITE_MASK = 0x0000003E;  // Bits 1-5
  static const uint32_t MOTOR_WRITE_MASK = 0x001FF81E;   // Bits 1-4, 11-20
  static const uint32_t MOTOR_BUSY_MASK = 0x000FEF1E;    // Bits 1-4, 12-19
  static const uint16_t FAST_SYNC_MS = 20;
  static const uint16_t SLOW_SYNC_MS = 500;

  typedef enum {
    OK = 0x00,
    MOTOR_BUSY = 0x1F,
    INVALID_MOTORID = 0x2D,
    COMMAND_IGNORED = 0x2E,
    READONLY_ATTRIBUTE = 0x30,
    UNSET = 0xFF,
  } ResCode;

#pragma pack(push, 1)
  struct MotorBlock {
    volatile uint8_t stateFlags;     // +0x00
    volatile int32_t pos;            // +0x01
    volatile int32_t targetPos;      // +0x05
    volatile uint16_t speed;         // +0x09
    volatile uint8_t settingsFlags;  // +0x0B
    volatile uint16_t maxSpeed;      // +0x0C
    volatile uint16_t acceleration;  // +0x0E
    volatile int32_t cmdMove;        // +0x10
    volatile uint8_t cmdStop;        // +0x14
    uint8_t _reserved[11];
  };

  struct DeviceMap {
    volatile uint8_t repCode;    // 0x00
    volatile uint8_t sysFlags;   // 0x01
    volatile uint16_t bootId;    // 0x02
    uint8_t _pad[28];            // To 0x20
    MotorBlock motors[4];        // Support up to 4
  };
#pragma pack(pop)

  union RegisterMap {
    DeviceMap regs;
    uint8_t buffer[sizeof(DeviceMap)];
  };

  I2CMotors(TwoWire& wire, Motor* motorPtr, uint8_t motorCount);

  void begin(uint8_t address, uint32_t freq);
  void setBootId(uint16_t id);
  void update();

  void handleRead();
  void handleWrite(int howMany);
  const uint8_t numMotors;

private:
  TwoWire& wire;
  Motor* motors;

  volatile RegisterMap mem;
  volatile uint8_t ptr = 0;
  volatile bool masterWriting = false;

  uint32_t lastFastSync = 0;
  uint32_t lastSlowSync = 0;

  void handleMotorWrite(const uint8_t motorIdx, const uint8_t offset, const uint8_t incoming);
  void memSyncInterval();
  void syncAll();
  void syncAll(Motor& m);
  void syncMotorState();
  void syncMotorState(Motor& m);
  void syncMotorSettings();
  void syncMotorSettings(Motor& m);

  static bool isWriteable(uint8_t p);
  static uint8_t motorOffset(const uint8_t p);
};

#endif
