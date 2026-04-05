#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <Arduino.h>
#include "MoicProtocol.h"
#include "IMotor.h"
#include "MotorState.h"

namespace Moic {

class ManagedMotor {
public:
  static const uint16_t FAST_SYNC_MS = 20;
  static const uint16_t SLOW_SYNC_MS = 500;
  ManagedMotor(IMotor* m);
  uint8_t write(const uint8_t offset, const uint8_t incoming, const uint8_t source);
  void tick();
  bool busy();
  bool scriptActive();
  uint8_t enterScript(uint8_t page, const uint8_t arg);
  void exitScript(const uint8_t code);
  bool isPageInStack(const uint8_t page);
  IMotor* m;
  volatile MotorInterface* mregs;
  VMContext* vmctx;
  void syncLockInc();
  void syncLockDec();
  void setForceStateSyncAt(const uint32_t);
private:
  MotorInterface _mregs;
  VMContext _vmctx;
  bool _scriptActive = false;
  bool _isBusyFast = false;
  uint32_t lastFastSync = 0;
  uint32_t lastSlowSync = 0;
  uint32_t forceStateSyncAt = 0;
  uint8_t syncLockout = 0;
  void memSyncInterval();
  void syncMotorState();
  void syncMotorSettings();
  void syncScriptState();
};

static const uint8_t OFFSET_WRITEMASKS[0x40] = {
  0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
  0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
  0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
  0x83, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00,
  0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
}

// current_position   | 0x04 | 4 bytes
// settings_flags     | 0x10 | 1 bytes
// enable_delay_ms    | 0x11 | 1 bytes
// sleep_timeout_ms   | 0x12 | 2 bytes
// max_speed          | 0x14 | 4 bytes
// acceleration       | 0x18 | 4 bytes
// move               | 0x1C | 4 bytes
// move_to            | 0x20 | 4 bytes
// delay              | 0x24 | 4 bytes
// stop               | 0x28 | 1 bytes
// move_rev           | 0x30 | 4 bytes
// const uint64_t VMEXC_WRITE_MASK = 0x000F01FFFFFF00F0;
// current_position   | 0x04 | 4 bytes
// settings_flags     | 0x10 | 1 bytes
// enable_delay_ms    | 0x11 | 1 bytes
// sleep_timeout_ms   | 0x12 | 2 bytes
// max_speed          | 0x14 | 4 bytes
// acceleration       | 0x18 | 4 bytes
// move               | 0x1C | 4 bytes
// move_to            | 0x20 | 4 bytes
// delay              | 0x24 | 4 bytes
// stop               | 0x28 | 1 bytes
// script_clear       | 0x29 | 1 bytes
// script_exec        | 0x2A | 2 bytes
// move_rev           | 0x30 | 4 bytes
// const uint64_t BUSIO_WRITE_MASK = 0x000F0FFFFFFF00F0;
// stop               | 0x28 | 1 bytes
// const uint64_t BUSY_WRITE_MASK = 0x0000010000000000;

#endif
