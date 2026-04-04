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
#include "Moperf.h"

class I2CMotors {
public:
  static const uint8_t PAGE_REGISTER = 0x04;
  // Constraint: Needs to be no more than (0x100 - SCRIPT_PAGE_SIZE)
  //             to support global registers on all pages.
  static const uint8_t MOTOR_BASE_ADDR = 0x08;
  static const uint8_t LOWER_BLOCK_SIZE = Moic::MOTOR_BLOCK_SIZE + MOTOR_BASE_ADDR;
  static const uint8_t TOTAL_BLOCK_SIZE = Moic::MOTOR_BLOCK_SIZE * 2 + MOTOR_BASE_ADDR;
  static const uint8_t SCRIPT_PAGE_START = 0x10;
  static const uint8_t SCRIPT_PAGE_END = SCRIPT_PAGE_START * (Moic::NUM_SCRIPT_PAGES + 1);
  // Constraint: Must be less than 0x10
  static const uint8_t MAX_MOTORS = 8;

  // Writable: 0x01 (sysFlags), 0x04 (PageReg)
  static const uint8_t DEVICE_WRITE_MASK = 0x12;

  enum SysFlagBit : uint8_t {
    BitPerfEnabled = 4,
  };
#pragma pack(push, 1)
  struct DeviceMap {
    volatile uint8_t repCode;   // 0x00
    volatile uint8_t sysFlags;  // 0x01
    volatile uint16_t bootId;   // 0x02
    uint8_t _pad[4];
  }; 
  struct PerfData {
    volatile uint32_t count;       // Total samples
    volatile uint32_t max_jitter;  // µs
    volatile float avg_jitter;  // µs
    volatile float stdev_jitter;   // µs
  };
#pragma pack(pop)
  // Constraint: Must be greater than TOTAL_BLOCK_SIZE
  static const uint8_t PERF_BLOCK_START = 0xD0;
  static const uint8_t PERF_BLOCK_END = PERF_BLOCK_START + sizeof(PerfData);

  I2CMotors(TwoWire& wire, Moic::ManagedMotor** mms, uint8_t count);

  void setBootId(uint16_t id);

  void handleRead();
  void handleWrite(int howMany);

  const uint8_t numMotors;

  void observeDelta(const uint32_t delta);
  bool updatePerf();
private:
  JitterMonitor jitmon;
  bool perfOn = false;
  volatile DeviceMap regs;
  volatile PerfData perf;
  Moic::ManagedMotor** mms;
  TwoWire& wire;
  volatile uint8_t currentPage = 0;
  volatile uint8_t ptr = 0;
  uint8_t read(const uint8_t page, const uint8_t ptr);
  uint8_t write(const uint8_t ptr, const uint8_t incoming);
  static bool isWriteable(const uint8_t page, const uint8_t ptr);
  static uint8_t getMidx(const uint8_t page, const uint8_t ptr);
  static uint8_t getStructOffset(const uint8_t ptr);
};

#endif
