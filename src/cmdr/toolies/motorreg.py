from __future__ import annotations

import struct
from components.motors import Motor

def generate_cpp_masks():
  write_regs = []
  busy_regs = []

  # 1. Inspect ATTRMAP (Attributes/Telemetry)
  for name, attr in Motor.ATTRMAP.items():
    # Only process motor-space registers (non-negative src) that are writeable
    if attr.src is not None and attr.src >= 0 and attr.writeable:
      byte_len = struct.calcsize(attr.fmt)
      reg_info = (attr.name, attr.src, byte_len)
      write_regs.append(reg_info)
      
      # Usually, most writeable attributes (pos, speed, accel) 
      # should be blocked if the motor is already moving.
      # You can exclude specific ones here if needed.
      busy_regs.append(reg_info)

  # 2. Inspect ACTMAP (Actions/Triggers)
  for name, act in Motor.ACTMAP.items():
    # act is (name, reg_addr, fmt_string)
    # Note: 'x' is 1 byte, 'l' is 4 bytes
    reg_addr = act[1]
    fmt = act[2]
      
    if isinstance(reg_addr, int) and reg_addr >= 0:
      byte_len = struct.calcsize(fmt) if fmt else 1
      if fmt == 'x':
        byte_len = 1 # struct.calcsize('x') is 1
      
      reg_info = (name, reg_addr, byte_len)
      write_regs.append(reg_info)
      
      # Actions like 'move' are busy-protected, but 'stop' is NOT
      if name != 'stop':
        busy_regs.append(reg_info)

  def build_mask(regs):
    mask = 0
    for name, start, length in regs:
      for i in range(max(1, length)):
        mask |= (1 << (start + i))
    return mask

  w_mask = build_mask(write_regs)
  b_mask = build_mask(busy_regs)

  print(f"// Automatically Generated from {Motor.__name__}")
  print(f"const uint64_t MOTOR_WRITE_MASK = 0x{w_mask:012X}ULL;")
  print(f"const uint64_t MOTOR_BUSY_MASK  = 0x{b_mask:012X}ULL;")
  
  print("\n// Writeable Registry Audit:")
  for name, start, length in sorted(write_regs, key=lambda x: x[1]):
      print(f"// {name:18} | offset: 0x{start:02x} | span: {length} bytes")

# Usage:
# from components.motors import Motor
# generate_cpp_masks(Motor)
