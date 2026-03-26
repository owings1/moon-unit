from __future__ import annotations

import struct
from components.motors import Motor

def generate_cpp_masks():
  write_regs = []
  busy_regs = []
  script_regs = []
  scriptok = {
    'stop',
    # 'script_index',
    'script_clear',
  }
  busyok = scriptok | {'enable_delay_ms', 'sleep_timeout_ms'}
  # 1. Inspect ATTRMAP (Attributes/Telemetry)
  for name, attr in Motor.ATTRMAP.items():
    # Only process motor-space registers (non-negative src) that are writeable
    if attr.src is not None and attr.src >= 0 and attr.src < 0x38 and attr.writeable:
      byte_len = struct.calcsize(attr.fmt)
      reg_info = (attr.name, attr.src, byte_len)
      write_regs.append(reg_info)
      if attr.name not in busyok:
        busy_regs.append(reg_info)
      if attr.name not in scriptok:
        script_regs.append(reg_info)

  # 2. Inspect ACTMAP (Actions/Triggers)
  for name, act in Motor.ACTMAP.items():
    if isinstance(act.src, int) and act.src >= 0:
      byte_len = struct.calcsize(act.fmt)# if act.fmt else 1
      # if act.fmt == 'x':
      #   byte_len = 1 # struct.calcsize('x') is 1
      reg_info = (act.name, act.src, byte_len)
      write_regs.append(reg_info)
      if act.name not in busyok:
        busy_regs.append(reg_info)
      if act.name not in scriptok:
        script_regs.append(reg_info)

  def build_mask(regs):
    mask = 0
    for name, start, length in regs:
      for i in range(max(1, length)):
        mask |= (1 << (start + i))
    return mask

  w_mask = build_mask(write_regs)
  b_mask = build_mask(busy_regs)
  s_mask = build_mask(script_regs)

  print("  // Writeable:")
  for name, start, length in sorted(write_regs, key=lambda x: x[1]):
      print(f"  // {name:18} | offset: 0x{start:02x} | span: {length} bytes")
  print(f"  static const uint64_t MOTOR_WRITE_MASK = 0x{w_mask:012X}ULL;")
  print("  // Busy Protected:")
  for name, start, length in sorted(busy_regs, key=lambda x: x[1]):
      print(f"  // {name:18} | offset: 0x{start:02x} | span: {length} bytes")
  print(f"  static const uint64_t MOTOR_BUSY_MASK = 0x{b_mask:012X}ULL;")
  print("  // Script Protected:")
  for name, start, length in sorted(script_regs, key=lambda x: x[1]):
      print(f"  // {name:18} | offset: 0x{start:02x} | span: {length} bytes")
  print(f"  static const uint64_t SCRIPT_LOCK_MASK = 0x{s_mask:012X}ULL;")
  
