from __future__ import annotations

"""
from toolies import motorreg; motorreg.generate_cpp_masks()
"""

import struct
import moic

BLOCK_CUTOFF = 0x40
def generate_cpp_masks():
  write_regs = []
  busy_regs = []
  script_regs = []
  exclude = {
    'cond_call',
    'cond_jump',
    'call',
    'jump',
  }
  scriptok = {
    'stop',
    'script_clear',
  }
  busyok = scriptok | {'enable_delay_ms', 'sleep_timeout_ms'}
  nonops = {
    'script_clear',
    'script_exec',
  }
  # 1. Inspect ATTRMAP (Attributes/Telemetry)
  names = set(moic.opsmap).union(nonops).difference(exclude)
  for name in names:
    attr = moic.attrsmap[name]
    # Only process motor-space registers (non-negative src) that are writeable
    if attr.offset < BLOCK_CUTOFF:
      byte_len = struct.calcsize(attr.fmt)
      reg_info = (attr.name, attr.offset, byte_len)
      write_regs.append(reg_info)
      if attr.name not in busyok:
        busy_regs.append(reg_info)
      if attr.name not in scriptok:
        script_regs.append(reg_info)

  # # 2. Inspect ACTMAP (Actions/Triggers)
  # for name, act in Motor.ACTMAP.items():
  #   if name in exclude:
  #     continue
  #   if isinstance(act.src, int) and act.src >= 0 and act.src < BLOCK_CUTOFF:
  #     byte_len = struct.calcsize(act.fmt)# if act.fmt else 1
  #     # if act.fmt == 'x':
  #     #   byte_len = 1 # struct.calcsize('x') is 1
  #     reg_info = (act.name, act.src, byte_len)
  #     write_regs.append(reg_info)
  #     if act.name not in busyok:
  #       busy_regs.append(reg_info)
  #     if act.name not in scriptok:
  #       script_regs.append(reg_info)

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
  
