from __future__ import annotations

import struct
from collections import namedtuple

try:
  from typing import Any, Callable
except ImportError:
  pass

class Attribute(namedtuple('Attribute', ('offset', 'fmt', 'size'))):
  offset: int
  fmt: bytes
  size: int

  @property
  def name(self) -> str:
    return revoffs[self.offset]

class Flag(namedtuple('Flag', ('bit', 'attr'))):
  bit: int
  attr: Attribute

class Attributes:
  state_flags = Attribute(0x00, b'B', 1)#
  script_repcode = Attribute(0x03, b'B', 1)#
  current_position = Attribute(0x04, b'l', 4)
  target_position = Attribute(0x08, b'l', 4)#
  speed = Attribute(0x0C, b'f', 4)#
  settings_flags = Attribute(0x10, b'B', 1)
  enable_delay_ms = Attribute(0x11, b'B', 1)
  sleep_timeout_ms = Attribute(0x12, b'H', 2)
  max_speed = Attribute(0x14, b'f', 4)
  acceleration = Attribute(0x18, b'f', 4)
  move = Attribute(0x1C, b'l', 4)
  move_to = Attribute(0x20, b'l', 4)
  delay = Attribute(0x24, b'L', 4)
  stop = Attribute(0x28, b'x', 1)
  script_clear = Attribute(0x29, b'B', 1)#
  script_exec = Attribute(0x2A, b'2B', 2)#
  script_page = Attribute(0x2C, b'B', 1)#
  script_index = Attribute(0x2D, b'B', 1)#
  call = Attribute(0x2E, b'2B', 2)
  cond_call = Attribute(0x30, b'4B', 4)
  cond_jump = Attribute(0x34, b'4B', 4)
  jump = Attribute(0x38, b'2B', 2)
  move_rev = Attribute(0x3C, b'l', 4)

attrsmap: dict[str, Attribute] = {
  name: attr for (name, attr) in
  ((name, getattr(Attributes, name)) for name in dir(Attributes))
  if isinstance(attr, Attribute)}

revoffs: dict[int, str] = {attr.offset: name for name, attr in attrsmap.items()}

revops: dict[int, Attribute] = {
  key: attrsmap[revoffs[key]] for key in
  set(revoffs).difference((0x00, 0x03, 0x08, 0x0C, 0x29, 0x2A, 0x2C, 0x2D))}

opsmap: dict[str, Attribute] = {attr.name: attr for attr in revops.values()}

class Flags:
  is_limit_cw = Flag(0, Attributes.state_flags)
  is_limit_acw = Flag(1, Attributes.state_flags)
  is_active = Flag(2, Attributes.state_flags)
  is_moving = Flag(3, Attributes.state_flags)
  is_stopping = Flag(4, Attributes.state_flags)
  is_manual_position = Flag(5, Attributes.state_flags)
  is_script_active = Flag(6, Attributes.state_flags)
  is_delay_active = Flag(7, Attributes.state_flags)
  limits_enabled = Flag(0, Attributes.settings_flags)
  sleep_enabled = Flag(1, Attributes.settings_flags)

class FunId:
  AND_STATEFLAGS_RHS = Attributes.state_flags.offset
  EQL_RETURNCODE_RHS = Attributes.script_repcode.offset
  AND_SETTINGSFLAGS_RHS = Attributes.settings_flags.offset
  EQL_LASTCONDARG_RHS = Attributes.cond_call.offset
  AND_LASTCONDARG_RHS = Attributes.cond_call.offset + 1
  EQL_CALLARG_RHS = Attributes.call.offset
  AND_CALLARG_RHS = Attributes.call.offset + 1
  ALWAYS_TRUE = 0xFF >> 1

class Return:
  OK = 0x00
  USR1 = 0xFA
  USR2 = 0xFB
  USR3 = 0xFC
  USR4 = 0xFD
  USR5 = 0xFE
  UNSET = 0xFF

class Condition(namedtuple('Condition', ('func', 'rhs'))):
  func: int
  rhs: int

  def __invert__(self):
    return type(self)(self.func ^ 0x80, self.rhs)

  @classmethod
  def forflag(cls, name: str, negate: bool = False):
    flag: Flag = getattr(Flags, name)
    return cls(flag.attr.offset ^ (bool(negate) << 7), 1 << flag.bit)

  @classmethod
  def argeql(cls, rhs: int, negate: bool = False):
    return cls(FunId.EQL_CALLARG_RHS ^ (bool(negate) << 7), rhs)

  @classmethod
  def argand(cls, rhs: int, negate: bool = False):
    return cls(FunId.AND_CALLARG_RHS ^ (bool(negate) << 7), rhs)

  @classmethod
  def repeql(cls, rhs: int, negate: bool = False):
    return cls(FunId.EQL_RETURNCODE_RHS ^ (bool(negate) << 7), rhs)

class Script:
  def __init__(self) -> None:
    self.instructions: list[tuple[str, tuple[Any, ...]]|bytes] = []
    self.labels: dict[str, int] = {}
    self._size = 0

  @property
  def size(self) -> int:
    return self._size

  def label(self, name: str) -> int:
    "Mark the NEXT instruction with this name."
    if name in self.labels:
      raise ValueError(f'Duplicate label {name!r}')
    return self.labels.setdefault(name, self.size)

  def add(self, op: str|bytes|int, *args) -> None:
    if not args:
      if op == 'end':
        op = 0xFF
      if isinstance(op, int):
        op = op.to_bytes(1, 'little')
      if isinstance(op, bytes):
        self.instructions.append(op)
        self._size += len(op)
    else:
      self.instructions.append((op, args))
      if isinstance(op, int):
        if op & 0x80:
          oplen = 2
        elif op & 0x40:
          oplen = 1
        else:
          oplen = revops[op].size
      else:
        if op.endswith('**'):
          oplen = 2
        elif op.endswith('*'):
          oplen = 1
        else:
          oplen = opsmap[op].size
      self._size += 1 + oplen

  def compile(self, resolver: Callable|None = None) -> bytes:
    "Resolve labels and pack bytes."
    buf = bytearray()
    for instruction in self.instructions:
      if isinstance(instruction, bytes):
        buf.extend(instruction)
        continue
      op, args = instruction
      if resolver:
        args = map(resolver, args)
      args = map(self.resolve, args)
      if isinstance(op, int):
        if op & 0x80:
          fmt = b'BB'
        elif op & 0x40:
          fmt = b'B'
        else:
          fmt = revops[op].fmt
        opcode = op
      else:
        if op.endswith('**'):
          fmt = b'BB'
          opcode = opsmap[op[:-2]].offset | 0x80
        elif op.endswith('*'):
          fmt = b'B'
          opcode = opsmap[op[:-1]].offset | 0x40
        else:
          attr = opsmap[op]
          fmt = attr.fmt
          opcode = attr.offset
      buf.append(opcode)
      buf.extend(struct.pack(b'<'+fmt, *args))
    buf.append(0xFF)
    return bytes(buf)

  def resolve(self, arg: str|int|float) -> int|float:
    if isinstance(arg, str):
      # Resolve Label strings to absolute byte offsets
      return self.labels[arg]
    return arg

  def if_flag(self, name: str, negate: bool = False):
    return If(self, Condition.forflag(name, negate=negate))

  def if_argeql(self, rhs: int, negate: bool = False):
    return If(self, Condition.argeql(rhs, negate=negate))

  def if_argand(self, rhs: int, negate: bool = False):
    return If(self, Condition.argand(rhs, negate=negate))

  def if_repeql(self, rhs: int, negate: bool = False):
    return If(self, Condition.repeql(rhs, negate=negate))

  def while_flag(self, name: str, negate: bool = False):
    return While(self, Condition.forflag(name, negate=negate))

class If:
  def __init__(self, script: Script, condition: Condition) -> None:
    self.script = script
    self.condition = condition
    self.id = len(script.instructions)
    self.else_label = f'if_else_{self.id}'
    self.exit_label = f'if_exit_{self.id}'
    self._has_run_else = False

  def __enter__(self):
    # If NOT condition, jump to the ELSE marker
    self.script.add(Attributes.cond_jump.offset, *~self.condition, 0xFF, self.else_label)
    return self

  def else_(self):
    "Transition point between True and False branches."
    self._has_run_else = True
    # True block is finished; jump over the upcoming Else block
    self.script.add(Attributes.jump.offset, 0xFF, self.exit_label)
    # Mark where the Else block starts
    self.script.label(self.else_label)

  def __exit__(self, exc_type, exc_val, exc_tb):
    # If we didn't have an else block, the jump-target is just 'here'
    if not self._has_run_else:
      self.script.label(self.else_label)
    self.script.label(self.exit_label)

class While:
  def __init__(self, script: Script, condition: Condition) -> None:
    self.script = script
    self.condition = condition
    self.id = len(script.instructions)
    self.start_label = f'loop_s_{self.id}'
    self.exit_label = f'loop_e_{self.id}'

  def __enter__(self):
    self.script.label(self.start_label)
    # 'If NOT (condition), Jump to Exit'
    self.script.add(Attributes.cond_jump.offset, *~self.condition, 0xFF, self.exit_label)
    return self

  def __exit__(self, exc_type, exc_val, exc_tb):
    self.script.add(Attributes.jump.offset, 0xFF, self.start_label)
    self.script.label(self.exit_label)

