from __future__ import annotations

from collections import OrderedDict

import busio

from . import Component, I2CMixin, RefreshMixin

__all__ = (
  'Controller',
  'Motor',
)

LSHIFT_CATEGORY = 0x06
LSHIFT_MOTORIDX = 0x04

C1_MASK = 0x1 << LSHIFT_CATEGORY
C2_MASK = 0x2 << LSHIFT_CATEGORY
C3_MASK = 0x3 << LSHIFT_CATEGORY

C1_FLAG1 = C1_MASK | 0x00
C1_FLAG2 = C1_MASK | 0x01

C1_POSITION = C1_MASK | 0x02
C1_MAX_SPEED = C1_MASK | 0x03
C1_ACCELERATION = C1_MASK | 0x04
C1_MILLISTEPS_PER_DEGREE = C1_MASK | 0x05
C1_MAX_DEGREES = C1_MASK | 0x06
C1_DEFAULT_SPEED = C1_MASK | 0x07
C1_HOMING_SPEED = C1_MASK | 0x08
C1_ABS_MAX_SPEED = C1_MASK | 0x09
C1_MAX_ACCELERATION = C1_MASK | 0x0a
C1_POSITION_MAX = C1_MASK | 0x0b
C1_TARGET_POSITION = C1_MASK | 0x0c

C2_STOP = C2_MASK | 0x00
C2_HOME = C2_MASK | 0x01
C2_END = C2_MASK | 0x02
C2_LIMITS_ON = C2_MASK | 0x03
C2_LIMITS_OFF = C2_MASK | 0x04

C2_MOVE_TO = C2_MASK | 0x07
C2_MOVE_CW = C2_MASK | 0x08
C2_MOVE_ACW = C2_MASK | 0x09
C2_SET_MAX_SPEED = C2_MASK | 0x0a
C2_SET_ACCELERATION = C2_MASK | 0x0b
C2_SET_HOMING_SPEED = C2_MASK | 0x0c

C2_MOVE_TO_AT_SPEED = C2_MASK | 0x0d
C2_MOVE_CW_AT_SPEED = C2_MASK | 0x0e
C2_MOVE_ACW_AT_SPEED = C2_MASK | 0x0f

C3_STOP_ALL = C3_MASK | 0x01
C3_HOME_ALL = C3_MASK | 0x02
C3_END_ALL = C3_MASK | 0x03
C3_LIMITS_ON_ALL = C3_MASK | 0x04
C3_LIMITS_OFF_ALL = C3_MASK | 0x05

C3_MOVE_MANY_NO_TIMING = C3_MASK | 0x20
C3_MOVE_MANY_TIMING = C3_MASK | 0x21
C3_MOVE_MANY_TO_NO_TIMING = C3_MASK | 0x22
C3_MOVE_MANY_TO_TIMING = C3_MASK | 0x23

CODE_OK = 0x00
CODE_MOTOR_BUSY = 0x1f
CODE_MALFORMED_COMMAND = 0x28
CODE_UNKNOWN_COMMAND = 0x2c
CODE_INVALID_MOTORID = 0x2d
CODE_COMMAND_IGNORED = 0x2e

POS_NULL = 10_000_000

class Controller(I2CMixin, RefreshMixin, Component):
  ACTMAP = OrderedDict((x[0], x) for x in (
    ('stop_all', C3_STOP_ALL, 0),
    ('home_all', C3_HOME_ALL, 0),
    ('end_all', C3_END_ALL, 0),
    ('limits_on_all', C3_LIMITS_ON_ALL, 0),
    ('limits_off_all', C3_LIMITS_OFF_ALL, 0),
    ('move_many_no_timing', C3_MOVE_MANY_NO_TIMING, 'M'),
    ('move_many_timing', C3_MOVE_MANY_TIMING, 'M'),
    ('move_many_to_no_timing', C3_MOVE_MANY_TO_NO_TIMING, 'M'),
    ('move_many_to_timing', C3_MOVE_MANY_TO_TIMING, 'M'),
  ))

  def __init__(self, i2c: busio.I2C, address: int, *, refresh_interval: int = 1000, motors: int = 0) -> None:
    super().__init__(i2c=i2c, address=address)
    self.refresh_interval = refresh_interval
    self.motors: tuple[Motor, ...] = tuple(
      Motor(self, i + 1) for i in range(motors))
    self.packed = b''.join(m.packed for m in self.motors)

  def refresh(self) -> bool:
    a = self.packed
    moving = False
    for m in self.motors:
      m.read('flag1')
      moving = moving or m['is_moving']
    for m in self.motors:
      if moving:
        m.read('position')
        m.read('target_position')
      else:
        for name in m.ATTRMAP:
          if name != 'flag1':
            m.read(name)
    self.packed = b''.join(m.packed for m in self.motors)
    return a != self.packed

  def write(self, name: str, flag: int|None = None, values: tuple[int, ...]|None = None) -> int:
    actdef = self.ACTMAP[name]
    reg = actdef[1]
    if actdef[2] == 'M':
      check_byte(flag)
      if not 2 <= len(values) <= 4:
        raise ValueError(f'{values=}')
      bufw = bytearray(2 + 4 * len(values))
      bufw[1] = flag
      i = 2
      for v in values:
        check_long(v)
        bufw[i:i+4] = v.to_bytes(4)
        i += 4
    else:
      if flag is not None:
        raise ValueError(f'{flag=}')
      if values is not None:
        raise ValueError(f'{values=}')
      bufw = bytearray(1)
    bufw[0] = reg
    bufr = bytearray(1)
    with self.device as device:
      device.write_then_readinto(bufw, bufr)
    return int.from_bytes(bufr)

class Motor(Component):
  PACKSIZE = 46
  ATTRMAP = OrderedDict((x[0], x) for x in (
    ('flag1', C1_FLAG1, 0, 1),
    ('flag2', C1_FLAG2, 1, 2),
    ('position', C1_POSITION, 2, 6),
    ('max_speed', C1_MAX_SPEED, 6, 10),
    ('acceleration', C1_ACCELERATION, 10, 14),
    ('millisteps_per_degree', C1_MILLISTEPS_PER_DEGREE, 14, 18),
    ('max_degrees', C1_MAX_DEGREES, 18, 22),
    ('default_speed', C1_DEFAULT_SPEED, 22, 26),
    ('homing_speed', C1_HOMING_SPEED, 26, 30),
    ('abs_max_speed', C1_ABS_MAX_SPEED, 30, 34),
    ('max_acceleration', C1_MAX_ACCELERATION, 34, 38),
    ('position_max', C1_POSITION_MAX, 38, 42),
    ('target_position', C1_TARGET_POSITION, 42, 46),
  ))
  FLAGMAP = OrderedDict((x[0], x) for x in (
    ('is_limit_cw', 'flag1', 0x0),
    ('is_limit_acw', 'flag1', 0x1),
    ('is_moving', 'flag1', 0x2),
    ('is_active', 'flag1', 0x3),
    ('has_homed', 'flag1', 0x4),
    ('limits_enabled', 'flag1', 0x5),
    ('is_homing', 'flag1', 0x6),
    ('is_ending', 'flag1', 0x7),
  ))
  ACTMAP = OrderedDict((x[0], x) for x in (
    ('stop', C2_STOP, 0),
    ('home', C2_HOME, 0),
    ('end', C2_END, 0),
    ('limits_on', C2_LIMITS_ON, 0),
    ('limits_off', C2_LIMITS_OFF, 0),
    ('move_to', C2_MOVE_TO, 1),
    ('move_cw', C2_MOVE_CW, 1),
    ('move_acw', C2_MOVE_ACW, 1),
    ('set_max_speed', C2_SET_MAX_SPEED, 1),
    ('set_acceleration', C2_SET_ACCELERATION, 1),
    ('set_homing_speed', C2_SET_HOMING_SPEED, 1),
    ('move_to_at_speed', C2_MOVE_TO_AT_SPEED, 2),
    ('move_cw_at_speed', C2_MOVE_CW_AT_SPEED, 2),
    ('move_acw_at_speed', C2_MOVE_ACW_AT_SPEED, 2),
  ))

  @property
  def component_address(self) -> int:
    return self.mc.component_address | self.id

  def __init__(self, mc: Controller, id: int) -> None:
    if not 1 <= id <= 4:
      raise ValueError(f'{id=}')
    self.mc = mc
    self.id = id
    self.packed = bytearray(self.PACKSIZE)
    self.idmask = self.id - 1 << LSHIFT_MOTORIDX

  def __getitem__(self, name: str):
    if name in self.FLAGMAP:
      flagdef = self.FLAGMAP[name]
      return bool((self[flagdef[1]] >> flagdef[2]) & 1)
    attrdef = self.ATTRMAP[name]
    slc = slice(attrdef[2], attrdef[3])
    value = int.from_bytes(self.packed[slc])
    if value == POS_NULL and (name == 'position' or name == 'target_position'):
      return None
    if name == 'position_max' and not value:
      return None
    return value

  def items(self) -> Generator[tuple[str, int]]:
    return (
      (name, self[name])
      for names in (self.FLAGMAP, self.ATTRMAP)
        for name in names)

  def readall(self) -> None:
    for name in self.ATTRMAP:
      self.read(name)

  def read(self, name: str) -> None:
    attrdef = self.ATTRMAP[name]
    reg = attrdef[1] | self.idmask
    with self.mc.device as device:
      device.write_then_readinto(
        reg.to_bytes(1),
        self.packed,
        out_end=1,
        in_start=attrdef[2],
        in_end=attrdef[3])

  def write(self, name: str, p1: int|None = None, p2: int|None = None) -> int:
    actdef = self.ACTMAP[name]
    reg = actdef[1] | self.idmask
    bufw = bytearray(1 + 4 * actdef[2])
    bufw[0] = reg
    if actdef[2]:
      check_long(p1)
      bufw[1:5] = p1.to_bytes(4)
      if actdef[2] > 1:
        check_long(p2)
        bufw[5:9] = p2.to_bytes(4)
    bufr = bytearray(1)
    with self.mc.device as device:
      device.write_then_readinto(bufw, bufr)
    return int.from_bytes(bufr)

def check_long(value: int) -> None:
  if not (isinstance(value, int) and value >= 0 and value.bit_length() <= 0x20):
    raise ValueError(f'{value=}')

def check_byte(value: int) -> None:
  if not (isinstance(value, int) and value >= 0 and value.bit_length() <= 0x08):
    raise ValueError(f'{value=}')

try:
  from typing import Generator
except ImportError:
  pass