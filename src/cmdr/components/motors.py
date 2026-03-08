from __future__ import annotations

from collections import OrderedDict

import busio

from . import Component, DeviceComponent
from utils import debug, Pkr

__all__ = (
  'MotorController',
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

class MotorController(DeviceComponent):
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

  def __init__(self, i2c: busio.I2C, address: int = 0x9, *, refresh_interval: int = 500, motors: int = 0) -> None:
    super().__init__(i2c=i2c, address=address)
    self.refresh_interval = refresh_interval
    self.motors: tuple[Motor, ...] = tuple(
      Motor(self, i + 1) for i in range(motors))
    self.packed = b''.join(m.packed for m in self.motors)

  def subcomponents(self):
    return self.motors

  def refresh(self) -> bool:
    a = self.packed
    moving = False
    for m in self.motors:
      m.read('state_flags')
      moving = moving or m['is_moving']
    for m in self.motors:
      if moving:
        m.read('position')
        m.read('target_position')
      else:
        for name in m.ATTRMAP:
          if name != 'state_flags':
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

  def debug_lines(self) -> Generator[str]:
    yield from super().debug_lines()
    for m in self.motors:
      yield ''
      for line in m.debug_lines():
        yield f'[M{m.id}] {line}'
      yield ''

class Motor(Component):
  PKR = Pkr('>')
  ATTRMAP = OrderedDict((x[0], x) for x in (
    ('state_flags', C1_FLAG1, PKR.size, PKR.add('H') and PKR.size),
    ('settings_flags', C1_FLAG2, PKR.size, PKR.add('B') and PKR.size),
    ('position', C1_POSITION, PKR.size, PKR.add('L') and PKR.size),
    ('max_speed', C1_MAX_SPEED, PKR.size, PKR.add('L') and PKR.size),
    ('acceleration', C1_ACCELERATION, PKR.size, PKR.add('L') and PKR.size),
    ('millisteps_per_degree', C1_MILLISTEPS_PER_DEGREE, PKR.size, PKR.add('L') and PKR.size),
    ('max_degrees', C1_MAX_DEGREES, PKR.size, PKR.add('L') and PKR.size),
    ('default_speed', C1_DEFAULT_SPEED, PKR.size, PKR.add('L') and PKR.size),
    ('homing_speed', C1_HOMING_SPEED, PKR.size, PKR.add('L') and PKR.size),
    ('abs_max_speed', C1_ABS_MAX_SPEED, PKR.size, PKR.add('L') and PKR.size),
    ('max_acceleration', C1_MAX_ACCELERATION, PKR.size, PKR.add('L') and PKR.size),
    ('position_max', C1_POSITION_MAX, PKR.size, PKR.add('L') and PKR.size),
    ('target_position', C1_TARGET_POSITION, PKR.size, PKR.add('L') and PKR.size),
  ))
  FLAGMAP = OrderedDict((x[0], x) for x in (
    ('is_limit_cw', 'state_flags', 0x0, 0x1),
    ('is_limit_acw', 'state_flags', 0x1, 0x1),
    ('is_active', 'state_flags', 0x2, 0x1),
    ('is_moving', 'state_flags', 0x3, 0x1),
    ('has_homed', 'state_flags', 0x4, 0x1),
    ('is_homing', 'state_flags', 0x5, 0x1),
    ('is_ending', 'state_flags', 0x6, 0x1),
    ('is_force_stop', 'state_flags', 0x7, 0x1),
    ('is_stopping', 'state_flags', 0x8, 0x1),
    ('is_forwarding', 'state_flags', 0x9, 0x1),
    ('is_backing', 'state_flags', 0xa, 0x1),
    ('limits_enabled', 'settings_flags', 0x0, 0x1),
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

  def __init__(self, mc: MotorController, id: int) -> None:
    if not 1 <= id <= 4:
      raise ValueError(f'{id=}')
    self.mc = mc
    self.id = id
    self.packed = bytearray(self.PKR.size)
    self.idmask = self.id - 1 << LSHIFT_MOTORIDX

  def __getitem__(self, name: str):
    if name in self.FLAGMAP:
      flagdef = self.FLAGMAP[name]
      return (self[flagdef[1]] >> flagdef[2]) & flagdef[3]
    attrdef = self.ATTRMAP[name]
    slc = slice(attrdef[2], attrdef[3])
    value = int.from_bytes(self.packed[slc])
    if value == POS_NULL and (name == 'position' or name == 'target_position'):
      return None
    if name == 'position_max' and not value:
      return None
    return value

  def refresh_if_needed(self) -> Literal[0]:
    return 0

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
  from typing import Generator, Literal
except ImportError:
  pass