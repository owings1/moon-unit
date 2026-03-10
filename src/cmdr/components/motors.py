from __future__ import annotations

import struct
from collections import OrderedDict, namedtuple

import board
import busio
from utils import Pkr, debug

from . import Component, DeviceComponent

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
# C1_BACKING_STEPS = C1_MASK | 0x05
# C1_MAX_STEPS = C1_MASK | 0x06
C1_DEFAULT_SPEED = C1_MASK | 0x07
# C1_HOMING_SPEED = C1_MASK | 0x08
C1_ABS_MAX_SPEED = C1_MASK | 0x09
C1_MAX_ACCELERATION = C1_MASK | 0x0a
# C1_POSITION_MAX = C1_MASK | 0x0b
C1_TARGET_POSITION = C1_MASK | 0x0c

C2_STOP = C2_MASK | 0x00
# C2_HOME = C2_MASK | 0x01
# C2_END = C2_MASK | 0x02
C2_LIMITS_ON = C2_MASK | 0x03
C2_LIMITS_OFF = C2_MASK | 0x04

C2_MOVE_TO = C2_MASK | 0x07
C2_MOVE_CW = C2_MASK | 0x08
C2_MOVE_ACW = C2_MASK | 0x09

C2_MOVE_TO_AT_SPEED = C2_MASK | 0x0d
C2_MOVE_CW_AT_SPEED = C2_MASK | 0x0e
C2_MOVE_ACW_AT_SPEED = C2_MASK | 0x0f

C3_STATE_FLAGS = C3_MASK | 0x00
C3_STOP_ALL = C3_MASK | 0x01
# C3_HOME_ALL = C3_MASK | 0x02
# C3_END_ALL = C3_MASK | 0x03
# C3_LIMITS_ON_ALL = C3_MASK | 0x04
# C3_LIMITS_OFF_ALL = C3_MASK | 0x05

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
CODE_COMMAND_PARTIALLY_IGNORED = 0x2f
CODE_READONLY_ATTRIBUTE = 0x30

POS_NULL = 10_000_000

class MotorController(DeviceComponent):
  ACTMAP = OrderedDict((x[0], x) for x in (
    ('stop_all', C3_STOP_ALL, 0),
    # ('home_all', C3_HOME_ALL, 0),
    # ('end_all', C3_END_ALL, 0),
    # ('limits_on_all', C3_LIMITS_ON_ALL, 0),
    # ('limits_off_all', C3_LIMITS_OFF_ALL, 0),
    ('move_many_no_timing', C3_MOVE_MANY_NO_TIMING, 'M'),
    ('move_many_timing', C3_MOVE_MANY_TIMING, 'M'),
    ('move_many_to_no_timing', C3_MOVE_MANY_TO_NO_TIMING, 'M'),
    ('move_many_to_timing', C3_MOVE_MANY_TO_TIMING, 'M'),
  ))

  def __init__(
    self,
    i2c: busio.I2C|None = None,
    address: int = 0x9,
    refresh_interval: int = 500,
    motors: int = 0,
    motors_init: list[dict[str, int]]|None = None,
  ) -> None:
    super().__init__(i2c=i2c or board.I2C(), address=address)
    self.refresh_interval = refresh_interval
    self.motors: tuple[Motor, ...] = tuple(
      Motor(self, i + 1) for i in range(motors))
    if motors_init:
      for m, init in zip(self.motors, motors_init):
        if init:
          for k, v in init.items():
            if k == 'persist_id':
              m.persist_id = v
            else:
              m.write(k, v)
    self.moving = False
    # self.packed = b''.join(m.packed for m in self.motors)
    self.packed = bytearray(1)

  def subcomponents(self):
    return self.motors

  def refresh(self) -> bool:
    a = bytes(self.packed)
    # buf = bytearray(1)
    with self.device as device:
      device.write_then_readinto(C3_STATE_FLAGS.to_bytes(), self.packed)
    self.moving = (self.packed[0] & 1) == 1
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

  # def debug_lines(self) -> Generator[str]:
  #   yield from super().debug_lines()
  #   for m in self.motors:
  #     yield ''
  #     for line in m.debug_lines():
  #       yield f'[M{m.id}] {line}'
  #     yield ''
class MotorAttr(namedtuple('MotorAttrBase', ('name', 'reg', 'start', 'end', 'fmt', 'writeable', 'islocal'))):
  name: str
  reg: int|None
  start: int
  end: int
  fmt: str
  writeable: bool
  islocal: bool

def mattr(pkr: Pkr, name: str, reg: int|None, fmt: str, writeable: bool):
  start = pkr.size
  pkr.add(fmt)
  end = pkr.size
  return MotorAttr(
    name=name,
    reg=reg,
    start=start,
    end=end,
    fmt=fmt,
    writeable=writeable,
    islocal=reg is None)

class Motor(Component):
  PKR = Pkr('<')
  ATTRMAP: dict[str, MotorAttr] = OrderedDict()
  ATTRMAP['state_flags'] = mattr(PKR, 'state_flags', C1_FLAG1, 'H', False)
  ATTRMAP['settings_flags'] = mattr(PKR, 'settings_flags', C1_FLAG2, 'B', False)
  ATTRMAP['position'] = mattr(PKR, 'position', C1_POSITION, 'l', True)
  ATTRMAP['max_speed'] = mattr(PKR, 'max_speed', C1_MAX_SPEED, 'H', True)
  ATTRMAP['acceleration'] = mattr(PKR, 'acceleration', C1_ACCELERATION, 'H', True)
  ATTRMAP['backing_steps'] = mattr(PKR, 'backing_steps', None, 'H', True)
  ATTRMAP['max_steps'] = mattr(PKR, 'max_steps', None, 'L', True)
  ATTRMAP['default_speed'] = mattr(PKR, 'default_speed', C1_DEFAULT_SPEED, 'H', True)
  ATTRMAP['homing_speed'] = mattr(PKR, 'homing_speed', None, 'H', True)
  ATTRMAP['fixing_speed'] = mattr(PKR, 'fixing_speed', None, 'H', True)
  ATTRMAP['abs_max_speed'] = mattr(PKR, 'abs_max_speed', C1_ABS_MAX_SPEED, 'H', True)
  ATTRMAP['max_acceleration'] = mattr(PKR, 'max_acceleration', C1_MAX_ACCELERATION, 'H', True)
  ATTRMAP['position_max'] = mattr(PKR, 'position_max', None, 'L', True)
  ATTRMAP['target_position'] = mattr(PKR, 'target_position', C1_TARGET_POSITION, 'l', False)
  FLAGMAP = OrderedDict((x[0], x) for x in (
    ('is_limit_cw', 'state_flags', 0x0, 0x1),
    ('is_limit_acw', 'state_flags', 0x1, 0x1),
    ('is_active', 'state_flags', 0x2, 0x1),
    ('is_moving', 'state_flags', 0x3, 0x1),
    # ('has_homed', 'state_flags', 0x4, 0x1),
    # ('is_homing', 'state_flags', 0x5, 0x1),
    # ('is_ending', 'state_flags', 0x6, 0x1),
    ('is_force_stop', 'state_flags', 0x7, 0x1),
    ('is_stopping', 'state_flags', 0x8, 0x1),
    # ('is_forwarding', 'state_flags', 0x9, 0x1),
    # ('is_backing', 'state_flags', 0xa, 0x1),
    ('is_manual_position', 'state_flags', 0xb, 0x1),
    ('limits_enabled', 'settings_flags', 0x0, 0x1),
  ))
  ACTMAP = OrderedDict((x[0], x) for x in (
    ('stop', C2_STOP, ''),
    ('home', '_routine_home', ''),
    ('end', '_routine_end', ''),
    ('limits_on', C2_LIMITS_ON, ''),
    ('limits_off', C2_LIMITS_OFF, ''),
    ('move_to', C2_MOVE_TO, 'L'),
    ('move_cw', C2_MOVE_CW, 'L'),
    ('move_acw', C2_MOVE_ACW, 'L'),
    # ('move_to_at_speed', C2_MOVE_TO_AT_SPEED, 2),
    # ('move_cw_at_speed', C2_MOVE_CW_AT_SPEED, 2),
    # ('move_acw_at_speed', C2_MOVE_ACW_AT_SPEED, 2),
  ))
  routine: MotorRoutine|None = None

  @property
  def component_address(self) -> int:
    return self.mc.component_address | self.id

  @property
  def refresh_interval(self) -> int:
    return self.mc.refresh_interval

  def __init__(self, mc: MotorController, id: int) -> None:
    if not 1 <= id <= 4:
      raise ValueError(f'{id=}')
    self.mc = mc
    self.id = id
    self.packed = bytearray(self.PKR.size)
    self.idmask = self.id - 1 << LSHIFT_MOTORIDX
    self.write('homing_speed', 4000)
    self.write('fixing_speed', 400)

  def __getitem__(self, name: str):
    if name in self.FLAGMAP:
      flagdef = self.FLAGMAP[name]
      return (self[flagdef[1]] >> flagdef[2]) & flagdef[3]
    attrdef = self.ATTRMAP[name]
    value = struct.unpack_from(self.PKR.bom+attrdef.fmt, self.packed, attrdef.start)
    if len(value) == 1:
      value = value[0]
      if value == POS_NULL and (name == 'position' or name == 'target_position'):
        return None
      if name == 'position_max' and not value:
        return None
    return value

  def refresh(self):
    a = bytes(self.packed)
    if self.routine:
      try:
        next(self.routine)
      except StopIteration:
        self.routine = None
    for name in self.ATTRMAP:
      self.read(name)
    return a != self.packed

  def read(self, name: str) -> None:
    attrdef = self.ATTRMAP[name]
    if attrdef.islocal:
      return
    reg = attrdef.reg | self.idmask
    with self.mc.device as device:
      device.write_then_readinto(
        reg.to_bytes(1),
        self.packed,
        out_end=1,
        in_start=attrdef.start,
        in_end=attrdef.end)

  def write(self, name: str, *v) -> int:
    if name in self.ATTRMAP:
      attrdef = self.ATTRMAP[name]
      if not attrdef.writeable:
        raise ValueError(f'{name} is readonly')
      if attrdef.islocal:
        fmt = self.PKR.bom + attrdef.fmt
        struct.pack_into(fmt, self.packed, attrdef.start, *v)
        return 0x0
      reg = attrdef.reg | self.idmask
      fmt = '>B' + attrdef.fmt
    else:
      actdef = self.ACTMAP[name]
      if isinstance(actdef[1], str):
        if self.routine:
          return CODE_COMMAND_IGNORED
        self.routine = getattr(self, actdef[1])()
        return CODE_OK
      reg = actdef[1] | self.idmask
      fmt = '>B' + actdef[2]
    bufw = bytearray(struct.calcsize(fmt))
    struct.pack_into(fmt, bufw, 0, reg, *v)
    bufr = bytearray(1)
    with self.mc.device as device:
      device.write_then_readinto(bufw, bufr)
    return int.from_bytes(bufr)

  def _routine_home(self):
    return MotorHomeRoutine(self)
    # yield from self._routine_home_or_end(limitflag='is_limit_acw', movefwd='move_acw', moveback='move_cw')
    # self.write('position', 0)

  def _routine_end(self):
    return MotorEndRoutine(self)
    # yield from self._routine_home_or_end(limitflag='is_limit_cw', movefwd='move_cw', moveback='move_acw')
    # if self['is_manual_position']:
    #   self.read('position')
    #   self.write('position_max', self['position'])

  # def _routine_home_or_end(self, limitflag: str, movefwd: str, moveback: str):

  #   def islimit() -> bool:
  #     self.read('state_flags')
  #     return self[limitflag]

  #   def moving() -> bool:
  #     self.read('state_flags')
  #     return self['is_moving']

  #   self.read('settings_flags')
  #   if not self['limits_enabled']:
  #     return

  #   if moving():
  #     if self.write('stop') != 0:
  #       return
  #     yield
  #   while moving():
  #     yield
  #   self.read('max_acceleration')
  #   old_max_speed = self.read('max_speed') or self['max_speed']
  #   old_acceleration = self.read('acceleration') or self['acceleration']
  #   try:
  #     if self.write('acceleration', self['max_acceleration']) != 0:
  #       return
  #     if self.write('max_speed', self['homing_speed']) != 0:
  #       return

  #     while not islimit():
  #       if self.write(movefwd, self['max_steps']) != 0:
  #         return
  #       yield
  #       while moving():
  #         yield
  #     while islimit():
  #       if self.write(moveback, self['backing_steps']) != 0:
  #         return
  #       yield
  #       while moving():
  #         yield
  #     if self.write('max_speed', self['fixing_speed']) != 0:
  #       return
  #     while not islimit():
  #       if self.write(movefwd, self['backing_steps'] * 2) != 0:
  #         return
  #       yield
  #       while moving():
  #         yield
  #   finally:
  #     self.write('max_speed', old_max_speed)
  #     self.write('acceleration', old_acceleration)

class MotorRoutine:
  status_text = ''
  error = False

  def __init__(self, motor: Motor):
    self.motor = motor
    self.status_text = 'Initializing'

  def next(self):
    raise StopIteration

  def cancel(self):
    pass

  def moving(self) -> bool:
    self.motor.read('state_flags')
    return self.motor['is_moving']

class MotorHomeEndBase(MotorRoutine):
  limitflag: str
  movefwd: str
  moveback: str

  def __init__(self, motor: Motor):
    super().__init__(motor)
    self.old_max_speed = None
    self.old_acceleration = None
    self.it = self.gen()

  def islimit(self) -> bool:
    self.motor.read('state_flags')
    return self.motor[self.limitflag]

  def next(self):
    return next(self.it)

  def cancel(self):
    self.motor.write('stop')
    self.cleanup()
    self.error = True
    self.status_text = 'Canceled'
    self.it = iter(())

  def gen(self):
    m = self.motor
    m.read('settings_flags')
    if not m['limits_enabled']:
      self.error = True
      self.status_text = 'Limits disabled'
      return

    if self.moving():
      self.status_text = 'Stopping'
      if m.write('stop') != 0:
        self.error = True
        self.status_text = 'Stop failed'
        return
      yield
    while self.moving():
      yield
    m.read('max_acceleration')
    self.old_max_speed = m.read('max_speed') or m['max_speed']
    self.old_acceleration = m.read('acceleration') or m['acceleration']
    try:
      if m.write('acceleration', m['max_acceleration']) != 0:
        self.error = True
        self.status_text = 'Write failed: acceleration'
        return
      if m.write('max_speed', m['homing_speed']) != 0:
        self.error = True
        self.status_text = 'Write failed: max_speed'
        return

      while not self.islimit():
        if m.write(self.movefwd, m['max_steps']) != 0:
          self.error = True
          self.status_text = f'Write failed: {self.movefwd}'
          return
        self.status_text = 'Moving'
        yield
        while self.moving():
          yield
      while self.islimit():
        if m.write(self.moveback, m['backing_steps']) != 0:
          self.error = True
          self.status_text = f'Write failed: {self.moveback}'
          return
        self.status_text = 'Backing'
        yield
        while self.moving():
          yield
      if m.write('max_speed', m['fixing_speed']) != 0:
        self.error = True
        self.status_text = 'Write failed: max_speed'
        return
      while not self.islimit():
        if m.write(self.movefwd, m['backing_steps'] * 2) != 0:
          self.error = True
          self.status_text = f'Write failed: {self.movefwd}'
          return
        self.status_text = 'Fixing'
        yield
        while self.moving():
          yield
      self.finish()
    finally:
      self.cleanup()

  def finish(self):
    pass

  def cleanup(self):
    m = self.motor
    if self.old_max_speed:
      m.write('max_speed', self.old_max_speed)
    if self.old_acceleration:
      m.write('acceleration', self.old_acceleration)

class MotorHomeRoutine(MotorHomeEndBase):
  limitflag: str = 'is_limit_acw'
  movefwd: str = 'move_acw'
  moveback: str = 'move_cw'

  def finish(self):
    if self.motor.write('position', 0) != 0:
      self.error = True
      self.status_text = 'Write failed: position'
      return
    self.status_text = 'Finished'

class MotorEndRoutine(MotorHomeEndBase):
  limitflag: str = 'is_limit_cw'
  movefwd: str = 'move_cw'
  moveback: str = 'move_acw'

  def finish(self):
    m = self.motor
    if m['is_manual_position']:
      m.read('position')
      if m.write('position_max', self['position']) != 0:
        self.error = True
        self.status_text = 'Write failed: position_max'
        return
    self.status_text = 'Finished'

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