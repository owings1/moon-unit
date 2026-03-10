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

C1_STATE_FLAGS = C1_MASK | 0x00
C1_SETTINGS_FLAGS = C1_MASK | 0x01

C1_POSITION = C1_MASK | 0x02
C1_MAX_SPEED = C1_MASK | 0x03
C1_ACCELERATION = C1_MASK | 0x04
C1_ABS_MAX_SPEED = C1_MASK | 0x09
C1_MAX_ACCELERATION = C1_MASK | 0x0a
C1_TARGET_POSITION = C1_MASK | 0x0c

C2_STOP = C2_MASK | 0x00
C2_MOVE_CW = C2_MASK | 0x08
C2_MOVE_ACW = C2_MASK | 0x09

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
    ('stop_all', '_act_stop_all', ''),
    # ('home_all', C3_HOME_ALL, 0),
    # ('end_all', C3_END_ALL, 0),
    # ('limits_on_all', C3_LIMITS_ON_ALL, 0),
    # ('limits_off_all', C3_LIMITS_OFF_ALL, 0),
    # ('move_many_no_timing', C3_MOVE_MANY_NO_TIMING, 'M'),
    # ('move_many_timing', C3_MOVE_MANY_TIMING, 'M'),
    # ('move_many_to_no_timing', C3_MOVE_MANY_TO_NO_TIMING, 'M'),
    # ('move_many_to_timing', C3_MOVE_MANY_TO_TIMING, 'M'),
  ))
  routine: MotorControllerRoutine|None = None

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
    self.packed = bytearray(0)

  def subcomponents(self):
    return self.motors

  def refresh(self) -> bool:
    return False

  def write(self, name: str, *v) -> int:
    actdef = self.ACTMAP[name]
    if isinstance(actdef[1], str):
      if actdef[2]:
        v = struct.unpack(actdef[2], struct.pack(actdef[2], *v))
      if actdef[1].startswith('_act'):
        return getattr(self, actdef[1])(*v)
    return CODE_UNKNOWN_COMMAND

  def _act_stop_all(self):
    for m in self.motors:
      m.write('stop')
    return CODE_OK

class MotorControllerRoutine:
  ...

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
  ATTRMAP['state_flags'] = mattr(PKR, 'state_flags', C1_STATE_FLAGS, 'B', False)
  ATTRMAP['settings_flags'] = mattr(PKR, 'settings_flags', C1_SETTINGS_FLAGS, 'B', True)
  ATTRMAP['position'] = mattr(PKR, 'position', C1_POSITION, 'l', True)
  ATTRMAP['max_speed'] = mattr(PKR, 'max_speed', C1_MAX_SPEED, 'H', True)
  ATTRMAP['acceleration'] = mattr(PKR, 'acceleration', C1_ACCELERATION, 'H', True)
  ATTRMAP['backing_steps'] = mattr(PKR, 'backing_steps', None, 'H', True)
  ATTRMAP['max_steps'] = mattr(PKR, 'max_steps', None, 'L', True)
  ATTRMAP['default_speed'] = mattr(PKR, 'default_speed', None, 'H', True)
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
    ('is_stopping', 'state_flags', 0x4, 0x1),
    ('is_manual_position', 'state_flags', 0x5, 0x1),
    ('limits_enabled', 'settings_flags', 0x0, 0x1),
  ))
  ACTMAP = OrderedDict((x[0], x) for x in (
    ('stop', C2_STOP, ''),
    ('home', '_routine_home', ''),
    ('end', '_routine_end', ''),
    ('limits_on', '_act_limits_on', ''),
    ('limits_off', '_act_limits_off', ''),
    # ('move_to', C2_MOVE_TO, 'L'),
    ('move_cw', C2_MOVE_CW, 'L'),
    ('move_acw', C2_MOVE_ACW, 'L'),
    # ('move_to_at_speed', C2_MOVE_TO_AT_SPEED, 2),
    ('move_cw_at_speed', '_routine_move_cw_at_speed', 'HL'),
    ('move_acw_at_speed', '_routine_move_acw_at_speed', 'HL'),
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
    self.write('default_speed', 2000)
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
        self.routine.next()
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

  def write(self, name: str, *v, unsafe: bool = False) -> int:
    if not unsafe and self.routine:
      if name == 'stop':
        self.routine.cancel()
        self.routine = None
      else:
        return CODE_MOTOR_BUSY
    if name in self.ATTRMAP:
      attrdef = self.ATTRMAP[name]
      if not attrdef.writeable:
        return CODE_READONLY_ATTRIBUTE
      if attrdef.islocal:
        fmt = self.PKR.bom + attrdef.fmt
        struct.pack_into(fmt, self.packed, attrdef.start, *v)
        return 0x0
      reg = attrdef.reg | self.idmask
      fmt = '>B' + attrdef.fmt
    else:
      actdef = self.ACTMAP[name]
      if isinstance(actdef[1], str):
        if actdef[2]:
          v = struct.unpack(actdef[2], struct.pack(actdef[2], *v))
        if actdef[1].startswith('_routine'):
          if self.routine:
            return CODE_COMMAND_IGNORED
          self.routine = getattr(self, actdef[1])(*v)
          self.refresh_next_tick = True
          return CODE_OK
        if actdef[1].startswith('_act'):
          return getattr(self, actdef[1])(*v, unsafe=unsafe)
        return CODE_UNKNOWN_COMMAND
      reg = actdef[1] | self.idmask
      fmt = '>B' + actdef[2]
    bufw = bytearray(struct.calcsize(fmt))
    struct.pack_into(fmt, bufw, 0, reg, *v)
    bufr = bytearray(1)
    with self.mc.device as device:
      device.write_then_readinto(bufw, bufr)
    return int.from_bytes(bufr)

  def _act_limits_on(self, **kw):
    flagdef = self.FLAGMAP['limits_enabled']
    return self.write(flagdef[1], self[flagdef[1]] | (1 << flagdef[2]), **kw)

  def _act_limits_off(self, **kw):
    flagdef = self.FLAGMAP['limits_enabled']
    return self.write(flagdef[1], self[flagdef[1]] & ~(1 << flagdef[2]), **kw)

  def _routine_home(self):
    return MotorHomeRoutine(self)

  def _routine_end(self):
    return MotorEndRoutine(self)

  def _routine_move_cw_at_speed(self, speed: int, steps: int):
    return MotorMoveAtSpeed(self, 'move_cw', speed, steps)

  def _routine_move_acw_at_speed(self, speed: int, steps: int):
    return MotorMoveAtSpeed(self, 'move_acw', speed, steps)

class MotorRoutine:
  status_text = ''
  error = False

  def __init__(self, motor: Motor, it: Generator[None]):
    self.motor = motor
    self.it = it
    self.status_text = 'Initializing'

  def next(self):
    return next(self.it)

  def cancel(self):
    pass

  def moving(self) -> bool:
    self.motor.read('state_flags')
    return self.motor['is_moving']

  def write(self, name: str, *v) -> int:
    return self.motor.write(name, *v, unsafe=True)

class MotorMoveAtSpeed(MotorRoutine):
  old_max_speed = None

  def __init__(self, motor: Motor, move: str, speed: int, steps: int):
    self.move = move
    self.speed = speed
    self.steps = steps
    super().__init__(motor, self.gen())

  def gen(self):
    if self.moving():
      self.error = True
      self.status_text = 'Motor busy'
      return
    m = self.motor
    m.read('max_speed')
    self.old_max_speed = m['max_speed']
    try:
      if self.write('max_speed', self.speed) != 0:
        self.error = True
        self.status_text = 'Write failed: max_speed'
        return
      if self.write(self.move, self.steps) != 0:
        self.error = True
        self.status_text = f'Write failed: {self.move}'
        return
      self.status_text = 'Moving'
      yield
      while self.moving():
        yield
    finally:
      if self.old_max_speed:
        self.write('max_speed', self.old_max_speed)

class MotorHomeEndBase(MotorRoutine):
  limitflag: str
  movefwd: str
  moveback: str
  old_max_speed = None
  old_acceleration = None

  def __init__(self, motor: Motor):
    super().__init__(motor, self.gen())

  def islimit(self) -> bool:
    self.motor.read('state_flags')
    return self.motor[self.limitflag]

  def cancel(self):
    self.write('stop')
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
      if self.write('stop') != 0:
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
      if self.write('acceleration', m['max_acceleration']) != 0:
        self.error = True
        self.status_text = 'Write failed: acceleration'
        return
      if self.write('max_speed', m['homing_speed']) != 0:
        self.error = True
        self.status_text = 'Write failed: max_speed'
        return

      while not self.islimit():
        if self.write(self.movefwd, m['max_steps']) != 0:
          self.error = True
          self.status_text = f'Write failed: {self.movefwd}'
          return
        self.status_text = 'Moving'
        yield
        while self.moving():
          yield
      while self.islimit():
        if self.write(self.moveback, m['backing_steps']) != 0:
          self.error = True
          self.status_text = f'Write failed: {self.moveback}'
          return
        self.status_text = 'Backing'
        yield
        while self.moving():
          yield
      if self.write('max_speed', m['fixing_speed']) != 0:
        self.error = True
        self.status_text = 'Write failed: max_speed'
        return
      while not self.islimit():
        if self.write(self.movefwd, m['backing_steps'] * 2) != 0:
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
    if self.old_max_speed:
      self.write('max_speed', self.old_max_speed)
    if self.old_acceleration:
      self.write('acceleration', self.old_acceleration)

class MotorHomeRoutine(MotorHomeEndBase):
  limitflag: str = 'is_limit_acw'
  movefwd: str = 'move_acw'
  moveback: str = 'move_cw'

  def finish(self):
    if self.write('position', 0) != 0:
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
      if self.write('position_max', m['position']) != 0:
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