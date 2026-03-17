from __future__ import annotations

import struct
import traceback
from collections import OrderedDict

import busio
from micropython import const
from utils import Pkr

from . import CompAttr, DeviceComponent

try:
  from typing import Any, Generator, Iterable
except ImportError:
  pass

__all__ = ('Motor',)

LSHIFT_PAGE = const(0x06)
LSHIFT_MIDX = const(0x04)

P1_MASK = 0x1 << LSHIFT_PAGE

M_STATE_FLAGS = P1_MASK | 0x0
M_SETTINGS_FLAGS = P1_MASK | 0x1
M_POSITION = P1_MASK | 0x2
M_MAX_SPEED = P1_MASK | 0x3
M_ACCELERATION = P1_MASK | 0x4
M_MOVE_CW = P1_MASK | 0x5
M_MOVE_ACW = P1_MASK | 0x6
M_SPEED = P1_MASK | 0x7
M_TARGET_POSITION = P1_MASK | 0xc
M_STOP = P1_MASK | 0xf

CODE_OK = const(0x00)
CODE_OTHER_ERROR = const(0x07)
CODE_WRITE_FAILED = const(0x0b)
CODE_MOTOR_BUSY = const(0x1f)
CODE_CANCELED = const(0x20)
CODE_MALFORMED_COMMAND = const(0x28)
CODE_UNKNOWN_COMMAND = const(0x2c)
CODE_INVALID_MOTORID = const(0x2d)
CODE_COMMAND_IGNORED = const(0x2e)
CODE_COMMAND_PARTIALLY_IGNORED = const(0x2f)
CODE_READONLY_ATTRIBUTE = const(0x30)

class MotorAttr(CompAttr):
  src: int|None

class Motor(DeviceComponent):
  PKR = Pkr('<')
  ATTRMAP: dict[str, MotorAttr] = MotorAttr.makeattrs(PKR, OrderedDict(
    state_flags=dict(src=M_STATE_FLAGS, fmt='B'),
    target_position=dict(src=M_TARGET_POSITION, fmt='l'),
    speed=dict(src=M_SPEED, fmt='h'),
    # --- writeable & persisted attributes
    position=dict(src=M_POSITION, fmt='l', writeable=True),
    position_max=dict(fmt='L', writeable=True),
    settings_flags=dict(src=M_SETTINGS_FLAGS, fmt='B', writeable=True),
    max_speed=dict(src=M_MAX_SPEED, fmt='H', writeable=True),
    default_speed=dict(fmt='H', writeable=True),
    homing_speed=dict(fmt='H', writeable=True),
    fixing_speed=dict(fmt='H', writeable=True),
    acceleration=dict(src=M_ACCELERATION, fmt='H', writeable=True),
    backing_steps=dict(fmt='H', writeable=True),
    msteps_per_degree=dict(fmt='L', writeable=True),
  ))
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
    ('stop', M_STOP, ''),
    ('home', '_routine_home', ''),
    ('end', '_routine_end', ''),
    ('home_end', '_routine_home_end', ''),
    ('limits_on', '_act_limits_on', ''),
    ('limits_off', '_act_limits_off', ''),
    # ('move_to', C2_MOVE_TO, 'L'),
    ('move', '_act_move', 'l'),
    ('move_cw', M_MOVE_CW, 'L'),
    ('move_acw', M_MOVE_ACW, 'L'),
    # ('move_to_at_speed', C2_MOVE_TO_AT_SPEED, 2),
    ('move_at_speed', '_routine_move_at_speed', 'Hl'),
    ('move_cw_at_speed', '_routine_move_cw_at_speed', 'HL'),
    ('move_acw_at_speed', '_routine_move_acw_at_speed', 'HL'),
  ))
  SLCINFO_PERSIST = MotorAttr.sliceinfo(ATTRMAP, 3, None)
  PERSIST_NS = const(0x9100)
  PERSIST_VER = const(0x04)
  init_defaults = OrderedDict(
    max_speed=0x7ff,
    default_speed=0x7ff,
    homing_speed=0xfff,
    fixing_speed=0x1ff,
    acceleration=0xffff,
    msteps_per_degree=0x27ff)
  routine: MotorRoutine|None = None

  @property
  def component_address(self) -> int:
    return (self.device_address << 0x8) | self.id

  def __init__(
    self,
    id: int, 
    bus: busio.I2C|None = None,
    address: int = 0x9,
    refresh_interval: int = 500,
    **init_data
  ) -> None:
    if not 1 <= id <= 4:
      raise ValueError(f'{id=}')
    super().__init__(bus, address)
    self.refresh_interval = refresh_interval
    self.id = id
    initial = OrderedDict(self.init_defaults)
    initial.update(init_data)
    self.packed = bytearray(self.PKR.size)
    self.idmask = self.id - 1 << LSHIFT_MIDX
    for k, v in initial.items():
      self.write(k, v)

  # def __getitem__(self, name: str):
  #   if name in self.FLAGMAP:
  #     flagdef = self.FLAGMAP[name]
  #     return (self[flagdef[1]] >> flagdef[2]) & flagdef[3]
  #   return self.ATTRMAP[name].unpack_from(self.packed)

  def refresh(self):
    a = bytes(self.packed)
    if self.routine:
      try:
        next(self.routine)
      except StopIteration:
        self.routine = None
    for attr in self.ATTRMAP.values():
      if attr.src is not None:
        self.read(attr.name)
    return a != self.packed

  def read(self, name: str) -> None:
    attr = self.ATTRMAP[name]
    reg = attr.src | self.idmask
    with self.device as device:
      device.write_then_readinto(
        reg.to_bytes(1),
        self.packed,
        out_end=1,
        in_start=attr.start,
        in_end=attr.end)

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
      if attrdef.src is None:
        attrdef.pack_into(self.packed, v)
        return CODE_OK
      reg = attrdef.src | self.idmask
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
    bufw = struct.pack(fmt, reg, *v)
    bufr = bytearray(1)
    with self.device as device:
      device.write_then_readinto(bufw, bufr)
    return int.from_bytes(bufr)

  def dump_persistent(self):
    return self.packed[self.SLCINFO_PERSIST.slc]

  def load_persistent(self, buf: bytes) -> None:
    slcinfo = self.SLCINFO_PERSIST
    values = struct.unpack(slcinfo.fmt, buf)
    for attr, value in zip(slcinfo.attrs, values):
      self.write(attr.name, value)

  def _act_move(self, steps: int, **kw):
    name = 'move_acw' if steps < 0 else 'move_cw'
    return self.write(name, abs(steps), **kw)

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

  def _routine_home_end(self):
    return MotorChainRoutine(self, (self._routine_home(), self._routine_end()))

  def _routine_move_at_speed(self, speed: int, steps: int):
    move = 'move_acw' if steps < 0 else 'move_cw'
    return MotorMoveAtSpeed(self, move, speed, abs(steps))

  def _routine_move_cw_at_speed(self, speed: int, steps: int):
    return MotorMoveAtSpeed(self, 'move_cw', speed, steps)

  def _routine_move_acw_at_speed(self, speed: int, steps: int):
    return MotorMoveAtSpeed(self, 'move_acw', speed, steps)

class RoutineError(Exception):
  errcode = CODE_OTHER_ERROR
  errtext = 'Routine Error'

  def __init__(self, errtext: str|None = None, code: int|None = None):
    if errtext:
      self.errtext = errtext
    if code is not None:
      self.errcode = code

class WriteFailed(Exception):
  errcode = CODE_WRITE_FAILED
  errtext = 'Write Failed'

class MotorRoutine:
  status_text = ''
  error = None
  errcode = None 
  canceled = False
  overrides: dict[str, Any]

  def __init__(self, motor: Motor, it: Generator[None]):
    self.motor = motor
    self.it = it
    self.status_text = 'Created'
    self.overrides = OrderedDict()

  def __next__(self):
    try:
      return next(self.it)
    except StopIteration:
      self.cleanup()
      raise
    except Exception as err:
      self.error = err
      traceback.print_exception(err)
      if isinstance(err, RoutineError):
        self.errcode = err.errcode
        self.status_text = f'Error: {err.errtext}'
      else:
        self.errcode = CODE_OTHER_ERROR
        self.status_text = f'Error: {err:!r}'
      self.cleanup()

  def __iter__(self):
    return self

  def cancel(self):
    self.error = RoutineError(code=CODE_CANCELED)
    self.errcode = CODE_CANCELED
    self.canceled = True
    self.status_text = 'Canceling'
    self.it = iter(())
    try:
      if self.moving():
        self.write('stop')
    finally:
      self.cleanup()
    self.status_text = 'Canceled'

  def cleanup(self):
    'Cleanup should be idempotent'
    if self.overrides:
      for key in tuple(self.overrides):
        self.write(key, self.overrides.pop(key))

  def moving(self) -> bool:
    self.motor.read('state_flags')
    return self.motor['is_moving']

  def write(self, name: str, *v, check: bool = True) -> int:
    code = self.motor.write(name, *v, unsafe=True)
    if check and code != CODE_OK:
      raise WriteFailed(name)
    return CODE_OK

  def ymove(self, *args, **kw):
    self.write(*args, **kw)
    yield
    while self.moving():
      yield

class MotorMoveAtSpeed(MotorRoutine):

  def __init__(self, motor: Motor, move: str, speed: int, steps: int):
    self.move = move
    self.speed = speed
    self.steps = steps
    super().__init__(motor, self.gen())

  def gen(self):
    if self.moving():
      raise RoutineError(code=CODE_MOTOR_BUSY)
    m = self.motor
    self.overrides['max_speed'] = m['max_speed']
    self.write('max_speed', self.speed)
    self.status_text = 'Moving'
    yield from self.ymove(self.move, self.steps)

class MotorHomeEndBase(MotorRoutine):
  limitflag: str
  movefwd: str
  moveback: str

  def __init__(self, motor: Motor):
    super().__init__(motor, self.gen())

  def islimit(self) -> bool:
    self.motor.read('state_flags')
    return self.motor[self.limitflag]

  def gen(self):
    m = self.motor
    m.read('settings_flags')
    if not m['limits_enabled']:
      raise RoutineError('Limits disabled')
    if self.moving():
      raise RoutineError(code=CODE_MOTOR_BUSY)
    self.overrides['max_speed'] = m['max_speed']
    self.write('max_speed', m['homing_speed'])
    if not self.islimit():
      self.status_text = 'Moving'
      yield from self.ymove(self.movefwd, 0x7fffffff)
    if not self.islimit():
      raise RoutineError('Cannot reach limit')
    while self.islimit():
      self.status_text = 'Backing'
      yield from self.ymove(self.moveback, m['backing_steps'])
    self.write('max_speed', m['fixing_speed'])
    while not self.islimit():
      self.status_text = 'Fixing'
      yield from self.ymove(self.movefwd, m['backing_steps'] * 2)
    self.finish()
    self.status_text = 'Finished'

  def finish(self):
    pass

class MotorHomeRoutine(MotorHomeEndBase):
  limitflag: str = 'is_limit_acw'
  movefwd: str = 'move_acw'
  moveback: str = 'move_cw'

  def finish(self):
    self.write('position', 0)

class MotorEndRoutine(MotorHomeEndBase):
  limitflag: str = 'is_limit_cw'
  movefwd: str = 'move_cw'
  moveback: str = 'move_acw'

  def finish(self):
    m = self.motor
    if m['is_manual_position']:
      m.read('position')
      self.write('position_max', m['position'])

class MotorChainRoutine(MotorRoutine):

  def __init__(self, motor: Motor, chain: Iterable[MotorRoutine]):
    self.chain = chain
    self.routine = None
    super().__init__(motor, self.gen())

  def cancel(self):
    if self.routine:
      self.routine.cancel()
    super().cancel()

  def cleanup(self):
    if self.routine:
      self.routine.cleanup()
    super().cleanup()

  def gen(self):
    for routine in self.chain:
      self.routine = routine
      for _ in routine:
        self.status_text = routine.status_text
        yield
      if routine.error:
        raise routine.error
      yield
