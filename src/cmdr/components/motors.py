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

LSHIFT_PAGE = 0x06
LSHIFT_MIDX = 0x04

P1_MASK = 0x1 << LSHIFT_PAGE

M_STATE_FLAGS = P1_MASK | 0x0
M_SETTINGS_FLAGS = P1_MASK | 0x1
M_POSITION = P1_MASK | 0x2
M_MAX_SPEED = P1_MASK | 0x3
M_ACCELERATION = P1_MASK | 0x4
M_MOVE_CW = P1_MASK | 0x5
M_MOVE_ACW = P1_MASK | 0x6
M_ABS_MAX_SPEED = P1_MASK | 0x9
M_MAX_ACCELERATION = P1_MASK | 0xa
M_TARGET_POSITION = P1_MASK | 0xc
M_STOP = P1_MASK | 0xf

CODE_OK = 0x00
CODE_WRITE_FAILED = 0x0b
CODE_MOTOR_BUSY = 0x1f
CODE_CANCELED = 0x20
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
  ))

  def __init__(
    self,
    i2c: busio.I2C|None = None,
    address: int = 0x9,
    refresh_interval: int = 500,
    # motors: int = 0,
    # motors_init: list[dict[str, int]]|None = None,
  ) -> None:
    super().__init__(i2c=i2c or board.I2C(), address=address)
    self.refresh_interval = refresh_interval
    # self.motors: tuple[Motor, ...] = tuple(
    #   Motor(self, i + 1) for i in range(motors))
    # if motors_init:
    #   for m, init in zip(self.motors, motors_init):
    #     if init:
    #       for k, v in init.items():
    #         if k == 'persist_id':
    #           m.persist_id = v
    #         else:
    #           m.write(k, v)
    self.packed = b''
    self.motors: list[Motor] = []

  # def subcomponents(self):
  #   return self.motors

  def app_ready(self, app):
    self.motors.sort(key=lambda m: m.id)

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

class MotorAttr(namedtuple('MotorAttrBase', ('name', 'reg', 'start', 'end', 'fmt', 'scale', 'writeable', 'islocal'))):
  name: str
  reg: int|None
  start: int
  end: int
  fmt: str
  scale: int|float
  writeable: bool
  islocal: bool

def mattr(pkr: Pkr, name: str, reg: int|None, fmt: str, writeable: bool, scale: int|float = 1):
  start = pkr.size
  pkr.add(fmt)
  end = pkr.size
  return MotorAttr(
    name=name,
    reg=reg,
    start=start,
    end=end,
    fmt=fmt,
    scale=scale,
    writeable=writeable,
    islocal=reg is None)

class Motor(Component):
  PKR = Pkr('<')
  ATTRMAP: dict[str, MotorAttr] = OrderedDict(
    state_flags=mattr(PKR, 'state_flags', M_STATE_FLAGS, 'B', False),
    settings_flags=mattr(PKR, 'settings_flags', M_SETTINGS_FLAGS, 'B', True),
    position=mattr(PKR, 'position', M_POSITION, 'l', True),
    max_speed=mattr(PKR, 'max_speed', M_MAX_SPEED, 'H', True),
    acceleration=mattr(PKR, 'acceleration', M_ACCELERATION, 'H', True),
    backing_steps=mattr(PKR, 'backing_steps', None, 'H', True),
    max_steps=mattr(PKR, 'max_steps', None, 'L', True),
    msteps_per_degree=mattr(PKR, 'msteps_per_degree', None, 'L', True),
    default_speed=mattr(PKR, 'default_speed', None, 'H', True),
    homing_speed=mattr(PKR, 'homing_speed', None, 'H', True),
    fixing_speed=mattr(PKR, 'fixing_speed', None, 'H', True),
    abs_max_speed=mattr(PKR, 'abs_max_speed', M_ABS_MAX_SPEED, 'H', True),
    max_acceleration=mattr(PKR, 'max_acceleration', M_MAX_ACCELERATION, 'H', True),
    position_max=mattr(PKR, 'position_max', None, 'L', True),
    target_position=mattr(PKR, 'target_position', M_TARGET_POSITION, 'l', False),
  )
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
  PERSIST_ATTRNAMES = tuple(ATTRMAP)[1:-1]
  PERSIST_SLC = slice(ATTRMAP[PERSIST_ATTRNAMES[0]].start, ATTRMAP[PERSIST_ATTRNAMES[-1]].end)
  PERSIST_FMT = PKR.bom + PKR.fmt[len(PKR.bom):][1:-1]
  PERSIST_NS = 0x9100
  PERSIST_VER = 0x03
  init_defaults = OrderedDict(
    default_speed=2000,
    homing_speed=4000,
    fixing_speed=400,
    msteps_per_degree=10_000)
  mc: MotorController
  routine: MotorRoutine|None = None

  @property
  def component_address(self) -> int:
    return self.controller_address | self.id

  @property
  def refresh_interval(self) -> int:
    return self.mc.refresh_interval

  def __init__(self, id: int, controller_address: int = 0x900, **init_data) -> None:
    if not 1 <= id <= 4:
      raise ValueError(f'{id=}')
    self.id = id
    self.controller_address = controller_address
    self.init_data = OrderedDict(self.init_defaults)
    self.init_data.update(init_data)
    self.packed = bytearray(self.PKR.size)
    self.idmask = self.id - 1 << LSHIFT_MIDX

  def app_init(self, app) -> None:
    self.mc = app.components[self.controller_address]
    self.mc.motors.append(self)
    for k, v in self.init_data.items():
      self.write(k, v)

  def __getitem__(self, name: str):
    if name in self.FLAGMAP:
      flagdef = self.FLAGMAP[name]
      return (self[flagdef[1]] >> flagdef[2]) & flagdef[3]
    attrdef = self.ATTRMAP[name]
    raw = struct.unpack_from(self.PKR.bom+attrdef.fmt, self.packed, attrdef.start)
    it = (x * attrdef.scale for x in raw)
    if len(raw) == 1:
      value = next(it)
      if value == POS_NULL and (name == 'position' or name == 'target_position'):
        return None
      if name == 'position_max' and not value:
        return None
      return value
    return tuple(it)

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
      if attrdef.scale != 1:
        v = tuple(round(x / attrdef.scale) for x in v)
      if attrdef.islocal:
        fmt = self.PKR.bom + attrdef.fmt
        struct.pack_into(fmt, self.packed, attrdef.start, *v)
        return CODE_OK
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
    bufw = struct.pack(fmt, reg, *v)
    bufr = bytearray(1)
    with self.mc.device as device:
      device.write_then_readinto(bufw, bufr)
    return int.from_bytes(bufr)

  def dump_persistent(self):
    return self.packed[self.PERSIST_SLC]

  def load_persistent(self, buf: bytes) -> None:
    values = struct.unpack(self.PERSIST_FMT, buf)
    if len(values) == len(self.PERSIST_ATTRNAMES):
      for name, value in zip(self.PERSIST_ATTRNAMES, values):
        self.write(name, value)

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

class WriteFailed(Exception):
  pass

class MotorRoutine:
  status_text = ''
  error = False
  errcode = None 
  canceled = False

  def __init__(self, motor: Motor, it: Generator[None]):
    self.motor = motor
    self.it = it
    self.status_text = 'Initializing'

  def __next__(self):
    try:
      return next(self.it)
    except StopIteration:
      self.cleanup()
      raise

  def __iter__(self):
    return self

  def cancel(self):
    self.error = True
    self.errcode = CODE_CANCELED
    self.canceled = True
    self.status_text = 'Canceling'
    if self.moving():
      self.write('stop')
    self.cleanup()
    self.status_text = 'Canceled'
    self.it = iter(())

  def cleanup(self):
    'Cleanup should be idempotent'
    pass

  def moving(self) -> bool:
    self.motor.read('state_flags')
    return self.motor['is_moving']

  def write(self, name: str, *v) -> int:
    return self.motor.write(name, *v, unsafe=True)

  def write_chk(self, name: str, *v) -> bool:
    if self.write(name, *v) != 0:
      self.error = True
      self.errcode = CODE_WRITE_FAILED
      self.status_text = f'Write failed: {name}'
      return False
    return True

  def write_chkraise(self, name: str, *v) -> None:
    if not self.write_chk(name, *v):
      raise WriteFailed

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
      self.errcode = CODE_MOTOR_BUSY
      self.status_text = 'Motor busy'
      return
    m = self.motor
    m.read('max_speed')
    self.old_max_speed = m['max_speed']
    try:
      self.write_chkraise('max_speed', self.speed)
      self.write_chkraise(self.move, self.steps)
      self.status_text = 'Moving'
      yield
      while self.moving():
        yield
    except WriteFailed:
      pass
    finally:
      self.cleanup()

  def cleanup(self):
    if self.old_max_speed:
      if self.write('max_speed', self.old_max_speed) == 0:
        self.old_max_speed = None

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

  def gen(self):
    m = self.motor
    m.read('settings_flags')
    if not m['limits_enabled']:
      self.error = True
      self.errcode = CODE_COMMAND_IGNORED
      self.status_text = 'Limits disabled'
      return

    if self.moving():
      self.status_text = 'Stopping'
      if not self.write_chk('stop'):
        return
      yield
    while self.moving():
      yield

    m.read('max_acceleration')
    m.read('max_speed')
    m.read('acceleration')
    m.read('max_steps')
    m.read('fixing_speed')
    self.old_max_speed = m['max_speed']
    self.old_acceleration = m['acceleration']
    try:
      self.write_chkraise('acceleration', m['max_acceleration'])
      self.write_chkraise('max_speed', m['homing_speed'])
      while not self.islimit():
        self.write_chkraise(self.movefwd, m['max_steps'])
        self.status_text = 'Moving'
        yield
        while self.moving():
          yield
      while self.islimit():
        self.write_chkraise(self.moveback, m['backing_steps'])
        self.status_text = 'Backing'
        yield
        while self.moving():
          yield
      self.write_chkraise('max_speed', m['fixing_speed'])
      while not self.islimit():
        self.write_chkraise(self.movefwd, m['backing_steps'] * 2)
        self.status_text = 'Fixing'
        yield
        while self.moving():
          yield
      self.finish()
    except WriteFailed:
      pass
    finally:
      self.cleanup()

  def finish(self):
    pass

  def cleanup(self):
    if self.old_max_speed:
      if self.write('max_speed', self.old_max_speed) == 0:
        self.old_max_speed = None
    if self.old_acceleration:
      if self.write('acceleration', self.old_acceleration) == 0:
        self.old_acceleration = None

class MotorHomeRoutine(MotorHomeEndBase):
  limitflag: str = 'is_limit_acw'
  movefwd: str = 'move_acw'
  moveback: str = 'move_cw'

  def finish(self):
    self.write_chkraise('position', 0)
    self.status_text = 'Finished'

class MotorEndRoutine(MotorHomeEndBase):
  limitflag: str = 'is_limit_cw'
  movefwd: str = 'move_cw'
  moveback: str = 'move_acw'

  def finish(self):
    m = self.motor
    if m['is_manual_position']:
      m.read('position')
      self.write_chkraise('position_max', m['position'])
    self.status_text = 'Finished'

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
        self.error = routine.error
        self.errcode = routine.errcode
        yield
      yield
    self.cleanup()
    yield

try:
  from typing import Generator, Literal, Iterable
except ImportError:
  pass