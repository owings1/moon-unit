from __future__ import annotations

import struct
import traceback
from collections import OrderedDict, deque, namedtuple

import busio
from micropython import const
from utils import Pkr

from . import CompAttr, DeviceComponent

try:
  from typing import Any, Generator, Iterable
except ImportError:
  pass

__all__ = ('Motor',)

PAGE_REGISTER = const(0x04)
CODE_OK = const(0x00)
CODE_OTHER_ERROR = const(0x07)
CODE_WRITE_FAILED = const(0x0B)
CODE_MOTOR_BUSY = const(0x1F)
CODE_CANCELED = const(0x20)
CODE_UNKNOWN_COMMAND = const(0x2C)
CODE_INVALID_MOTORID = const(0x2D)
CODE_COMMAND_IGNORED = const(0x2E)
CODE_COMMAND_PARTIALLY_IGNORED = const(0x2F)
CODE_READONLY_ATTRIBUTE = const(0x30)
CODE_OVERFLOW = const(0x31)
CODE_UNSET = const(0xFF)

class ActDef(namedtuple('ActDef', ('name', 'src', 'fmt'))):
  name: str
  src: str|int
  fmt: str

class MotorAttr(CompAttr):
  src: int|None

  def regptr(self, motor: Motor):
    if self.src is None:
      return
    if self.src < 0:
      return -1 * self.src
    return self.src + motor.regoffset

  def pagebuf(self, motor: Motor):
    if self.src is None:
      return
    if self.name[:-1] == 'script':
      return motor.script_pagebufs[int(self.name[-1])]
    return motor.pagebuf

class Motor(DeviceComponent):
  PKR = Pkr('<')
  ATTRMAP: dict[str, MotorAttr] = MotorAttr.makeattrs(PKR, OrderedDict(
    # --- Negative src indicates global device register
    boot_id=dict(src=-0x02, fmt='H'),
    state_flags=dict(src=0x00, fmt='B'),
    script_repcode=dict(src=0x03, fmt='B'),
    target_position=dict(src=0x08, fmt='l'),
    speed=dict(src=0x0C, fmt='f'),
    # --- writeable & persisted attributes
    position=dict(src=0x04, fmt='l', writeable=True),
    position_max=dict(fmt='L', writeable=True),
    settings_flags=dict(src=0x10, fmt='B', writeable=True),
    max_speed=dict(src=0x14, fmt='f', writeable=True),
    default_speed=dict(fmt='f', writeable=True),
    homing_speed=dict(fmt='f', writeable=True),
    fixing_speed=dict(fmt='f', writeable=True),
    acceleration=dict(src=0x18, fmt='f', writeable=True),
    backing_steps=dict(fmt='L', writeable=True),
    msteps_per_degree=dict(fmt='L', writeable=True),
    enable_delay_ms=dict(src=0x11, fmt='B', writeable=True),
    sleep_timeout_ms=dict(src=0x12, fmt='H', writeable=True),
    # --- not persisted
    script0=dict(fmt='248B', src=-0x08, writeable=True),
    script1=dict(fmt='248B', src=-0x08, writeable=True),
    script2=dict(fmt='248B', src=-0x08, writeable=True),
    script3=dict(fmt='248B', src=-0x08, writeable=True),
  ))
  FLAGMAP = OrderedDict((x[0], x) for x in (
    ('is_limit_cw', 'state_flags', 0x0, 0x1),
    ('is_limit_acw', 'state_flags', 0x1, 0x1),
    ('is_active', 'state_flags', 0x2, 0x1),
    ('is_moving', 'state_flags', 0x3, 0x1),
    ('is_stopping', 'state_flags', 0x4, 0x1),
    ('is_manual_position', 'state_flags', 0x5, 0x1),
    ('is_script_active', 'state_flags', 0x6, 0x1),
    ('is_delay_active', 'state_flags', 0x7, 0x1),
    ('limits_enabled', 'settings_flags', 0x0, 0x1),
    ('sleep_enabled', 'settings_flags', 0x1, 0x1),
  ))
  ACTMAP = OrderedDict((x[0], ActDef(*x)) for x in (
    ('stop', 0x28, 'x'),
    ('home', '_routine_home', ''),
    ('end', '_routine_end', ''),
    ('home_end', '_routine_home_end', ''),
    ('limits_on', '_act_limits_on', ''),
    ('limits_off', '_act_limits_off', ''),
    ('move', 0x1C, 'l'),
    ('move_to', 0x20, 'l'),
    ('move_at_speed', '_act_move_at_speed', 'fl'),
    ('move_to_at_speed', '_act_move_to_at_speed', 'fl'),
    ('delay', 0x24, 'L'),
    ('script_clear', 0x29, 'B'),
    ('script_exec', 0x2A, 'B'),
    ('call', 0x2E, '2B'),
    ('cond_call', 0x30, '4B'),
    ('cond_jump', 0x34, '4B'),
    ('jump', 0x38, '2B'),
  ))
  SLCINFO_PERSIST = MotorAttr.sliceinfo(ATTRMAP, 5, 5+12)
  PERSIST_NS = 0x9100
  PERSIST_VER = 0x08
  init_defaults = OrderedDict(
    settings_flags=0x03,
    max_speed=0x7ff,
    default_speed=0x7ff,
    homing_speed=0xfff,
    fixing_speed=0x1ff,
    acceleration=0xffff,
    msteps_per_degree=0x27ff,
    sleep_timeout_ms=2000,
    enable_delay_ms=2)
  routine: MotorRoutine|None = None

  @property
  def component_address(self) -> int:
    return (self.device_address << 0x8) | self.id

  def __init__(
    self,
    id: int, 
    bus: busio.I2C|None = None,
    address: int = 0x09,
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
    self.regoffset = (0x78 * ((self.id - 1) % 2)) + 0x08
    self.pagebuf = struct.pack(b'<2B', PAGE_REGISTER, (self.id - 1) // 2)
    self.script_pagebufs = tuple(
      struct.pack(b'<2B', PAGE_REGISTER, x + (self.id - 1))
      for x in range(0x10, 0x50, 0x10))
    self.rbuf = bytearray(2)
    for k, v in initial.items():
      self.write(k, v, fail=True)
    self.read('boot_id')
    self.last_boot_id = self['boot_id']

  def refresh(self):
    self.check_boot_id()
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
    with self.device as device:
      device.write(attr.pagebuf(self))
      device.write_then_readinto(
        attr.regptr(self).to_bytes(1, 'little'),
        self.packed,
        in_start=attr.start,
        in_end=attr.end)

  def write(self, name: str, *v, unsafe: bool = False, fail: bool = False) -> int:
    code = self._write(name, *v, unsafe=unsafe)
    if fail and code != CODE_OK:
      raise WriteFailed(f'Write Failed {name=} code={hex(code)}')
    return code

  def _write(self, name: str, *v, unsafe: bool = False) -> int:
    if not unsafe and self.routine:
      if name == 'stop':
        self.routine = self.routine.cancel()
      else:
        return CODE_MOTOR_BUSY
    if name in self.ATTRMAP:
      attr = self.ATTRMAP[name]
      if not attr.writeable:
        return CODE_READONLY_ATTRIBUTE
      if attr.src is None:
        attr.pack_into(self.packed, v)
        return CODE_OK
      pagebuf = attr.pagebuf(self)
      bufw = self._get_attr_write_buf(attr, *v)
    else:
      actdef = self.ACTMAP[name]
      if isinstance(actdef.src, str):
        if actdef.fmt:
          v = struct.unpack(actdef.fmt, struct.pack(actdef.fmt, *v))
        if actdef.src.startswith('_routine'):
          if self.routine:
            return CODE_COMMAND_IGNORED
          self.routine = getattr(self, actdef.src)(*v)
          next(self.routine)
          return CODE_OK
        if actdef.src.startswith('_act'):
          return getattr(self, actdef.src)(*v, unsafe=unsafe)
        return CODE_UNKNOWN_COMMAND
      pagebuf = self.pagebuf
      bufw = self._get_act_write_buf(actdef, *v)
    with self.device as device:
      device.write(pagebuf)
      device.write(bufw)
      # Read response code
      device.write_then_readinto(self.rbuf, self.rbuf, out_end=1, in_start=1)
    return self.rbuf[1]
    
  def _get_attr_write_buf(self, attr: MotorAttr, *v):
    reg = attr.regptr(self)
    if attr.name[:-1] == 'script':
      if not v:
        raise ValueError(f'Missing parameters for {attr.name}')
      bufw = bytearray(1)
      if len(v) == 1 and isinstance(v[0], (bytes, bytearray)):
        bufw.extend(v[0])
      else:
        bufw.extend(v)
      if len(bufw) - 1 > attr.end - attr.start:
        raise ValueError(f'Size exceeds max for {attr.name}')
    else:
      fmt = '<B' + attr.fmt
      bufw = bytearray(struct.calcsize(fmt))
      attr.pack_into(bufw, v, 1)
    bufw[0] = reg
    return bufw

  def _get_act_write_buf(self, actdef: ActDef, *v):
    reg = actdef.src + self.regoffset
    fmt = '<B' + actdef.fmt
    return struct.pack(fmt, reg, *v)
    
  def check_boot_id(self):
    self.read('boot_id')
    if self['boot_id'] != self.last_boot_id:
      print(f'Warning: boot_id changed, restoring state')
      self.load_persistent(self.dump_persistent())
      self.last_boot_id = self['boot_id']

  def dump_persistent(self):
    return self.packed[self.SLCINFO_PERSIST.slc]

  def load_persistent(self, buf: bytes, *, fail: bool = True) -> None:
    slcinfo = self.SLCINFO_PERSIST
    values = struct.unpack(slcinfo.fmt, buf)
    for attr, value in zip(slcinfo.attrs, values):
      self.write(attr.name, value, fail=fail)

  def debug_format_item(self, k: str, v: Any) -> str:
    if k[:-1] == 'script':
      if any(v):
        sq = deque((), 8)
        for x in range(8):
          it = (f'{hex(v[x*31+i])[2:]:0>2}' for i in range(31))
          sq.append(' '.join(it))
        return f'{k}=(\n{'\n'.join(sq)}\n)'
      return f'{k}=<empty>'
    return super().debug_format_item(k, v)

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

  def _act_move_at_speed(self, speed: float, steps: int, **kw) -> int:
    s = self.script_builder()
    s.add('max_speed', speed)
    s.add('move', steps)
    s.add('max_speed', self['max_speed'])
    return s.save(exec=True, fail=False, **kw)

  def _act_move_to_at_speed(self, speed: float, position: int, **kw) -> int:
    s = self.script_builder()
    s.add('max_speed', speed)
    s.add('move_to', position)
    s.add('max_speed', self['max_speed'])
    return s.save(exec=True, fail=False, **kw)

  def script_builder(self):
    return MotorScriptBuilder(self)

  @classmethod
  def script_cmdinfo(cls, name: str):
    if name in cls.ATTRMAP:
      base = cls.ATTRMAP[name]
      if not base.writeable:
        raise ValueError(f'{name} not scriptable (read only)')
    elif name in cls.ACTMAP:
      base = cls.ACTMAP[name]
    else:
      raise ValueError(f'{name=}')
    if not (isinstance(base.src, int) and base.src >= 0):
      raise ValueError(f'{name} not scriptable')
    return base.fmt, base.src

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
      try:
        self.write('stop', check=False)
      except:
        pass
      traceback.print_exception(err)
      if isinstance(err, RoutineError):
        self.errcode = err.errcode
        self.status_text = f'Error: {self.errcode and hex(self.errcode)} {err.errtext}'
      else:
        self.errcode = CODE_OTHER_ERROR
        self.status_text = f'Error: {err!r}'
      self.cleanup()

  def __iter__(self):
    return self

  def cancel(self):
    self.error = RoutineError(None, CODE_CANCELED)
    self.errcode = CODE_CANCELED
    self.canceled = True
    self.status_text = 'Canceling'
    self.it = iter(())
    if self.moving():
      self.write('stop')
      self.status_text = 'Canceling (cleanup)'
      def defered():
        while self.moving():
          yield
        self.cleanup()
      return defered()
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
      raise WriteFailed(f'{name=} {hex(code)}')
    return CODE_OK

  def ymove(self, *args, **kw):
    self.write('move', *args, **kw)
    yield
    while self.moving():
      yield

  def ymoveto(self, *args, **kw):
    self.write('move_to', *args, **kw)
    yield
    while self.moving():
      yield

class MotorHomeEndBase(MotorRoutine):
  limitflag: str
  movefwd: int
  moveback: int

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
      raise RoutineError(None, CODE_MOTOR_BUSY)
    self.overrides['max_speed'] = m['max_speed']
    self.write('max_speed', m['homing_speed'])
    if not self.islimit():
      self.status_text = 'Moving'
      yield from self.ymove(self.movefwd * 0x7fffffff)
    if not self.islimit():
      raise RoutineError('Cannot reach limit')
    while self.islimit():
      self.status_text = 'Backing'
      yield from self.ymove(self.moveback * m['backing_steps'])
    self.write('max_speed', m['fixing_speed'])
    while not self.islimit():
      self.status_text = 'Fixing'
      yield from self.ymove(self.movefwd * m['backing_steps'] * 2)
    self.finish()
    self.status_text = 'Finished'

  def finish(self):
    pass

class MotorHomeRoutine(MotorHomeEndBase):
  limitflag: str = 'is_limit_acw'
  movefwd: int = -1
  moveback: int = 1

  def finish(self):
    self.write('position', 0)

class MotorEndRoutine(MotorHomeEndBase):
  limitflag: str = 'is_limit_cw'
  movefwd: int = 1
  moveback: int = -1

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

class MotorScriptBuilder:

  def __init__(self, motor: Motor):
    self.motor = motor
    self.cmds = []

  def clear(self):
    self.cmds = []

  def copy(self):
    other = type(self)(self.motor)
    other.cmds.extend(self.cmds)
    return other

  def pop(self):
    return self.cmds.pop()

  def add(self, name: str, *v, end: bool = False):
    fmt, src = self.motor.script_cmdinfo(name)
    if end:
      fmt += 'B'
    newlength = len(self) + struct.calcsize(fmt) + 2
    if newlength > 0xF8:
      raise ValueError(f'Size limit exceeded')
    buf = bytearray()
    buf.append(src)
    buf.extend(struct.pack(f'<{fmt}', *v))
    if end:
      buf.append(0xFF)
    self.cmds.append((name, v, f'B{fmt}', bytes(buf)))
    return newlength

  def cond_hasflag(self, name: str, page: int, sidx: int):
    flagdef = self.motor.FLAGMAP[name]
    return self.add('cond_call', 0x10, 1 << flagdef[2], page, sidx)

  def cond_notflag(self, name: str, page: int, sidx: int):
    flagdef = self.motor.FLAGMAP[name]
    return self.add('cond_call', 0x11, 1 << flagdef[2], page, sidx)
    
  def tobuffer(self, end: bool = True):
    buf = bytearray()
    for x in self.cmds:
      buf.extend(x[3])
    if end:
      buf.append(0xFF)
    return bytes(buf)

  def buffmt(self, end: bool = True):
    fmt = '<' + ''.join(x[2] for x in self.cmds)
    if end:
      fmt += 'B'
    return fmt

  def save(self, page: int = 0, *, exec: bool = False, fail: bool = True, **kw):
    motor = self.motor
    code = motor.write('script_clear', page, fail=fail, **kw)
    if code != CODE_OK:
      return code
    code = motor.write(f'script{page}', self.tobuffer(), fail=fail, **kw)
    if code != CODE_OK:
      return code
    if exec:
      code = motor.write('script_exec', page, fail=fail, **kw)
    return code
    
  def __len__(self):
    return struct.calcsize(self.buffmt(end=False))


"""
b1 = m.script_builder()
x = [1635, 1835, 2060, 2183, 2450, 2750, 2914, 3270, -2914, -2750, -2450, -2183, -2060, -1835, -1635]
for steps in x:
  b1.add('max_speed', abs(steps))
  b1.add('move', round(-steps * 0.475))
  b1.add('delay', 50)

b2 = m.script_builder()
y = [1945, 2183, 2312, 2750, 2914, 3270, 3465, 3889, -3465, -3270, -2914, -2750, -2312, -2183, -1945]
for steps in y:
  b2.add('max_speed', abs(steps))
  b2.add('move', round(-steps * 0.2375))
  b2.add('delay', 25)

b2.add('script_call', 1)

b0 = m.script_builder()
b0.add('script_call', 1)
b0.add('script_call', 2)
b0.add('script_call', 2)
b0.add('script_call', 1)
"""
