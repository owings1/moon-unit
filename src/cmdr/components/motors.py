from __future__ import annotations

import struct
import traceback
from collections import OrderedDict, deque

import busio
import moic
from micropython import const
from moic import FunId, Return
from utils import Pkr

from . import CompAttr, DeviceComponent, ActDef, FlagDef

try:
  from typing import Any, Generator, Iterable, Literal
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
CODE_UNINVITED_POINTER = const(0x2F)
CODE_READONLY_ATTRIBUTE = const(0x30)
CODE_OVERFLOW = const(0x31)
CODE_UNSET = const(0xFF)

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
      return motor.script_pagebufs[int(self.name[-1], 0x10)]
    return motor.pagebuf

  @classmethod
  def prepdefn(cls, defn: dict[str, Any]):
    if defn['name'] in moic.attrsmap:
      attr = moic.attrsmap[defn['name']]
      defn.setdefault('src', attr.offset)
      defn.setdefault('fmt', attr.fmt)
    super().prepdefn(defn)

class Motor(DeviceComponent):
  PKR = Pkr(b'<')
  ATTRMAP: dict[str, MotorAttr] = MotorAttr.makeattrs(PKR, OrderedDict(
    # --- Negative src indicates global device register
    boot_id=dict(src=-0x02, fmt=b'H'),
    state_flags={},
    script_repcode={},
    script_index={},
    target_position={},
    speed={},
    # --- writeable & persisted attributes
    current_position=dict(writeable=True),
    position_max=dict(fmt=b'L', writeable=True),
    settings_flags=dict(writeable=True),
    max_speed=dict(writeable=True),
    default_speed=dict(fmt=b'f', writeable=True),
    homing_speed=dict(fmt=b'f', writeable=True),
    fixing_speed=dict(fmt=b'f', writeable=True),
    acceleration=dict(writeable=True),
    backing_steps=dict(fmt=b'l', writeable=True),
    msteps_per_degree=dict(fmt=b'L', writeable=True),
    enable_delay_ms=dict(writeable=True),
    sleep_timeout_ms=dict(writeable=True),
    # --- not persisted
    script0=dict(fmt=b'248B', src=-0x08, writeable=True),
    script1=dict(fmt=b'248B', src=-0x08, writeable=True),
    script2=dict(fmt=b'248B', src=-0x08, writeable=True),
    script3=dict(fmt=b'248B', src=-0x08, writeable=True),
    script4=dict(fmt=b'248B', src=-0x08, writeable=True),
    script5=dict(fmt=b'248B', src=-0x08, writeable=True),
    script6=dict(fmt=b'248B', src=-0x08, writeable=True),
  ))
  FLAGMAP = OrderedDict((x[0], FlagDef(*x)) for x in (
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
    ('stop', moic.Attributes.stop.offset, b'x'),
    ('home', '_act_home', ''),
    ('end', '_act_end', ''),
    ('home_end', '_act_home_end', ''),
    ('limits_on', '_act_limits_on', ''),
    ('limits_off', '_act_limits_off', ''),
    ('move', moic.Attributes.move.offset, moic.Attributes.move.fmt),
    ('move_rev', moic.Attributes.move_rev.offset, moic.Attributes.move_rev.fmt),
    ('move_to', moic.Attributes.move_to.offset, moic.Attributes.move_to.fmt),
    ('move_at_speed', '_act_move_at_speed', b'fl'),
    ('move_to_at_speed', '_act_move_to_at_speed', b'fl'),
    ('delay', moic.Attributes.delay.offset, b'L'),
    ('script_clear', moic.Attributes.script_clear.offset, b'B'),
    ('script_exec', moic.Attributes.script_exec.offset, b'B'),
    ('call', moic.Attributes.call.offset, b'2B'),
    ('cond_call', moic.Attributes.cond_call.offset, b'4B'),
    ('cond_jump', moic.Attributes.cond_jump.offset, b'4B'),
    ('jump', moic.Attributes.jump.offset, b'2B'),
  ))
  SLCINFO_PERSIST = MotorAttr.sliceinfo(ATTRMAP, 6, 6+12)
  PERSIST_NS = 0x9100
  PERSIST_VER = 0x0A
  SLCINFO_MOICDB = MotorAttr.sliceinfo(ATTRMAP, 9, 9+6)
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
      for x in range(0x10, 0xC0, 0x10))
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

  def write(self, name: str, *v, unsafe: bool = False, fail: bool = True) -> int:
    code = self._write(name, *v, unsafe=unsafe)
    if fail and code != Return.OK:
      raise WriteFailed(f'Write Failed {name=} code={hex(code)}', code=code)
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
      fmt = b'<B' + attr.fmt
      bufw = bytearray(struct.calcsize(fmt))
      attr.pack_into(bufw, v, 1)
    bufw[0] = reg
    return bufw

  def _get_act_write_buf(self, actdef: ActDef, *v):
    reg = actdef.src + self.regoffset
    fmt = b'<B' + actdef.fmt
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

  def sync_moicdb(self):
    slcinfo = self.SLCINFO_MOICDB
    self.write('script_clear', 5, fail=True)
    self.write('script5', self.packed[slcinfo.slc], fail=True)

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

  def _act_home(self, **kw) -> int:
    scripts: HomeEndScripts = HomeEndScripts()
    self.sync_moicdb()
    self.write('script_clear', 0, **kw)
    self.write('script_clear', 4, **kw)
    self.write('script4', scripts.lib(5), **kw)
    self.write('script0', scripts.build('home', 4), **kw)
    return self.write('script_exec', 0, **kw)

  def _act_end(self, **kw) -> int:
    scripts: HomeEndScripts = HomeEndScripts()
    self.sync_moicdb()
    self.write('script_clear', 0, **kw)
    self.write('script_clear', 4, **kw)
    self.write('script4', scripts.lib(5), **kw)
    self.write('script0', scripts.build('end', 4), **kw)
    return self.write('script_exec', 0, **kw)

  def _act_home_end(self, **kw) -> int:
    scripts: HomeEndScripts = HomeEndScripts()
    self.sync_moicdb()
    self.write('script_clear', 0, **kw)
    self.write('script_clear', 4, **kw)
    self.write('script4', scripts.lib(5), **kw)
    self.write('script0', scripts.build('home_end', 4), **kw)
    return self.write('script_exec', 0, **kw)
    
  def _act_move_at_speed(self, speed: float, steps: int, **kw) -> int:
    s = moic.Script()
    s.add('max_speed', speed)
    s.add('move', steps)
    s.add('max_speed', self['max_speed'])
    code = self.write('script_clear', 0, **kw)
    if code != CODE_OK:
      return code
    code = self.write('script0', s.compile(), **kw)
    if code != CODE_OK:
      return code
    return self.write('script_exec', 0, **kw)

  def _act_move_to_at_speed(self, speed: float, position: int, **kw) -> int:
    s = moic.Script()
    s.add('max_speed', speed)
    s.add('move_to', position)
    s.add('max_speed', self['max_speed'])
    code = self.write('script_clear', 0, **kw)
    if code != CODE_OK:
      return code
    code = self.write('script0', s.compile(), **kw)
    if code != CODE_OK:
      return code
    return self.write('script_exec', 0, **kw)

class HomeEndScripts:

  def lib(self, dbpg: int):
    dbstart = Motor.SLCINFO_MOICDB.slc.start
    mattrs = Motor.ATTRMAP
    lib = moic.Script()
    __ = 0xFF
    with lib.if_argand(3, negate=True):
      lib.add(Return.USR3)
    with lib.if_flag('limits_enabled', negate=True):
      lib.add(Return.USR1)
    with lib.if_argand(1):
      lib.add('call', __, 'subroutine:home')
      with lib.if_repeql(Return.USR2):
        lib.add('call', __, 'subroutine:cleanup')
        lib.add(Return.USR2)
    with lib.if_argand(2):
      lib.add('call', __, 'subroutine:end')
      with lib.if_repeql(Return.USR2):
        lib.add('call', __, 'subroutine:cleanup')
        lib.add(Return.USR2)
    lib.add('call', __, 'subroutine:cleanup')
    lib.add(Return.UNSET)
    
    for label, direction in (('home', 'acw'), ('end', 'cw')):
      if direction == 'acw':
        mvback, mvfix = 'move', 'move_rev'
      else:
        mvback, mvfix = 'move_rev', 'move'
      flag = f'is_limit_{direction}'
      lib.label(f'subroutine:{label}')
      lib.add('max_speed**', dbpg, mattrs['homing_speed'].start - dbstart)
      with lib.if_flag(flag, negate=True):
        lib.add(mvfix,  0x7fffffff)
      with lib.if_flag(flag, negate=True):
        lib.add(Return.USR2)
      with lib.while_flag(flag):
        lib.add(f'{mvback}**', dbpg, mattrs['backing_steps'].start - dbstart)
      lib.add('max_speed**', dbpg, mattrs['fixing_speed'].start - dbstart)
      with lib.while_flag(flag, negate=True):
        lib.add(f'{mvfix}**', dbpg, mattrs['backing_steps'].start - dbstart)
      if label == 'home':
        lib.add('current_position', 0)
      lib.add(Return.UNSET)

    lib.label('subroutine:cleanup')
    lib.add('max_speed**', dbpg, mattrs['default_speed'].start - dbstart)
    lib.add(Return.UNSET)

    return lib.compile()
    
  def build(self, routine: Literal['home', 'end', 'home_end'], libpg: int):
    main = moic.Script()
    reqs = (routine.startswith, 'home'), (routine.endswith, 'end')
    arg = 0
    for i, (pred, pat) in enumerate(reqs):
      if pred(pat):
        arg |= i + 1
    main.add('cond_call', FunId.ALWAYS_TRUE, arg, libpg, 0)
    for ret in (Return.USR1, Return.USR2, Return.USR3):
      with main.if_repeql(ret):
        main.add(ret)
    main.add(Return.UNSET)
    return main.compile()
    
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
  def __init__(self, errtext: str|None = None, code: int|None = None):
    if errtext:
      self.errtext = errtext
    if code is not None:
      self.errcode = code

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
