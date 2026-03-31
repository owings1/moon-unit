from __future__ import annotations

import struct
import traceback
from collections import OrderedDict, deque

import busio
import moic
from moic import Code
from utils import Pkr

from . import CompAttr, DeviceComponent, ActDef, FlagDef

try:
  from typing import Any, Generator, Iterable
except ImportError:
  pass

__all__ = ('Motor',)

class MotorAttr(CompAttr):
  src: int|None

  def regptr(self, motor: Motor):
    if self.src is None:
      return
    if self.src < 0:
      return -1 * self.src
    return self.src + motor.regoffset

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
    script_page={},
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
    # --- debug
    wait_end_time={},
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
  ACTMAP = OrderedDict(
    (x.name, ActDef(x.name, x.offset, x.fmt))
      if isinstance(x, moic.Attribute) else
    (x[0], ActDef(*x)) for x in (
    moic.Attributes.stop,
    ('home', '_act_home', ''),
    ('end', '_act_end', ''),
    ('home_end', '_act_home_end', ''),
    ('limits_on', '_act_limits_on', ''),
    ('limits_off', '_act_limits_off', ''),
    moic.Attributes.move,
    moic.Attributes.move_rev,
    moic.Attributes.move_to,
    ('move_at_speed', '_act_move_at_speed', b'fl'),
    ('move_to_at_speed', '_act_move_to_at_speed', b'fl'),
    moic.Attributes.delay,
    moic.Attributes.script_clear,
    moic.Attributes.script_exec,
    moic.Attributes.call,
    moic.Attributes.cond_call,
    moic.Attributes.cond_jump,
    moic.Attributes.jump,
  ))
  SLCINFO_PERSIST = MotorAttr.sliceinfo(ATTRMAP, 7, 7+12)
  PERSIST_NS = 0x9100
  PERSIST_VER = 0x0A
  init_defaults = OrderedDict(
    settings_flags=0x03,
    max_speed=2000.0,
    default_speed=2000.0,
    homing_speed=3000.0,
    fixing_speed=500.0,
    acceleration=60_000.0,
    msteps_per_degree=10_000,
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
    if not 1 <= id <= moic.MAX_MOTORS:
      raise ValueError(f'{id=}')
    super().__init__(bus, address)
    self.refresh_interval = refresh_interval
    self.id = id
    initial = OrderedDict(self.init_defaults)
    initial.update(init_data)
    self.packed = bytearray(self.PKR.size)
    self.regoffset = (moic.MOTOR_BLOCK_SIZE * ((self.id - 1) % 2)) + moic.MOTOR_BASE_ADDR
    self.pagebuf = struct.pack(b'<2B', moic.PAGE_REGISTER, (self.id - 1) // 2)
    self.rbuf = bytearray(2)
    for k, v in initial.items():
      self.write(k, v, fail=True)
    self.read('boot_id')
    self.last_boot_id = self['boot_id']
    self.scripts = MotorScripts(self)

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
      device.write(self.pagebuf)
      device.write_then_readinto(
        attr.regptr(self).to_bytes(1, 'little'),
        self.packed,
        in_start=attr.start,
        in_end=attr.end)

  def write(self, name: str, *v, unsafe: bool = False, fail: bool = True) -> int:
    code = self._write(name, *v, unsafe=unsafe)
    if fail and code != Code.OK:
      raise WriteFailed(f'Write Failed {name=}', code)
    return code

  def _write(self, name: str, *v, unsafe: bool = False) -> int:
    if not unsafe and self.routine:
      if name == 'stop':
        self.routine = self.routine.cancel()
      else:
        return Code.MOTOR_BUSY
    if name in self.ATTRMAP:
      attr = self.ATTRMAP[name]
      if not attr.writeable:
        return Code.READONLY_ATTRIBUTE
      if attr.src is None:
        attr.pack_into(self.packed, v)
        return Code.OK
      bufw = self._get_attr_write_buf(attr, *v)
    else:
      actdef = self.ACTMAP[name]
      if isinstance(actdef.src, str):
        # if actdef.fmt:
        #   v = struct.unpack(actdef.fmt, struct.pack(actdef.fmt, *v))
        if actdef.src.startswith('_routine'):
          if self.routine:
            return Code.COMMAND_IGNORED
          self.routine = getattr(self, actdef.src)(*v)
          next(self.routine)
          return Code.OK
        if actdef.src.startswith('_act'):
          return getattr(self, actdef.src)(*v, unsafe=unsafe)
        return Code.UNKNOWN_COMMAND
      bufw = self._get_act_write_buf(actdef, *v)
    with self.device as device:
      device.write(self.pagebuf)
      device.write(bufw)
      # Read response code
      device.write_then_readinto(self.rbuf, self.rbuf, out_end=1, in_start=1)
    return self.rbuf[1]
    
  def _get_attr_write_buf(self, attr: MotorAttr, *v):
    reg = attr.regptr(self)
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
    return self.write(flagdef.attr, self[flagdef.attr] | (1 << flagdef.bit), **kw)

  def _act_limits_off(self, **kw):
    flagdef = self.FLAGMAP['limits_enabled']
    return self.write(flagdef.attr, self[flagdef.attr] & ~(1 << flagdef.bit), **kw)

  def _act_home(self, **kw) -> int:
    return self.scripts.exec('home')

  def _act_end(self, **kw) -> int:
    return self.scripts.exec('end')

  def _act_home_end(self, **kw) -> int:
    return self.scripts.exec('home_end')
    
  def _act_move_at_speed(self, speed: float, steps: int, **kw) -> int:
    s = moic.Script()
    s.add('max_speed', speed)
    s.add('move', steps)
    s.add('max_speed', self['max_speed'])
    self.scripts.upload(0, s.compile())
    return self.write('script_exec', 0, 0, **kw)

  def _act_move_to_at_speed(self, speed: float, position: int, **kw) -> int:
    s = moic.Script()
    s.add('max_speed', speed)
    s.add('move_to', position)
    s.add('max_speed', self['max_speed'])
    self.scripts.upload(0, s.compile())
    return self.write('script_exec', 0, 0, **kw)

class MotorScripts:
  fixlib_page = 2
  moicdb_page = 3
  moicdb_slcinfo = MotorAttr.sliceinfo(Motor.ATTRMAP, 10, 10+6)

  def __init__(self, motor: Motor):
    self._static: dict[str, bytes] = {}
    self.last_boot_id = None
    self.motor = motor
    self.rbuf = motor.rbuf
    self.pagebufs = tuple(
      struct.pack(
        b'<2B',
        moic.PAGE_REGISTER,
        moic.SCRIPT_PAGE_START * (i + 1) + (self.motor.id - 1))
      for i in range(moic.NUM_SCRIPT_PAGES))
    self._execargs: dict[str, tuple[int, int]] = dict(
      home=(self.fixlib_page, 1),
      end=(self.fixlib_page, 2),
      home_end=(self.fixlib_page, 3),)

  def exec(self, name: str):
    page, arg = self._execargs[name]
    self.sync()
    return self.motor.write('script_exec', page, arg)

  def sync(self):
    self.upload(self.moicdb_page, self.moicdb_content)
    if self.motor['boot_id'] != self.last_boot_id:
      self.upload(self.fixlib_page, self.fixlib_content)
      self.last_boot_id = self.motor['boot_id']

  def upload(self, page: int, content: bytes|bytearray):
    if len(content) > moic.SCRIPT_PAGE_SIZE:
      raise ValueError(f'Script length exceeds max {moic.SCRIPT_PAGE_SIZE}')
    self.motor.write('script_clear', page)
    buf = bytearray()
    buf.append(moic.MOTOR_BASE_ADDR)
    buf.extend(content)
    with self.motor.device as device:
      device.write(self.pagebufs[page])
      device.write(buf)
      device.write_then_readinto(self.rbuf, self.rbuf, out_end=1, in_start=1)
    code = self.rbuf[1]
    if code != Code.OK:
      raise WriteFailed(f'Failed writing script {page=}', code)

  def download(self, page: int) -> bytearray:
    content = bytearray(moic.SCRIPT_PAGE_SIZE)
    buf = bytearray()
    buf.append(moic.MOTOR_BASE_ADDR)
    with self.motor.device as device:
      device.write(self.pagebufs[page])
      device.write_then_readinto(buf, content)
    return content

  def pprintf(self, script: int|bytes|bytearray) -> str:
    if isinstance(script, int):
      script = self.download(script)
    sq = deque((), 8)
    for x in range(8):
      it = (f'{hex(script[x*31+i])[2:]:0>2}' for i in range(31))
      sq.append(' '.join(it))
    return '\n'.join(sq)

  def pprint(self, script: int|bytes|bytearray) -> None:
    print(self.pprintf(script))

  @property
  def moicdb_content(self):
    return self.motor.packed[self.moicdb_slcinfo.slc]

  @property
  def fixlib_content(self):
    try:
      return self._static['fixlib']
    except KeyError:
      return self._static.setdefault('fixlib', self.fixlib_generate())

  def fixlib_generate(self):
    dbpg = self.moicdb_page
    ptr = self.moicdb_ptridx
    lib = moic.Script()
    __ = Code.UNSET
    with lib.if_argand(3, negate=True):
      lib.add(Code.USR3)
    with lib.if_flag('limits_enabled', negate=True):
      lib.add(Code.USR1)
    with lib.if_argand(1):
      lib.add('call', __, 'subroutine:home')
      with lib.if_repeql(Code.USR2):
        lib.add('call', __, 'subroutine:cleanup')
        lib.add(Code.USR2)
    with lib.if_argand(2):
      lib.add('call', __, 'subroutine:end')
      with lib.if_repeql(Code.USR2):
        lib.add('call', __, 'subroutine:cleanup')
        lib.add(Code.USR2)
    lib.add('call', __, 'subroutine:cleanup')
    lib.add(Code.UNSET)
    
    for label, direction in (('home', 'acw'), ('end', 'cw')):
      if direction == 'acw':
        mvback, mvfix = 'move', 'move_rev'
      else:
        mvback, mvfix = 'move_rev', 'move'
      flag = f'is_limit_{direction}'
      lib.label(f'subroutine:{label}')
      lib.add('max_speed**', dbpg, ptr('homing_speed'))
      with lib.if_flag(flag, negate=True):
        lib.add(mvfix,  0x7fffffff)
      with lib.if_flag(flag, negate=True):
        lib.add(Code.USR2)
      with lib.while_flag(flag):
        lib.add(f'{mvback}**', dbpg, ptr('backing_steps'))
        lib.add(f'{mvback}**', dbpg, ptr('backing_steps'))
      lib.add('max_speed**', dbpg, ptr('fixing_speed'))
      with lib.while_flag(flag, negate=True):
        lib.add(f'{mvfix}**', dbpg, ptr('backing_steps'))
      if label == 'home':
        lib.add('current_position', 0)
      lib.add(Code.UNSET)

    lib.label('subroutine:cleanup')
    lib.add('max_speed**', dbpg, ptr('default_speed'))
    lib.add(Code.UNSET)

    return lib.compile()

  def moicdb_ptridx(self, name: str) -> int:
    slcinfo = self.moicdb_slcinfo
    idx = Motor.ATTRMAP[name].start - slcinfo.slc.start
    if 0 <= idx < slcinfo.slc.stop:
      return idx
    raise ValueError(f'Not in moicdb slice: {name}')

class MotorError(Exception):
  errcode = Code.OTHER_ERROR
  errtext = 'Motor Error'

  def __init__(self, errtext: str|None = None, code: int|None = None):
    if errtext:
      self.errtext = errtext
    if code is not None:
      self.errcode = code
    self.errtext = f'{self.errtext} {hex(self.errcode)} {moic.codes[self.errcode]}'
    super().__init__(self.errtext, self.errcode)

class RoutineError(MotorError):
  errtext = 'Routine Error'

class WriteFailed(MotorError):
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
        self.status_text = f'Error: {err.errtext}'
      else:
        self.errcode = Code.OTHER_ERROR
        self.status_text = f'Error: {err!r}'
      self.cleanup()

  def __iter__(self):
    return self

  def cancel(self):
    self.error = RoutineError(None, Code.CANCELED)
    self.errcode = Code.CANCELED
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
    pass

  def moving(self) -> bool:
    self.motor.read('state_flags')
    return self.motor['is_moving']

  def write(self, name: str, *v) -> int:
    return self.motor.write(name, *v, unsafe=True)