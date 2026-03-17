from __future__ import annotations

import struct
from collections import namedtuple

import board
import busio
from utils import Pkr, millis

try:
  from typing import Any, ClassVar, Iterable, Self

  from app import App
except ImportError:
  pass

class Component:
  ATTRMAP: dict[str, CompAttr] = {}
  FLAGMAP = {}
  PERSIST_NS: int|None = None
  PERSIST_VER: int = 0x01
  component_address: int
  refresh_interval = 1000
  refreshed_at = 0
  refresh_next_tick = False
  changed_at = 0
  persistkey: tuple[int, int, int]|None = None
  debug: bool|None = None
  bus: busio.I2C|busio.SPI|None = None
  packed: bytes|bytearray|None = None

  @property
  def persist_id(self) -> int|None:
    return self.persistkey and self.persistkey[2]

  @persist_id.setter
  def persist_id(self, value: int|None) -> None:
    if self.PERSIST_NS and self.PERSIST_VER and value:
      self.persistkey = (self.PERSIST_NS, self.PERSIST_VER, value)
    else:
      self.persistkey = None

  @property
  def persistable(self) -> bool:
    return bool(self.persistkey)

  def __getitem__(self, name: str):
    if name in self.FLAGMAP:
      flagdef = self.FLAGMAP[name]
      return (self[flagdef[1]] >> flagdef[2]) & flagdef[3]
    if self.packed is None:
      raise KeyError(name)
    return self.ATTRMAP[name].unpack_from(self.packed)

  def items(self) -> Iterable[tuple[str, Any]]:
    return (
      (name, self[name])
      for names in (self.FLAGMAP, self.ATTRMAP)
        for name in names)

  def metaitems(self) -> Iterable[tuple[str, Any]]:
    yield 'classname', type(self).__name__
    yield 'component_address', hex(self.component_address)
    if self.persistkey:
      yield 'persistkey', tuple(map(hex, self.persistkey))

  def is_refresh_needed(self) -> bool:
    if self.refresh_next_tick:
      return True
    now = millis()
    if self.refreshed_at < now - self.refresh_interval:
      return True
    return False
  
  def refresh_if_needed(self) -> int:
    now = millis()
    if self.is_refresh_needed():
      self.refresh_next_tick = False
      change = self.refresh()
      self.refreshed_at = now
      if change:
        self.changed_at = now
      return 1 + change
    return 0
  
  def refresh(self) -> bool:
    return False

  def debug_lines(self) -> Iterable[str]:
    yield f'#######################################'
    for k, v in self.metaitems():
      yield f'# @{k} {v}'
    yield f'#######################################'
    for k, v in self.debugitems():
      yield f'{k}={v}'

  def debugitems(self) -> Iterable[tuple[str, Any]]:
    yield from self.items()

  def deinit(self) -> None:
    pass

  def load_persistent(self, buf: bytes) -> None:
    pass

  def dump_persistent(self) -> bytes|bytearray|None:
    pass

  def app_init(self, app: App) -> None:
    pass

  def app_ready(self, app: App) -> None:
    pass

class DeviceComponent(Component):

  def __init__(self, bus: busio.I2C|None, address: int) -> None:
    from adafruit_bus_device.i2c_device import I2CDevice
    bus = bus or board.I2C()
    self.device = I2CDevice(bus, address)
    self.device_address = address
    self.bus = bus

  @property
  def component_address(self) -> int:
    return self.device_address << 0x8

  def metaitems(self) -> Iterable[tuple[str, Any]]:
    yield from super().metaitems()
    yield 'device_address', hex(self.device_address)

class CompAttr(
  namedtuple(
    'CompAttr', (
      'name',
      'src',
      'start',
      'end',
      'fmt',
      'bom',
      'writeable',
      'scale'))):
  name: str
  src: Any
  start: int
  end: int
  fmt: str
  bom: str
  writeable: bool
  scale: int|float

  def descale(self, value: int|None|tuple[int|None, ...]) -> int|float|tuple[int|float, ...]:
    if isinstance(value, tuple):
      return tuple(map(self.descale, value))
    if value is None:
      value = 0
    if self.scale != 1:
      value = round(value / self.scale)
    return value

  def unpack_from(self, buf: bytearray, offset: int|None = None):
    if offset is None:
      offset = self.start
    elif offset < 0:
      offset += self.start
    raw = struct.unpack_from(self.bom+self.fmt, buf, offset)
    it = (x * self.scale for x in raw)
    if len(raw) == 1:
      return next(it)
    return tuple(it)

  def pack_into(self, buf: bytearray, value, offset: int|None = None):
    if offset is None:
      offset = self.start
    elif offset < 0:
      offset += self.start
    value = self.descale(value)
    if not isinstance(value, tuple):
      value = (value,)
    struct.pack_into(self.bom+self.fmt, buf, offset, *value)

  defaults: ClassVar = dict(writeable=False, scale=1, src=None)

  @classmethod
  def prepdefn(cls, defn: dict[str, Any]):
    defn.update(cls.defaults|defn)

  @classmethod
  def makeattrs(cls, pkr: Pkr, defns: dict[str, dict[str, Any]]):
    from collections import OrderedDict
    defnmap: dict[str, Self] = OrderedDict()
    for name, defn in defns.items():
      defn['name'] = name
      cls.prepdefn(defn)
      start = pkr.size
      pkr.add(defn['fmt'])
      end = pkr.size
      defnmap[name] = cls(start=start, end=end, bom=pkr.bom, **defn)
    return defnmap

  @classmethod
  def sliceinfo(cls, attrmap: dict[str, Self], start, end):
    attrs = tuple(attrmap.values())[start:end]
    slc = slice(attrs[0].start, attrs[-1].end)
    fmt = attrs[0].bom + ''.join(x.fmt for x in attrs)
    return cls.SliceInfo(attrs, fmt, slc)

  class SliceInfo(namedtuple('SliceInfo', ('attrs', 'fmt', 'slc'))):
    attrs: tuple[CompAttr, ...]
    fmt: str
    slc: slice

