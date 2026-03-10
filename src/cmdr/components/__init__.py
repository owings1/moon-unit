from __future__ import annotations

import board
import busio
from adafruit_bus_device.i2c_device import I2CDevice
from utils import millis


class Component:
  ATTRMAP = {}
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
    raise KeyError(name)

  def items(self) -> Iterable[tuple[str, Any]]:
    return (
      (name, self[name])
      for names in (self.FLAGMAP, self.ATTRMAP)
        for name in names)

  def metaitems(self) -> Iterable[tuple[str, Any]]:
    yield 'classname', type(self).__name__
    yield 'component_address', hex(self.component_address)
    if self.persistkey:
      yield 'persistkey', self.persistkey

  def refresh_if_needed(self) -> int:
    force_refresh, self.refresh_next_tick = self.refresh_next_tick, False
    now = millis()
    if force_refresh or self.refreshed_at < now - self.refresh_interval:
      change = self.refresh()
      self.refreshed_at = now
      if change:
        self.changed_at = now
      return 1 + change
    return 0
  
  def refresh(self) -> bool:
    return False

  def subcomponents(self) -> Iterable[Component]:
    return ()

  def debug_lines(self) -> Iterable[str]:
    yield f'#######################################'
    for k, v in self.metaitems():
      yield f'# @{k} {v}'
    yield f'#######################################'
    for k, v in self.items():
      yield f'{k}={v}'

  def deinit(self) -> None:
    pass

  def load_persistent(self, buf: bytes) -> None:
    pass

  def dump_persistent(self) -> bytes|bytearray|None:
    pass

  def app_ready(self, app: App) -> None:
    pass

class DeviceComponent(Component):

  def __init__(self, i2c: busio.I2C|None, address: int) -> None:
    i2c = i2c or board.I2C()
    self.device = I2CDevice(i2c, address)
    self.device_address = address
    self.bus = i2c

  @property
  def component_address(self) -> int:
    return self.device_address << 0x8

  def metaitems(self) -> Iterable[tuple[str, Any]]:
    yield from super().metaitems()
    yield 'device_address', hex(self.device_address)

try:
  from typing import Any, Iterable

  from app import App
except ImportError:
  pass