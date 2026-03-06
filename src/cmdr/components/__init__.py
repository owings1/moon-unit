from __future__ import annotations

import busio
from adafruit_bus_device.i2c_device import I2CDevice
from utils import millis

class Component:
  ATTRMAP = {}
  FLAGMAP = {}
  component_address: int
  refresh_interval = 1000
  refreshed_at = 0
  changed_at = 0
  debug: bool|None = None

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

  def refresh_if_needed(self) -> int:
    if self.refreshed_at < (now := millis()) - self.refresh_interval:
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

class DeviceComponent(Component):

  def __init__(self, i2c: busio.I2C, address: int) -> None:
    self.device = I2CDevice(i2c, address)
    self.device_address = address

  @property
  def component_address(self) -> int:
    return self.device_address << 0x8

  def metaitems(self) -> Iterable[tuple[str, Any]]:
    yield from super().metaitems()
    yield 'device_address', hex(self.device_address)

try:
  from typing import Any, Iterable
except ImportError:
  pass