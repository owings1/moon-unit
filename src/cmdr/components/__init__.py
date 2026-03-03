from __future__ import annotations

import busio
from adafruit_bus_device.i2c_device import I2CDevice
from utils import millis

class Component:
  packed: bytearray|bytes = b''
  component_address: int

class I2CMixin:

  def __init__(self, i2c: busio.I2C, address: int) -> None:
    self.device = I2CDevice(i2c, address)
    self.device_address = address

  @property
  def component_address(self) -> int:
    return self.device_address << 0x8

class RefreshMixin:
  refresh_interval = 1000
  refreshed_at = 0
  changed_at = 0

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