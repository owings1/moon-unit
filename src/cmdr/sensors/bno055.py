from __future__ import annotations

import time

from adafruit_bno055 import BNO055_I2C as SensorBase
from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_struct import ROUnaryStruct
from micropython import const
from utils import ysleep

try:
  from typing import Generator

  import busio
except ImportError:
  pass

PAGE_REGISTER = const(0x07)
CALIBRATION_REGISTER = const(0x35)
MODE_REGISTER = const(0x3d)
TRIGGER_REGISTER = const(0x3f)

CONFIG_MODE = const(0x00)
NDOF_MODE = const(0x0c)

class BNO055(SensorBase):
  calflag = ROUnaryStruct(CALIBRATION_REGISTER, 'B')

  def __init__(self, i2c: busio.I2C, address: int = 0x28, defer_init: bool = False) -> None:
    if defer_init:
      self.buffer = bytearray(2)
      self.i2c_device = I2CDevice(i2c, address)
    else:
      super().__init__(i2c, address)

  @property
  def mode(self) -> int:
    return super().mode

  @mode.setter
  def mode(self, new_mode: int) -> None:
    for _ in self.yset_mode(new_mode):
      time.sleep(0.001)

  def yset_mode(self, new_mode: int) -> Generator[None]:
    if self.mode == new_mode:
      return
    if self.mode != CONFIG_MODE:
      self._write_register(MODE_REGISTER, CONFIG_MODE)
      yield from ysleep(0.019)
    if new_mode != CONFIG_MODE:
      self._write_register(MODE_REGISTER, new_mode)
      yield from ysleep(0.07)

  def yreset(self) -> Generator[None]:
    yield from self.yset_mode(CONFIG_MODE)
    try:
      # reset
      self._write_register(TRIGGER_REGISTER, 0x20)
    except OSError:
      pass
    yield from ysleep(0.7)
    self.set_normal_mode()
    self._write_register(PAGE_REGISTER, 0x00)
    self._write_register(TRIGGER_REGISTER, 0x00)

  def yconfig(self, mode: int|None = None, **config) -> Generator[None]:
    mode = mode or self.mode
    if config:
      yield from self.yset_mode(CONFIG_MODE)
      for name, value in config.items():
        setattr(self, name, value)
    yield from self.yset_mode(mode)

  def yinit(self, mode: int = NDOF_MODE, **config) -> Generator[None]:
    yield from self.yreset()
    yield from self.yconfig(mode=mode, **config)

  # @property
  # def quaternion_euler_zyx(self) -> tuple[float, float, float]:
  #   """
  #   Source: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
  #   Addison L. Sears-Collins
  #   """
  #   w, x, y, z = self.quaternion
  #   if w is None or x is None or y is None or z is None:
  #     return 0.0, 0.0, 0.0
  #   # Roll (x-axis rotation)
  #   t0 = +2.0 * (w * x + y * z)
  #   t1 = +1.0 - 2.0 * (x * x + y * y)
  #   roll = math.atan2(t0, t1)

  #   # Pitch (y-axis rotation)
  #   t2 = +2.0 * (w * y - z * x)
  #   t2 = +1.0 if t2 > +1.0 else t2
  #   t2 = -1.0 if t2 < -1.0 else t2
  #   pitch = math.asin(t2)

  #   # Yaw (z-axis rotation)
  #   t3 = +2.0 * (w * z + x * y)
  #   t4 = +1.0 - 2.0 * (y * y + z * z)
  #   yaw = math.atan2(t3, t4)

  #   return tuple(abs(math.degrees(x)) for x in (yaw, pitch, roll))