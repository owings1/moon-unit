from __future__ import annotations

import struct
import time
from collections import OrderedDict

import board
import busio
from digitalio import DigitalInOut, Direction
from micropython import const
from utils import Pkr

from . import CompAttr, DeviceComponent

class IMU6(DeviceComponent):
  PKR = Pkr('<')
  ATTRMAP: dict[str, CompAttr] = CompAttr.makeattrs(PKR, OrderedDict(
    acceleration=dict(fmt='3e'),
    gyro=dict(fmt='3e'),
    temperature=dict(fmt='e'),
  ))
  onboard_i2c: bool = False
  _obpwr: DigitalInOut|None = None
  _obbus: busio.I2C|None = None

  def __init__(
    self,
    bus: busio.I2C|None = None,
    address: int = 0x6a,
    onboard_i2c: bool = False,
    refresh_interval: int = 200
  ) -> None:
    if onboard_i2c and bus is None:
      # From:
      # https://learn.adafruit.com/adafruit-lsm6ds3tr-c-6-dof-accel-gyro-imu/python-circuitpython
      # Copyright (c) 2020 Bryan Siepert for Adafruit Industries. MIT License
      #   > On the Seeed XIAO nRF52840 Sense the LSM6DS3TR-C IMU is connected on a separate
      #   > I2C bus and it has its own power pin that we need to enable.
      self.onboard_i2c = True
      self._obpwr = DigitalInOut(board.IMU_PWR)
      self._obpwr.direction = Direction.OUTPUT
      self._obpwr.value = True
      time.sleep(0.1)
      self._obbus = bus = busio.I2C(board.IMU_SCL, board.IMU_SDA)
    super().__init__(bus, address)
    from adafruit_lsm6ds.lsm6ds3trc import LSM6DS3TRC
    self.sensor = LSM6DS3TRC(self.bus, address)
    self.refresh_interval = refresh_interval
    self.packed = bytearray(self.PKR.size)

  # def __getitem__(self, name: str):
  #   if name in self.FLAGMAP:
  #     flagdef = self.FLAGMAP[name]
  #     return (self[flagdef[1]] >> flagdef[2]) & flagdef[3]
  #   return self.ATTRMAP[name].unpack_from(self.packed)

  def refresh(self) -> bool:
    change = False
    for attr in self.ATTRMAP.values():
      prev = change or self[attr.name]
      attr.pack_into(self.packed, getattr(self.sensor, attr.name))
      change = change or prev != self[attr.name]
    return change

  def deinit(self):
    if self.onboard_i2c:
      self._obbus.deinit()
      self._obpwr.value = False
      self._obpwr.deinit()

class Imu9RefreshState:
  next_stage = 1
  mode_restore = None
  it = None
  new_config = None
  is_init = False

  def ready(self):
    if self.waiting():
      try:
        next(self.it)
      except StopIteration:
        return True
    return False

  def waiting(self):
    return bool(self.next_stage and self.it)

class IMU9(DeviceComponent):
  PKR = Pkr('<')
  ATTRMAP: dict[str, CompAttr] = CompAttr.makeattrs(PKR, OrderedDict(
    calflag=dict(fmt='B'),
    acceleration=dict(fmt='3e'),
    magnetic=dict(fmt='3e'),
    gyro=dict(fmt='3e'),
    euler=dict(fmt='3e'),
    quaternion=dict(fmt='4e'),
    gravity=dict(fmt='3e'),
    linear_acceleration=dict(fmt='3e'),
    temperature=dict(fmt='b'),
    mode=dict(fmt='B', writeable=True),
    # ---- these require switch to config mode with delay
    offsets_accelerometer=dict(fmt='3h', writeable=True),
    offsets_magnetometer=dict(fmt='3h', writeable=True),
    offsets_gyroscope=dict(fmt='3h', writeable=True),
    radius_accelerometer=dict(fmt='h'),
    radius_magnetometer=dict(fmt='h'),
  ))
  FLAGMAP = OrderedDict((x[0], x) for x in (
    ('cal_mag', 'calflag', 0x0, 0x3),
    ('cal_accel', 'calflag', 0x2, 0x3),
    ('cal_gyro', 'calflag', 0x4, 0x3),
    ('cal_sys', 'calflag', 0x6, 0x3),
  ))
  SLCINFO_DATA = CompAttr.sliceinfo(ATTRMAP, 0, -5)
  SLCINFO_CONFIG = CompAttr.sliceinfo(ATTRMAP, -5, None)
  SLCINFO_PERSIST = CompAttr.sliceinfo(ATTRMAP, -6, -2)
  PERSIST_NS = const(0x4939)
  PERSIST_VER = const(0x03)
  MIN_REFRESH_INTERVAL = const(90)
  CONFIG_MODE = const(0x00)

  @property
  def refresh_interval(self) -> int:
    return max(self.MIN_REFRESH_INTERVAL, self._refresh_interval)

  @refresh_interval.setter
  def refresh_interval(self, value: int) -> None:
    self._refresh_interval = value

  def __init__(
    self,
    bus: busio.I2C|None = None,
    address: int = 0x29,
    refresh_interval: int = 200,
    **config,
  ) -> None:
    from sensors.bno055 import BNO055
    super().__init__(bus, address)
    self.sensor = BNO055(self.bus, address, defer_init=True)
    self.refresh_interval = refresh_interval
    self.packed = bytearray(self.PKR.size)
    self.refresh_state: Imu9RefreshState = Imu9RefreshState()
    self.refresh_state.it = self.sensor.yinit(**config)

  # def __getitem__(self, name: str):
  #   if name in self.FLAGMAP:
  #     flagdef = self.FLAGMAP[name]
  #     return (self[flagdef[1]] >> flagdef[2]) & flagdef[3]
  #   return self.ATTRMAP[name].unpack_from(self.packed)

  def refresh_if_needed(self) -> int:
    if self.refresh_state.waiting():
      if self.refresh_state.ready():
        self.refresh_next_tick = True
      else:
        return 0
    return super().refresh_if_needed()
  
  def refresh(self) -> bool:
    state = self.refresh_state
    if state.waiting() and not state.ready():
      return False
    stage = state.next_stage

    if stage == 3:
      is_first_data = not state.is_init
      state.next_stage = 1
      if state.new_config:
        state.it = self.sensor.yconfig(**state.new_config)
        state.new_config = None
        # trigger first yield
        state.ready()
      else:
        state.it = None
        state.is_init = True
      return is_first_data

    if stage == 1:
      slcinfo = self.SLCINFO_DATA
      state.mode_restore = self.sensor.mode
      next_mode = self.CONFIG_MODE
    elif stage == 2:
      slcinfo = self.SLCINFO_CONFIG
      next_mode = state.mode_restore
      state.mode_restore = None

    # a = self.packed[slcinfo.slc]
    change = False
    for attr in slcinfo.attrs:
      prev = change or self[attr.name]
      value = getattr(self.sensor, attr.src or attr.name)
      attr.pack_into(self.packed, value)
      change = change or prev != self[attr.name]

    state.it = self.sensor.yset_mode(next_mode)
    state.next_stage = stage + 1
    # trigger first yield
    state.ready()
    return state.is_init and change

  def write(self, name: str, value) -> None:
    attrdef = self.ATTRMAP[name]
    if not attrdef.writeable:
      raise ValueError(f'{name} is readonly')
    if isinstance(value, tuple):
      struct.pack(attrdef.fmt, *value)
    else:
      struct.pack(attrdef.fmt, value)
    setattr(self.sensor, attrdef.src, value)

  def dump_persistent(self):
    return self.packed[self.SLCINFO_PERSIST.slc]

  def load_persistent(self, buf: bytes) -> None:
    slcinfo = self.SLCINFO_PERSIST
    self.refresh_state.new_config = {
      attr.name: attr.unpack_from(buf, -slcinfo.slc.start)
      for attr in slcinfo.attrs}



"""
def t(x):
  s._write_register(0x3d, 0x00)
  time.sleep(0.019)
  s._read_register(0x67)
  s._write_register(0x3d, 0x09)
  time.sleep(x)
  return s._read_register(0x34)

if True:
  s._write_register(0x3d, 0x09)
  time.sleep(0.007)
  s._read_register(0x34)

if True:
  s._write_register(0x3d, 0x00)
  time.sleep(0.019)
  s._read_register(0x67)
  s._write_register(0x3d, 0x00)
  time.sleep(0.019)
  s._write_register(0x3d, 0x09)
  time.sleep(0.04)
  s._read_register(0x34)

"""