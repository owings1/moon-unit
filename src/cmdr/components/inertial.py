from __future__ import annotations

import math
import struct
import time
from collections import OrderedDict

import board
import busio
from digitalio import DigitalInOut, Direction
from utils import Pkr, ysleep

from . import CompAttr, DeviceComponent

try:
  from typing import Generator
except ImportError:
  pass

class IMU6(DeviceComponent):
  PKR = Pkr()
  TEMPERATURE_SENSITIVITY = 256
  TEMPERATURE_OFFSET = 25.0
  ATTRMAP = OrderedDict((x[0], x) for x in (
    ('accel', 'accel', PKR.size, PKR.add('3h')),
    ('gyro', 'gyro', PKR.size, PKR.add('3h')),
    ('temperature', 'temp', PKR.size, PKR.add('h')),
  ))
  onboard_i2c: bool = False
  _obpwr: DigitalInOut|None = None
  _obbus: busio.I2C|None = None

  def __init__(
    self,
    i2c: busio.I2C|None = None,
    address: int = 0x6a,
    onboard_i2c: bool = False,
    refresh_interval: int = 200
  ) -> None:
    if onboard_i2c and i2c is None:
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
      self._obbus = i2c = busio.I2C(board.IMU_SCL, board.IMU_SDA)
    super().__init__(i2c, address)
    from adafruit_lsm6ds.lsm6ds3trc import LSM6DS3TRC
    self.sensor = LSM6DS3TRC(self.bus, address)
    self.refresh_interval = refresh_interval
    self.packed = bytearray(self.PKR.size)

  def __getitem__(self, name: str):
    attrdef = self.ATTRMAP[name]
    raw = struct.unpack_from(attrdef[3], self.packed, attrdef[2])
    it = (self.scale(attrdef[1], x) for x in raw)
    if len(raw) == 1:
      return next(it)
    return tuple(it)

  def refresh(self) -> bool:
    a = bytes(self.packed)
    struct.pack_into(
      self.PKR.fmt,
      self.packed,
      0,
      *self.sensor._raw_accel_data,
      *self.sensor._raw_gyro_data,
      *self.sensor._raw_temp_data)
    return self.packed != a 

  def scale(self, fmt: str, value: int) -> float:
    if fmt == 'accel':
      return self.sensor._scale_xl_data(value)
    if fmt == 'gyro':
      return self.sensor._scale_gyro_data(value)
    if fmt == 'temp':
      return value / self.TEMPERATURE_SENSITIVITY + self.TEMPERATURE_OFFSET
    raise ValueError(f'{fmt}')

  def deinit(self):
    if self.onboard_i2c:
      self._obbus.deinit()
      self._obpwr.value = False
      self._obpwr.deinit()

class Imu9Attr(CompAttr):
  src: str

class IMU9(DeviceComponent):
  PKR = Pkr('<')
  SCALE_ACCEL = SCALE_GRAVITY = 1 / 100
  SCALE_GYRO = 0.001090830782496456
  SCALE_MAG = SCALE_EULER = 1 / 16
  SCALE_QUAT = 1 / (1 << 14)
  ATTRMAP: dict[str, CompAttr] = CompAttr.makeattrs(PKR, OrderedDict(
    calflag=dict(fmt='B'),
    accel=dict(src='acceleration', fmt='3h', scale=SCALE_ACCEL),
    mag=dict(src='magnetic', fmt='3h', scale=SCALE_MAG),
    gyro=dict(fmt='3h', scale=SCALE_GYRO),
    euler=dict(fmt='3h', scale=SCALE_EULER),
    quaternion_euler_zyx=dict(fmt='3h', scale=SCALE_EULER),
    quaternion=dict(fmt='4h', scale=SCALE_QUAT),
    gravity=dict(fmt='3h', scale=SCALE_GRAVITY),
    linearaccel=dict(src='linear_acceleration', fmt='3h', scale=SCALE_ACCEL),
    temperature=dict(fmt='b'),
    mode=dict(fmt='B', writeable=True),
    # ---- these require switch to config mode with delay
    offsets_accel=dict(src='offsets_accelerometer', fmt='3h', writeable=True),
    offsets_mag=dict(src='offsets_magnetometer', fmt='3h', writeable=True),
    offsets_gyro=dict(src='offsets_gyroscope', fmt='3h', writeable=True),
    radius_accel=dict(src='radius_accelerometer', fmt='h'),
    radius_mag=dict(src='radius_magnetometer', fmt='h'),
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
  PERSIST_NS = 0x4939
  PERSIST_VER = 0x03
  MIN_REFRESH_INTERVAL = 90

  @property
  def refresh_interval(self) -> int:
    return max(self.MIN_REFRESH_INTERVAL, self._refresh_interval)

  @refresh_interval.setter
  def refresh_interval(self, value: int) -> None:
    self._refresh_interval = value

  class RefreshState:
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

  def __init__(
    self,
    i2c: busio.I2C|None = None,
    address: int = 0x29,
    refresh_interval: int = 200,
    **config,
  ) -> None:
    super().__init__(i2c, address)
    self.sensor = self.SensorCls(self.bus, address, defer_init=True)
    self.refresh_interval = refresh_interval
    self.packed = bytearray(self.PKR.size)
    self.refresh_state = self.RefreshState()
    self.refresh_state.it = self.sensor.yinit(**config)

  def __getitem__(self, name: str):
    if name in self.FLAGMAP:
      flagdef = self.FLAGMAP[name]
      return (self[flagdef[1]] >> flagdef[2]) & flagdef[3]
    return self.ATTRMAP[name].unpack_from(self.packed)

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
      next_mode = self.sensor.CONFIG_MODE
    elif stage == 2:
      slcinfo = self.SLCINFO_CONFIG
      next_mode = state.mode_restore
      state.mode_restore = None

    a = self.packed[slcinfo.slc]
    for attr in slcinfo.attrs:
      value = getattr(self.sensor, attr.src or attr.name)
      attr.pack_into(self.packed, value)

    state.it = self.sensor.yset_mode(next_mode)
    state.next_stage = stage + 1
    # trigger first yield
    state.ready()
    return state.is_init and self.packed[slcinfo.slc] != a

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

  @classmethod
  def SensorCls(cls, *args, **kw):
    from adafruit_bno055 import BNO055_I2C as SensorBase
    from adafruit_bno055 import _ReadOnlyUnaryStruct
    from adafruit_bus_device.i2c_device import I2CDevice

    PAGE_REGISTER = 0x07
    CALIBRATION_REGISTER = 0x35
    MODE_REGISTER = 0x3d
    TRIGGER_REGISTER = 0x3f

    class SensorCls(SensorBase):
      CONFIG_MODE = 0x00
      COMPASS_MODE = 0x09
      NDOF_MODE = 0x0c

      def __init__(self, i2c: busio.I2C, address: int = 0x28, defer_init: bool = False) -> None:
        if defer_init:
          self.buffer = bytearray(2)
          self.i2c_device = I2CDevice(i2c, address)
        else:
          super().__init__(i2c, address)

      @property
      def mode(self):
        return super().mode

      @mode.setter
      def mode(self, new_mode: int) -> None:
        for _ in self.yset_mode(new_mode):
          time.sleep(0.001)

      def yset_mode(self, new_mode: int) -> Generator[None]:
        if self.mode == new_mode:
          return
        if self.mode != self.CONFIG_MODE:
          self._write_register(MODE_REGISTER, self.CONFIG_MODE)
          yield from ysleep(0.019)
        if new_mode != self.CONFIG_MODE:
          self._write_register(MODE_REGISTER, new_mode)
          yield from ysleep(0.07)

      def yreset(self):
        yield from self.yset_mode(self.CONFIG_MODE)
        try:
          # reset
          self._write_register(TRIGGER_REGISTER, 0x20)
        except OSError:
          pass
        yield from ysleep(0.7)
        self.set_normal_mode()
        self._write_register(PAGE_REGISTER, 0x00)
        self._write_register(TRIGGER_REGISTER, 0x00)

      def yconfig(self, mode: int|None = None, **config):
        mode = mode or self.mode
        if config:
          yield from self.yset_mode(self.CONFIG_MODE)
          for name, value in config.items():
            setattr(self, name, value)
        yield from self.yset_mode(mode)

      def yinit(self, mode: int = NDOF_MODE, **config):
        yield from self.yreset()
        yield from self.yconfig(mode=mode, **config)

      calflag = _ReadOnlyUnaryStruct(CALIBRATION_REGISTER, 'B')

      @property
      def quaternion_euler_zyx(self) -> tuple[float, float, float]:
        """
        Source: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
        Addison L. Sears-Collins
        """
        w, x, y, z = self.quaternion
        if w is None or x is None or y is None or z is None:
          return 0.0, 0.0, 0.0
        # Roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        # Pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        # Yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return tuple(abs(math.degrees(x)) for x in (yaw, pitch, roll))

    cls.SensorCls = SensorCls
    return SensorCls(*args, **kw)


"""
if True:
  s._write_register(0x3d, 0x00)
  time.sleep(0.019)
  s._read_register(0x67)
  s._write_register(0x3d, 0x09)
  time.sleep(0.065)
  s._read_register(0x34)

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