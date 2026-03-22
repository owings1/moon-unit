from __future__ import annotations

import struct
import time
from collections import OrderedDict, namedtuple
import math

import board
import busio
from digitalio import DigitalInOut, Direction
from micropython import const
from utils import Pkr, debug

from . import CompAttr, DeviceComponent, Component

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
  last_was_change = False

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
    declination_degrees=dict(fmt='e', src='self'),
    # euler_heading=dict(fmt='f', src='self'),
    quaternion=dict(fmt='4e'),
    heading=dict(fmt='f', src='self'),
    gravity=dict(fmt='3e'),
    linear_acceleration=dict(fmt='3e'),
    temperature=dict(fmt='b'),
    mode=dict(fmt='B', writeable=True),
    # ---- these require switch to config mode with delay
    offsets_accelerometer=dict(fmt='3h', writeable=True),
    offsets_magnetometer=dict(fmt='3h', writeable=True),
    offsets_gyroscope=dict(fmt='3h', writeable=True),
    radius_accelerometer=dict(fmt='h', writeable=True),
    radius_magnetometer=dict(fmt='h', writeable=True),
  ))
  FLAGMAP = OrderedDict((x[0], x) for x in (
    ('cal_mag', 'calflag', 0x0, 0x3),
    ('cal_accel', 'calflag', 0x2, 0x3),
    ('cal_gyro', 'calflag', 0x4, 0x3),
    ('cal_sys', 'calflag', 0x6, 0x3),
  ))
  SLCINFO_DATA = CompAttr.sliceinfo(ATTRMAP, 0, -5)
  SLCINFO_CONFIG = CompAttr.sliceinfo(ATTRMAP, -5, None)
  SLCINFO_PERSIST = CompAttr.sliceinfo(ATTRMAP, -6, None)
  PERSIST_NS = 0x4939
  PERSIST_VER = 0x04
  MIN_REFRESH_INTERVAL = 90
  CONFIG_MODE = 0x00

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

  def is_refresh_needed(self):
    if self.refresh_state.waiting():
      return self.refresh_state.ready()
    return super().is_refresh_needed()
  
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
      state.last_was_change = False
      return is_first_data

    if stage == 1:
      slcinfo = self.SLCINFO_DATA
      state.mode_restore = self.sensor.mode
      next_mode = self.CONFIG_MODE
    elif stage == 2:
      slcinfo = self.SLCINFO_CONFIG
      next_mode = state.mode_restore
      state.mode_restore = None

    change = False
    for attr in slcinfo.attrs:
      prev = change or self[attr.name]
      if attr.src == 'self':
        value = getattr(self, attr.name)
      else:
        value = getattr(self.sensor, attr.name)
      attr.pack_into(self.packed, value)
      change = change or prev != self[attr.name]

    state.it = self.sensor.yset_mode(next_mode)
    state.next_stage = stage + 1
    # trigger first yield
    state.ready()

    # Only return True for change once per cycle to reduce unnecessary load
    if stage == 1:
      state.last_was_change = change
      return False
    return state.is_init and (change or state.last_was_change)

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

  @property
  def heading(self):
    return self.quaternion_heading

  @property
  def quaternion_heading(self) -> float:
    w, x, y, z = self['quaternion']
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    mag_heading = math.degrees(math.atan2(t3, t4))
    mag_heading *= -1
    mag_heading = (mag_heading + 360) % 360
    return (mag_heading + self.declination_degrees) % 360

  @property
  def euler_heading(self):
    return (self['euler'][0] + self.declination_degrees) % 360

  @property
  def declination_degrees(self) -> float:
    return self.app.system_data.get('gps:wmm_declination', 0.0)

class Imu9Pair(namedtuple('Imu9Pair', ('a', 'b'))):
  a: IMU9
  b: IMU9

class IMU9OrthoPair(Component):
  PKR = Pkr('<')
  ATTRMAP: dict[str, CompAttr] = CompAttr.makeattrs(PKR, OrderedDict(
    a_calflag=dict(fmt='B'),
    a_quaternion=dict(fmt='4e'),
    a_heading=dict(fmt='f'),
    b_calflag=dict(fmt='B'),
    b_quaternion=dict(fmt='4e'),
    b_heading=dict(fmt='f'),
    delta=dict(fmt='f'),
    calibrated_offset=dict(fmt='f', writeable=True),
    heading=dict(fmt='f'),
    declination_degrees=dict(fmt='e'),
  ))
  FLAGMAP = OrderedDict((x[0], x) for x in (
    ('a_cal_mag', 'a_calflag', 0x0, 0x3),
    ('a_cal_accel', 'a_calflag', 0x2, 0x3),
    ('a_cal_gyro', 'a_calflag', 0x4, 0x3),
    ('a_cal_sys', 'a_calflag', 0x6, 0x3),
    ('b_cal_mag', 'b_calflag', 0x0, 0x3),
    ('b_cal_accel', 'b_calflag', 0x2, 0x3),
    ('b_cal_gyro', 'b_calflag', 0x4, 0x3),
    ('b_cal_sys', 'b_calflag', 0x6, 0x3),
  ))

  def __init__(
    self,
    a_address: int = 0x2800,
    b_address: int = 0x2900,
    calibrated_offset: float = 90.0,
    address: int = 0x1260,
    refresh_interval: int = 200,
  ) -> None:
    self.packed = bytearray(self.PKR.size)
    self._addrs = a_address, b_address
    self.component_address = address
    self.refresh_interval = refresh_interval
    self.write('calibrated_offset', calibrated_offset)

  def setup(self, app):
    self.pair = Imu9Pair(
      app.get_component(self._addrs[0]),
      app.get_component(self._addrs[1]))
    del self._addrs

  def refresh(self) -> bool:
    change = False
    for attr in self.ATTRMAP.values():
      if attr.writeable:
        continue
      prev = change or self[attr.name]
      if attr.name.startswith('a_'):
        value = self.pair.a[attr.name[2:]]
      elif attr.name.startswith('b_'):
        value = self.pair.b[attr.name[2:]]
      else:
        value = getattr(self, attr.name)
      attr.pack_into(self.packed, value)
      change = change or prev != self[attr.name]
    return change

  def write(self, name: str, *v) -> None:
    attr = self.ATTRMAP[name]
    if not attr.writeable:
      raise ValueError(f'{name} is read-only')
    attr.pack_into(self.packed, v)

  @property
  def heading(self) -> float:
    adj_b = (self['b_heading'] + self['calibrated_offset']) % 360
    rad_a = math.radians(self['a_heading'])
    rad_b = math.radians(adj_b)
    avg_x = (math.cos(rad_a) + math.cos(rad_b)) / 2
    avg_y = (math.sin(rad_a) + math.sin(rad_b)) / 2
    return math.degrees(math.atan2(avg_y, avg_x)) % 360

  @property
  def delta(self) -> float:
    return (self['a_heading'] - self['b_heading']) % 360

  @property
  def declination_degrees(self) -> float:
    return self.app.system_data.get('gps:wmm_declination', 0.0)

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