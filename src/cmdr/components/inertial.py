from __future__ import annotations

from collections import OrderedDict
from digitalio import DigitalInOut, Direction

import board
import struct
import busio
import time
import busio

from utils import debug, Pkr
from . import DeviceComponent

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

  def __init__(self, i2c: busio.I2C|None = None, address: int = 0x6a, *, onboard_i2c: bool = False, refresh_interval: int = 200) -> None:
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
    self.sensor = LSM6DS3TRC(i2c, address)
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

class IMU9(DeviceComponent):
  PKR = Pkr()
  SCALE_ACCEL = SCALE_GRAVITY = 1 / 100
  SCALE_GYRO = 0.001090830782496456
  SCALE_MAG = SCALE_EULER = 1 / 16
  SCALE_QUAT = 1 / (1 << 14)
  ATTRMAP = OrderedDict((x[0], x) for x in (
    ('accel', SCALE_ACCEL, PKR.size, PKR.add('3h')),
    ('mag', SCALE_MAG, PKR.size, PKR.add('3h')),
    ('gyro', SCALE_GYRO, PKR.size, PKR.add('3h')),
    ('euler', SCALE_EULER, PKR.size, PKR.add('3h')),
    ('quaternion', SCALE_QUAT, PKR.size, PKR.add('4h')),
    ('gravity', SCALE_GRAVITY, PKR.size, PKR.add('3h')),
    ('linearaccel', SCALE_ACCEL, PKR.size, PKR.add('3h')),
    ('offsets_accel', 1, PKR.size, PKR.add('3h')),
    ('offsets_mag', 1, PKR.size, PKR.add('3h')),
    ('offsets_gyro', 1, PKR.size, PKR.add('3h')),
    ('radius_accel', 1, PKR.size, PKR.add('h')),
    ('radius_mag', 1, PKR.size, PKR.add('h')),
    ('calflag', 1, PKR.size, PKR.add('B')),
    ('temperature', 1, PKR.size, PKR.add('b')),
    ('mode', 1, PKR.size, PKR.add('B')),
  ))
  FLAGMAP = OrderedDict((x[0], x) for x in (
    ('cal_mag', 'calflag', 0x0),
    ('cal_accel', 'calflag', 0x2),
    ('cal_gyro', 'calflag', 0x4),
    ('cal_sys', 'calflag', 0x6),
    ('calibrated', 'calflag', 0x0),
  ))
  ACTMAP = OrderedDict((x[0], x) for x in (
    ('offsets_accel', 'offsets_accelerometer', '3h'),
    ('offsets_mag', 'offsets_magnetometer', '3h'),
    ('offsets_gyro', 'offsets_gyroscope', '3h'),
  ))
  PERSIST_NS = 0x4939
  PERSIST_VER = 0x01
  PERSIST_SLC = slice(ATTRMAP['offsets_accel'][2], ATTRMAP['offsets_accel'][2] + struct.calcsize('9h'))

  def __init__(
    self, i2c: busio.I2C,
    address: int = 0x29,
    *,
    refresh_interval: int = 200,
    offsets: dict[str, tuple[int, int, int]]|None = None,
  ) -> None:
    super().__init__(i2c, address)
    from adafruit_bno055 import BNO055_I2C
    self.sensor = BNO055_I2C(i2c, address)
    self.refresh_interval = refresh_interval
    self.packed = bytearray(self.PKR.size)
    if offsets:
      for name, value in offsets.items():
        self.write(f'offsets_{name}', value)

  def __getitem__(self, name: str):
    if name in self.FLAGMAP:
      flagdef = self.FLAGMAP[name]
      value = self[flagdef[1]] >> flagdef[2]
      if name == 'calibrated':
        return value == 0xff
      else:
        return value & 0x3
    attrdef = self.ATTRMAP[name]
    raw = struct.unpack_from(attrdef[3], self.packed, attrdef[2])
    it = (x * attrdef[1] for x in raw)
    if len(raw) == 1:
      return next(it)
    return tuple(it)

  def refresh(self) -> bool:
    a = bytes(self.packed)
    sensor = self.sensor
    descale = self.descale
    cal = sensor.calibration_status
    struct.pack_into(
      self.PKR.fmt,
      self.packed,
      0,
      *descale(self.SCALE_ACCEL, *sensor.acceleration),
      *descale(self.SCALE_GYRO, *sensor.gyro),
      *descale(self.SCALE_MAG, *sensor.magnetic),
      *descale(self.SCALE_EULER, *sensor.euler),
      *descale(self.SCALE_QUAT, *sensor.quaternion),
      *descale(self.SCALE_GRAVITY, *sensor.gravity),
      *descale(self.SCALE_ACCEL, *sensor.linear_acceleration),
      *sensor.offsets_accelerometer,
      *sensor.offsets_magnetometer,
      *sensor.offsets_gyroscope,
      sensor.radius_accelerometer,
      sensor.radius_magnetometer,
      (cal[0] << 0x6) | (cal[1] << 0x4) | (cal[2] << 0x2) | (cal[3] << 0x0),
      sensor.temperature,
      sensor.mode,)
    return self.packed != a

  def write(self, name: str, value) -> None:
    actdef = self.ACTMAP[name]
    values = value if isinstance(value, tuple) else (value,)
    struct.pack(actdef[2], *values)
    setattr(self.sensor, actdef[1], value)

  def dump_persistent(self):
    if self['calibrated']:
      return self.packed[self.PERSIST_SLC]

  def load_persistent(self, buf: bytes) -> None:
    sensor = self.sensor
    values = struct.unpack('9h', buf)
    sensor.offsets_accelerometer = values[:3]
    sensor.offsets_gyroscope = values[3:6]
    sensor.offsets_magnetometer = values[6:]

  @staticmethod
  def descale(scale: int|float, *values: int|float|None):
    for value in values:
      if value is None:
        value = 0
      value *= scale
      if isinstance(value, float):
        value = round(value)
      yield value