from __future__ import annotations

import time
from collections import OrderedDict, deque

from micropython import const
import busio
from utils import Pkr, debug, millis

from . import CompAttr, DeviceComponent


# References:
#
# https://github.com/adafruit/Adafruit_GPS/blob/master/src/Adafruit_PMTK.h
#
# http://www.hhhh.org/wiml/proj/nmeaxor.html

PMTK_SET_NMEA_OUTPUT_GGAONLY = const(b'PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
'GPGGA only'
PMTK_SET_NMEA_OUTPUT_RMCONLY = const(b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
'GPRMC only'
PMTK_SET_NMEA_OUTPUT_RMCGGA = const(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
'GPRMC + GPGGA'
PMTK_SET_NMEA_OUTPUT_RMCGGAGSA = const(b'PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
'GPRMC + GPGGA + GPGSA'
PMTK_SET_NMEA_OUTPUT_ALLDATA = const(b'PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0')
'All data'

PMTK_SET_NMEA_UPDATE_1HZ = const(b'PMTK220,1000')
PMTK_API_SET_FIX_CTL_1HZ = const(b'PMTK300,1000,0,0,0,0')

INIT_CMDS = (
  PMTK_SET_NMEA_OUTPUT_RMCGGA,
  PMTK_SET_NMEA_UPDATE_1HZ,
  PMTK_API_SET_FIX_CTL_1HZ)

class GPS(DeviceComponent):
  PKR = Pkr('<')
  SCALE_LAT = 1 / (2_147_483_000 / 90)
  SCALE_LON = 1 / (2_147_483_000 / 180)
  SCALE_DEG = 1 / (4_294_966_000 / 360)
  SCALE_ALT = 1 / 10
  ATTRMAP: dict[str, CompAttr] = CompAttr.makeattrs(PKR, OrderedDict(
    fix_quality=dict(src=('sensor', 'fix_quality'), fmt='B'),
    latitude=dict(src=('sensor', 'latitude'), fmt='l', scale=SCALE_LAT),
    longitude=dict(src=('sensor', 'longitude'), fmt='l', scale=SCALE_LON),
    track_angle=dict(src=('sensor', 'track_angle_deg'), fmt='L', scale=SCALE_DEG), # RMC
    altitude=dict(src=('sensor', 'altitude_m'), fmt='l', scale=SCALE_ALT),         # GGA
    timestamp=dict(src=('sensor', 'timestamp_utc'), fmt='Q'),                      # RMC
    wmm_declination=dict(src=('wmmcalc', 'declination'), fmt='l', scale=SCALE_DEG),
    wmm_dip_angle=dict(src=('wmmcalc', 'dip_angle'), fmt='l', scale=SCALE_DEG),
    # wmm_intensity=dict('wmm_intensity', 'wmmcalc', 'intensity', 'f'),
    # wmm_horizontal_intensity=dict('wmm_horizontal_intensity', 'wmmcalc', 'horizontal_intensity', 'f'),
    # wmm_north_intensity=dict('wmm_north_intensity', 'wmmcalc', 'north_intensity', 'f'),
    # wmm_east_intensity=dict('wmm_east_intensity', 'wmmcalc', 'east_intensity', 'f'),
    # wmm_vertical_intensity=dict('wmm_vertical_intensity', 'wmmcalc', 'vertical_intensity', 'f'),
  ))
  CHANGED_SLC = slice(None, ATTRMAP['timestamp'].end)
  cmdwait = 1000

  def __init__(
    self,
    bus: busio.I2C|None = None,
    address: int = 0x10,
    refresh_interval: int = 1000,
    refresh_interval_nofix: int|None = None,
    refresh_interval_fix: int|None = None,
    read_timeout: float = 0.05,
  ) -> None:
    from adafruit_gps import GPS_GtopI2C
    super().__init__(bus, address)
    self.sensor = GPS_GtopI2C(
      self.bus,
      address=address,
      debug=False,
      timeout=read_timeout)
    self.refresh_interval = refresh_interval
    self.refresh_interval_nofix = refresh_interval_nofix or self.refresh_interval
    self.refresh_interval_fix = refresh_interval_fix or self.refresh_interval
    self.packed = bytearray(self.PKR.size)
    self.cmdtodo = deque(INIT_CMDS, len(INIT_CMDS))
    from contrib.wmm import WMMv2
    from contrib.wmmcof import wmm_cof
    self.wmm = WMMv2(*wmm_cof())

  @property
  def wmmcalc(self):
    return self.wmm.calculation

  def __getitem__(self, name: str) -> int|float|None:
    value = self.ATTRMAP[name].unpack_from(self.packed)
    if not (value or name == 'fix_quality' or self.packed[0]):
      return None
    return value

  def refresh_if_needed(self) -> int:
    return super().refresh_if_needed() or self.sensor.update() and 0

  def refresh(self) -> bool:
    self.sensor.update()
    if self.cmdtodo and self.refreshed_at < millis() - self.cmdwait:
      self.sensor.send_command(self.cmdtodo.popleft())
      return False
    a = self.packed[self.CHANGED_SLC]
    for attr in self.ATTRMAP.values():
      src = attr.src or ('sensor', attr.name)
      value = getattr(getattr(self, src[0]), src[1], 0) or 0
      if attr.name == 'timestamp':
        # This indicates we are done reading sensor attributes, and will
        # proceed with wmm attributes, so now is an efficient time to
        # refresh the wmm calculation without double-reading sensor values
        # or delaying to the next refresh when values may be stale.
        if isinstance(value, time.struct_time):
          if self['fix_quality'] & 1 and value.tm_year:
            altkm = self['altitude'] / 1000
            self.wmm.observe(self['latitude'], self['longitude'], decimyear(value), altkm)
          value = value.tm_year and time.mktime(value)
      attr.pack_into(self.packed, value)
    if self['fix_quality']:
      self.refresh_interval = self.refresh_interval_fix
    else:
      self.refresh_interval = self.refresh_interval_nofix
    return self.packed[self.CHANGED_SLC] != a

def decimyear(value: time.struct_time) -> float:
  return value.tm_year + value.tm_yday / (366 - bool(value.tm_year % 4))

class MagnetometerHMC(DeviceComponent):
  PKR = Pkr('<')
  ATTRMAP: dict[str, CompAttr] = CompAttr.makeattrs(PKR, OrderedDict(
    flags=dict(fmt='B'),
    magnetic=dict(fmt='3e'),
    gain=dict(fmt='H'),
    offset=dict(fmt='3e'),
    scale=dict(fmt='3e'),
  ))
  FLAGMAP = OrderedDict((x[0], x) for x in (
    ('overflow', 'flags', 0x5, 0x1),
    ('calibrated', 'flags', 0x6, 0x1),
  ))

  def __init__(
    self,
    bus: busio.I2C|None = None,
    address: int = 0x1e,
    refresh_interval: int = 1000
  ) -> None:
    from hmc5883l import HMC5883L
    super().__init__(bus, address)
    self.sensor = HMC5883L(self.bus, address)
    self.refresh_interval = refresh_interval
    self.packed = bytearray(self.PKR.size)

  def __getitem__(self, name: str):
    if name in self.FLAGMAP:
      flagdef = self.FLAGMAP[name]
      return (self[flagdef[1]] >> flagdef[2]) & flagdef[3]
    return self.ATTRMAP[name].unpack_from(self.packed)

  def refresh(self) -> bool:
    a = bytes(self.packed)
    for attr in self.ATTRMAP.values():
      attr.pack_into(self.packed, getattr(self.sensor, attr.name))
    return self.packed != a

class MagnetometerQMC(DeviceComponent):
  PKR = Pkr('<')
  ATTRMAP: dict[str, CompAttr] = CompAttr.makeattrs(PKR, OrderedDict(
    flags=dict(fmt='B'),
    magnetic=dict(fmt='3e'),
    temperature=dict(fmt='H'),
    gain=dict(fmt='H'),
    offset=dict(fmt='3e'),
    scale=dict(fmt='3e'),
  ))
  FLAGMAP = OrderedDict((x[0], x) for x in (
    ('overflow', 'flags', 0x5, 0x1),
    ('calibrated', 'flags', 0x6, 0x1),
  ))

  def __init__(
    self,
    bus: busio.I2C|None = None,
    address: int = 0x0d,
    refresh_interval: int = 1000
  ) -> None:
    from qmc5883l import QMC5883L
    super().__init__(bus, address)
    self.sensor = QMC5883L(self.bus, address)
    self.refresh_interval = refresh_interval
    self.packed = bytearray(self.PKR.size)

  def __getitem__(self, name: str):
    if name in self.FLAGMAP:
      flagdef = self.FLAGMAP[name]
      return (self[flagdef[1]] >> flagdef[2]) & flagdef[3]
    return self.ATTRMAP[name].unpack_from(self.packed)

  def refresh(self) -> bool:
    a = bytes(self.packed)
    for attr in self.ATTRMAP.values():
      attr.pack_into(self.packed, getattr(self.sensor, attr.name))
    return self.packed != a
