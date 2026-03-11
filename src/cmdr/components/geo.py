from __future__ import annotations

import time
from collections import OrderedDict, deque, namedtuple

import board
import busio
import struct
from utils import millis, debug, Pkr

from . import DeviceComponent

# References:
#
# https://github.com/adafruit/Adafruit_GPS/blob/master/src/Adafruit_PMTK.h
#
# http://www.hhhh.org/wiml/proj/nmeaxor.html

PMTK_SET_NMEA_OUTPUT_GGAONLY = b'PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'
'GPGGA only'
PMTK_SET_NMEA_OUTPUT_RMCONLY = b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'
'GPRMC only'
PMTK_SET_NMEA_OUTPUT_RMCGGA = b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'
'GPRMC + GPGGA'
PMTK_SET_NMEA_OUTPUT_RMCGGAGSA = b'PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0'
'GPRMC + GPGGA + GPGSA'
PMTK_SET_NMEA_OUTPUT_ALLDATA = b'PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0'
'All data'

PMTK_SET_NMEA_UPDATE_1HZ = b'PMTK220,1000'
PMTK_API_SET_FIX_CTL_1HZ = b'PMTK300,1000,0,0,0,0'

INIT_CMDS = (
  PMTK_SET_NMEA_OUTPUT_RMCGGA,
  PMTK_SET_NMEA_UPDATE_1HZ,
  PMTK_API_SET_FIX_CTL_1HZ)

class GpsAttr(namedtuple('GpsAttr', ('name', 'baseobj', 'objattr', 'start', 'end', 'fmt', 'scale'))):
  name: str
  baseobj: str
  objattr: str
  start: int
  end: int
  fmt: str
  scale: int|float

def gattr(pkr: Pkr, name: str, baseobj: str, objattr: str, fmt: str, scale: int|float = 1):
  start = pkr.size
  pkr.add(fmt)
  end = pkr.size
  return GpsAttr(
    name=name,
    baseobj=baseobj,
    objattr=objattr,
    start=start,
    end=end,
    fmt=fmt,
    scale=scale)

class GPS(DeviceComponent):
  PKR = Pkr('<')
  SCALE_LAT = 1 / (2_147_483_000 / 90)
  SCALE_LON = 1 / (2_147_483_000 / 180)
  SCALE_DEG = 1 / (4_294_966_000 / 360)
  SCALE_ALT = 1 / 10
  ATTRMAP: dict[str, GpsAttr] = OrderedDict(
    fix_quality=gattr(PKR, 'fix_quality', 'gtop', 'fix_quality', 'B'),
    latitude=gattr(PKR, 'latitude', 'gtop', 'latitude', 'l', scale=SCALE_LAT),
    longitude=gattr(PKR, 'longitude', 'gtop', 'longitude', 'l', scale=SCALE_LON),
    track_angle=gattr(PKR, 'track_angle', 'gtop', 'track_angle_deg', 'L', scale=SCALE_DEG), # RMC
    altitude=gattr(PKR, 'altitude', 'gtop', 'altitude_m', 'l', scale=SCALE_ALT),            # GGA
    timestamp=gattr(PKR, 'timestamp', 'gtop', 'timestamp_utc', 'Q'),                        # RMC
    wmm_declination=gattr(PKR, 'wmm_declination', 'wmmcalc', 'declination', 'l', scale=SCALE_DEG),
    wmm_dip_angle=gattr(PKR, 'wmm_dip_angle', 'wmmcalc', 'dip_angle', 'l', scale=SCALE_DEG),
    # wmm_intensity=gattr(PKR, 'wmm_intensity', 'wmmcalc', 'intensity', 'f'),
    # wmm_horizontal_intensity=gattr(PKR, 'wmm_horizontal_intensity', 'wmmcalc', 'horizontal_intensity', 'f'),
    # wmm_north_intensity=gattr(PKR, 'wmm_north_intensity', 'wmmcalc', 'north_intensity', 'f'),
    # wmm_east_intensity=gattr(PKR, 'wmm_east_intensity', 'wmmcalc', 'east_intensity', 'f'),
    # wmm_vertical_intensity=gattr(PKR, 'wmm_vertical_intensity', 'wmmcalc', 'vertical_intensity', 'f'),
  )
  CHANGED_SLC = slice(None, ATTRMAP['timestamp'].end)
  cmdwait = 1000

  def __init__(
    self,
    i2c: busio.I2C|None = None,
    address: int = 0x10,
    refresh_interval: int = 1000
  ) -> None:
    from adafruit_gps import GPS_GtopI2C
    self.bus = i2c or board.I2C()
    self.gtop = GPS_GtopI2C(self.bus, address=address, debug=False)
    self.device = self.gtop._i2c
    self.device_address = address
    self.refresh_interval = refresh_interval
    self.packed = bytearray(self.PKR.size)
    self.cmdtodo = deque(INIT_CMDS, len(INIT_CMDS))
    from contrib.wmm import WMMv2
    from contrib.wmmcof import wmm_cof
    self.wmm = WMMv2(*wmm_cof())

  @property
  def wmmcalc(self):
    return self.wmm.calculation

  def __getitem__(self, name: str) -> int|float|None:
    attrdef = self.ATTRMAP[name]
    raw = struct.unpack_from(self.PKR.bom+attrdef.fmt, self.packed, attrdef.start)
    it = (x * attrdef.scale for x in raw)
    if len(raw) == 1:
      value = next(it)
      if not (value or name == 'fix_quality' or self.packed[0]):
        return None
      return value
    return tuple(it)

  def refresh_if_needed(self) -> int:
    return super().refresh_if_needed() or self.gtop.update() and 0

  def refresh(self) -> bool:
    self.gtop.update()
    if self.cmdtodo and self.refreshed_at < millis() - self.cmdwait:
      self.gtop.send_command(self.cmdtodo.popleft())
      return False
    a = bytes(self.packed[self.CHANGED_SLC])
    for attrdef in self.ATTRMAP.values():
      value = getattr(getattr(self, attrdef.baseobj), attrdef.objattr, 0) or 0
      if attrdef.name == 'timestamp':
        # This indicates we are done reading gtop attributes, and will
        # proceed with wmm attributes, so now is an efficient time to
        # refresh the wmm calculation without double-reading gtop values
        # or delaying to the next refresh when values may be stale.
        if isinstance(value, time.struct_time):
          if self['fix_quality'] & 1 and value.tm_year:
            year = float(f'{value.tm_year}.{value.tm_mon}')
            altkm = self['altitude'] / 1000
            self.wmm.observe(self['latitude'], self['longitude'], year, altkm)
          value = value.tm_year and time.mktime(value)
      if attrdef.scale != 1:
        value = round(value / attrdef.scale)
      struct.pack_into(self.PKR.bom+attrdef.fmt, self.packed, attrdef.start, value)
    return  self.packed[self.CHANGED_SLC] != a
