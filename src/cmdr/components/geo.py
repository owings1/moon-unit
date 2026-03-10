from __future__ import annotations

import time
from collections import OrderedDict, deque

import board
import busio
from utils import millis

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

class GPS(DeviceComponent):
  cmdwait = 1000
  PACKSIZE = 25
  ATTRMAP = OrderedDict((x[0], x) for x in (
    ('fix_quality', 'fix_quality', 0, 1, 'b'),
    ('latitude', 'latitude', 1, 5, 'd'),
    ('longitude', 'longitude', 5, 9, 'd'),
    ('track_angle', 'track_angle_deg', 9, 13, 'D'), # RMC
    ('altitude', 'altitude_m', 13, 17, 'a'),        # GGA
    ('timestamp', 'timestamp_utc', 17, 25, 't'),    # RMC
  ))

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
    self.packed = bytearray(self.PACKSIZE)
    self.cmdtodo = deque(INIT_CMDS, len(INIT_CMDS))

  def __getitem__(self, name: str) -> int|float|None:
    attrdef = self.ATTRMAP[name]
    buf = self.packed[attrdef[2]:attrdef[3]]
    value = unpack(attrdef[4], buf)
    if not (value or name == 'fix_quality' or self.packed[0]):
      return None
    return value

  def refresh_if_needed(self) -> int:
    return super().refresh_if_needed() or self.gtop.update() and 0

  def refresh(self) -> bool:
    gtop = self.gtop
    gtop.update()
    if self.cmdtodo and self.refreshed_at < millis() - self.cmdwait:
      gtop.send_command(self.cmdtodo.popleft())
      return False
    a = bytes(self.packed[:17])
    for attrdef in self.ATTRMAP.values():
      value = getattr(gtop, attrdef[1])
      self.packed[attrdef[2]:attrdef[3]] = pack(attrdef[4], value)
    return self.packed[:17] != a

def pack(type: str, value: int|float|time.struct_time) -> bytes:
  if type == 'd':
    # decimal degrees -180 to 180
    return round(((value or 0) + 180) * 10_000_000).to_bytes(4)
  if type == 'D':
    # decimal degrees 0 to 360
    return round((value or 0) * 10_000_000).to_bytes(4)
  if type == 'a':
    # altitude meters +/-
    return (round((value or 0) * 10) + 10_000_000).to_bytes(4)
  if type == 'b':
    return int(value or 0).to_bytes(1)
  if type == 't':
    return int(value and value.tm_year and time.mktime(value) or 0).to_bytes(8)
  raise ValueError(f'{type=}')

def unpack(type: str, buf: bytes) -> int|float:
  if type == 'd':
    return int.from_bytes(buf) / 10_000_000.0 - 180
  if type == 'D':
    return int.from_bytes(buf) / 10_000_000.0
  if type == 'a':
    return (int.from_bytes(buf) - 10_000_000) / 10.0
  if type == 'b':
    return int.from_bytes(buf)
  if type == 't':
    return int.from_bytes(buf)
  raise ValueError(f'{type=}')
