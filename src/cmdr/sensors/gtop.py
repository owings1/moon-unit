from __future__ import annotations

import time

import supervisor
from adafruit_gps import GPS_GtopI2C as SensorBase
from micropython import const

try:
  import busio
except ImportError:
  pass

# References:
#
# https://github.com/adafruit/Adafruit_GPS/blob/master/src/Adafruit_PMTK.h
#
# http://www.hhhh.org/wiml/proj/nmeaxor.html

PMTK_SET_NMEA_OUTPUT_RMCGGA = const(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
'GPRMC + GPGGA'
# PMTK_SET_NMEA_OUTPUT_GGAONLY = const(b'PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# 'GPGGA only'
# PMTK_SET_NMEA_OUTPUT_RMCONLY = const(b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# 'GPRMC only'
# PMTK_SET_NMEA_OUTPUT_RMCGGAGSA = const(b'PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# 'GPRMC + GPGGA + GPGSA'
# PMTK_SET_NMEA_OUTPUT_ALLDATA = const(b'PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0')
# 'All data'

PMTK_SET_NMEA_UPDATE_1HZ = const(b'PMTK220,1000')
PMTK_API_SET_FIX_CTL_1HZ = const(b'PMTK300,1000,0,0,0,0')

INIT_CMDS = const((
  PMTK_SET_NMEA_OUTPUT_RMCGGA,
  PMTK_SET_NMEA_UPDATE_1HZ,
  PMTK_API_SET_FIX_CTL_1HZ))
_INIT_WAITMS = const(1000)

class Gtop(SensorBase):

  def __init__(
    self,
    bus: busio.I2C,
    address: int = 0x10,
    debug: bool = False,
    timeout: float = 0.05,
  ) -> None:
    from contrib.wmm import WMMv2
    from contrib.wmmcof import wmm_cof
    super().__init__(bus, address=address, debug=debug, timeout=timeout)
    self._initit = iter(INIT_CMDS)
    self.send_command(next(self._initit))
    self._lastinitat = supervisor.ticks_ms()
    self.wmm = WMMv2(*wmm_cof())

  @property
  def timestamp(self) -> int|None:
    value = self.timestamp_utc
    if value and value.tm_year:
      return int(time.mktime(value))

  @property
  def wmm_declination(self) -> float|None:
    if (calc := self.wmm.calculation):
      return calc.declination

  @property
  def wmm_dip_angle(self) -> float|None:
    if (calc := self.wmm.calculation):
      return calc.dip_angle

  def update(self):
    result = super().update()
    if self._initit:
      if supervisor.ticks_ms() - self._lastinitat < _INIT_WAITMS:
        return False
      try:
        cmd = next(self._initit)
      except StopIteration:
        self._initit = None
        del self._lastinitat
      else:
        self.send_command(cmd)
        self._lastinitat = supervisor.ticks_ms()
        return False
    if result and self.fix_quality & 1:
      if self.timestamp_utc and self.timestamp_utc.tm_year:
        decyear = decimyear(self.timestamp_utc)
        altkm = (self.altitude_m or 0) / 1000
        self.wmm.observe(self.latitude, self.longitude, decyear, altkm)
    return result

def decimyear(value: time.struct_time) -> float:
  return value.tm_year + value.tm_yday / (366 - bool(value.tm_year % 4))