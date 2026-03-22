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

"""
Packet Type: 314 PMTK_API_SET_NMEA_OUTPUT
0 NMEA_SEN_GLL, // GPGLL interval - Geographic Position - Latitude longitude
1 NMEA_SEN_RMC, // GPRMC interval - Recommended Minimum Specific GNSS Sentence
2 NMEA_SEN_VTG, // GPVTG interval - Course over Ground and Ground Speed
3 NMEA_SEN_GGA, // GPGGA interval - GPS Fix Data
4 NMEA_SEN_GSA, // GPGSA interval - GNSS DOPS and Active Satellites
5 NMEA_SEN_GSV, // GPGSV interval - GNSS Satellites in View
6 //Reserved
7 //Reserved
13 //Reserved
14 //Reserved
15 //Reserved
16 //Reserved
17 //Reserved
18 NMEA_SEN_MCHN, // PMTKCHN interval - GPS channel status

Supported Frequency Setting
0 - Disabled or not supported sentence
1 - Output once every one position fix
2 - Output once every two position fixes
3 - Output once every three position fixes
4 - Output once every four position fixes
5 - Output once every five position fixes

PMTK314,0,1,0,1,5,5,0,0,0,0,0,0,0,0,0,0,0,0,0
"""

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
_CMD_WAITMS = const(1000)
_UPDATE_MINMS = const(1000)

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
    self._cmdit = iter(INIT_CMDS)
    self.send_command(next(self._cmdit))
    self._lastcmdat = supervisor.ticks_ms()
    self._lastresultat = 0
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
    if self._cmdit:
      if supervisor.ticks_ms() - self._lastcmdat < _CMD_WAITMS:
        return False
      try:
        cmd = next(self._cmdit)
      except StopIteration:
        self._cmdit = None
        del self._lastcmdat
      else:
        self.send_command(cmd)
        self._lastcmdat = supervisor.ticks_ms()
        return False
    if supervisor.ticks_ms() - self._lastresultat < _UPDATE_MINMS:
      return False
    result = super().update()
    if result:
      self._lastresultat = supervisor.ticks_ms()
      if self.fix_quality & 1:
        if self.timestamp_utc and self.timestamp_utc.tm_year:
          decyear = decimyear(self.timestamp_utc)
          altkm = (self.altitude_m or 0) / 1000
          self.wmm.observe(self.latitude, self.longitude, decyear, altkm)
    return result

  def _update_timestamp_utc(self, time_utc: str, date: str|None = None) -> None:
    try:
      super()._update_timestamp_utc(time_utc, date)
    except ValueError:
      print(f'{time_utc=} {date=}')
      raise

def decimyear(value: time.struct_time) -> float:
  return value.tm_year + value.tm_yday / (366 - bool(value.tm_year % 4))