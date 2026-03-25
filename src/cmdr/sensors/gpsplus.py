"""
GPS+ UART to Buffer

* Author: Doug Owings
* License: MIT License

Copyright (C) 2026 Doug Owings. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
---------------------------------------------------------------------
Modified from original source, including parsing routines, from:

`adafruit_gps`

v3.11.6
https://github.com/adafruit/Adafruit_CircuitPython_GPS.git
Copyright (C) 2017 Tony DiCola for Adafruit Industries
Copyright (C) 2021 James Carr
MIT License
---------------------------------------------------------------------
"""
from __future__ import annotations

import struct
import time
import traceback

from micropython import const

try:
    from typing import Any, Iterable, Generator
    from busio import UART
    from circuitpython_typing import ReadableBuffer
    from contrib.wmm import WMMv2
except ImportError:
    pass

_BOM = const(b'<')

class PackedField:
  def __init__(self, fmt: bytes, offset: int):
    self.fmt = _BOM+fmt
    self.offset = offset

  def __get__(self, obj: GPS, objtype=None):
    value = struct.unpack_from(self.fmt, obj.buf, self.offset)
    return value[0] if len(value) == 1 else value

  def __set__(self, obj: GPS, value):
    if value is None:
      value = 0
    if isinstance(value, tuple):
      struct.pack_into(self.fmt, obj.buf, self.offset, *value)
    else:
      struct.pack_into(self.fmt, obj.buf, self.offset, value or 0)

_GLL = const(0)
_RMC = const(1)
_GGA = const(2)
_GSA = const(3)
_GSA_4_11 = const(4)
_GSV7 = const(5)
_GSV11 = const(6)
_GSV15 = const(7)
_GSV19 = const(8)
_RMC_4_1 = const(9)
_VTG = const(10)
_WMM_THRESHOLD = const(0.01) # ~1.1km threshold
CONSTELLATIONS = {
  b'GP': 0,
  b'GL': 1,
  b'GA': 2,
  b'GB': 3,
  b'GQ': 4,
  b'GI': 5,
  b'GN': 6}
_SENTENCE_PARAMS = (
  # 0 - _GLL
  b'dcdcscC',
  # 1 - _RMC
  b'scdcdcffsDCC',
  # 2 - _GGA
  b'sdcdciiffsfsIS',
  # 3 - _GSA
  b'ciIIIIIIIIIIIIfff',
  # 4 - _GSA_4_11
  b'ciIIIIIIIIIIIIfffS',
  # 5 - _GSV7
  b'iiiiiiI',
  # 6 - _GSV11
  b'iiiiiiIiiiI',
  # 7 - _GSV15
  b'iiiiiiIiiiIiiiI',
  # 8 - _GSV19
  b'iiiiiiIiiiIiiiIiiiI',
  # 9 - _RMC_4_1
  b'scdcdcffsDCCC',
  # 10 - _VTG
  b'fcFCfcfcC',
)

_SATTABLE_IDX = const(0x42)
_SATTABLE_MAX = const(6)
_CHECKSUM_IDX = const(0x66)


class GPS:
  """GPS parsing module.  Can parse simple NMEA data sentences from serial
  GPS modules to read latitude, longitude, and more.
  """
  timestamp_epoch = PackedField(b'Q', 0x00)
  "Timestamp in UTC"
  latitude = PackedField(b'f', 0x08)
  "Degrees latitude"
  longitude = PackedField(b'f', 0x0C)
  "Degrees latitude"
  altitude_m = PackedField(b'f', 0x10)
  "Antenna altitude relative to mean sea level"
  height_geoid = PackedField(b'f', 0x14)
  "Geoidal separation relative to WGS 84"
  speed_knots = PackedField(b'f', 0x18)
  "Ground speed in knots"
  track_angle_deg = PackedField(b'f', 0x1C)
  "Track angle in degrees"
  hdop = PackedField(b'f', 0x20)
  "Horizontal dilution of precision (GSA)"
  vdop = PackedField(b'f', 0x24)
  "Vertical dilution of precision"
  fix_data = PackedField(b'B', 0x28)
  'Fix quality lower 4 bits, satellites upper 4 bits'
  declination = PackedField(b'f', 0x29)
  dip_angle = PackedField(b'f', 0x2D)
  intensity = PackedField(b'f', 0x31) # nT
  horizontal_intensity = PackedField(b'f', 0x35) # nT
  vertical_intensity = PackedField(b'f', 0x39) # nT
  status_flags = PackedField(b'B', 0x3D)
  '[0:A/V, 1:M/A, 2:2D/3D, 3:DRDY]'
  update_counter = PackedField(b'H', 0x3E)
  # total_mess_num = PackedField(b'B', 0x40)
  # "Number of messages"
  # mess_num = PackedField(b'B', 0x41)
  # "Message number"
  gsv_msg_cycle = PackedField(b'B', 0x41)
  "GSV Message Cycle Total (0-3) / Current (4-7)"
  # 0x42 - 0x65 Sat Table
  checksum = PackedField(b'H', _CHECKSUM_IDX)
  "Data block CRC-16 checksum"

  """
  ====================================================================================
  IDX | HEX  | SIZE | TYPE    | NAME             | DESCRIPTION
  ----+------+------+---------+------------------+------------------------------------
  00  | 0x00 |  8   |  Q      | timestamp_epoch  | Unix Epoch
  08  | 0x08 |  4   |  f      | latitude         | WGS-84 Decimal Degrees
  12  | 0x0C |  4   |  f      | longitude        | WGS-84 Decimal Degrees
  16  | 0x10 |  4   |  f      | altitude_m       | MSL Altitude (Meters)
  20  | 0x14 |  4   |  f      | height_geoid     | Geoid Separation (Meters)
  24  | 0x18 |  4   |  f      | speed_knots      | Ground Speed (Knots)
  28  | 0x1C |  4   |  f      | track_angle_deg  | Track Angle (Degrees True)
  32  | 0x20 |  4   |  f      | hdop             | Horizontal Dilution (GSA)
  36  | 0x24 |  4   |  f      | vdop             | Vertical Dilution (GSA)
  40  | 0x28 |  1   |  B      | fix_data         | [0-3: Fix Qual, 4-7: Sat Count]
  41  | 0x29 |  4   |  f      | declination      | WMM Magnetic Declination (Deg)
  45  | 0x2D |  4   |  f      | dip_angle        | WMM Magnetic Dip Angle (Deg)
  49  | 0x31 |  4   |  f      | intensity        | WMM Total Intensity (nT)
  53  | 0x35 |  4   |  f      | horiz_intensity  | WMM Horizontal Intensity (nT)
  57  | 0x39 |  4   |  f      | vert_intensity   | WMM Vertical Intensity (nT)
  61  | 0x3D |  1   |  B      | status_flags     | [0:A/V, 1:M/A, 2-3:3D, 4:DRDY]
  62  | 0x3E |  2   |  H      | update_counter   | Data-Ready Latch (Increments)
  ----+------+------+---------+------------------+------------------------------------
  64  | 0x40 |  1   |  B      |                  | [TBD]
  65  | 0x41 |  1   |  B      | gsv_msg_cycle    | GSV Message [0-3: Total, 4-7: Current]
  66  | 0x42 |  36  | 2BbHB*6 | Sat Table        | 6 Slots (CID, PRN, Elev, Azim, SNR)
  102 | 0x66 |  2   |  H      | checksum         | CRC-16-CCITT over 0x00-0x66
  ====================================================================================
  """

  @property
  def timestamp_utc(self):
    ts = self.timestamp_epoch
    return time.localtime(ts) if ts > 0 else None

  @timestamp_utc.setter
  def timestamp_utc(self, value):
    if value is None:
      self.timestamp_epoch = 0
    else:
      self.timestamp_epoch = int(time.mktime(value))

  @property
  def fix_quality(self):
    """
    GPS quality indicator

      0 - fix not available
      1 - GPS fix
      2 - Differential GPS fix (values above 2 are 2.3 features)
      3 - PPS fix
      4 - Real Time Kinematic
      5 - Float RTK
      6 - estimated (dead reckoning)
      7 - Manual input mode
      8 - Simulation mode
    """
    return self.fix_data & 0x0F

  @fix_quality.setter
  def fix_quality(self, value):
    # Clear lower 4 bits, then set new value
    clean_val = (value or 0) & 0x0F
    self.fix_data = (self.fix_data & 0xF0) | clean_val

  @property
  def satellites(self):
    # Extract upper 4 bits
    return (self.fix_data >> 4) & 0x0F

  @satellites.setter
  def satellites(self, value):
    # Clear upper 4 bits, then set new value (shifted)
    clean_val = ((value or 0) & 0x0F) << 4
    self.fix_data = (self.fix_data & 0x0F) | clean_val

  @property
  def fix_quality_3d(self):
    """
    1 - no fix, 2 - 2D fix, 3 - 3D fix
    Stored in Bits 2-3 of _status_raw
    """
    # Extract bits 2 and 3, then shift down and add 1
    return ((self.status_flags >> 2) & 0x03) + 1

  @fix_quality_3d.setter
  def fix_quality_3d(self, value):
    # Ensure value is 1-3, subtract 1 to fit in 2 bits (0-2)
    val = (max(1, min(3, int(value or 1))) - 1) & 0x03
    # Clear bits 2-3 and set new value
    self.status_flags = (self.status_flags & ~(0x03 << 2)) | (val << 2)

  @property
  def data_ready(self):
    """
    Low-level flag: 1 = Buffer has been populated at least once.
    """
    return bool(self.status_flags & 0x10)

  @data_ready.setter
  def data_ready(self, value: bool):
    if value:
      self.status_flags |= 0x10
    else:
      self.status_flags &= ~0x10

  @property
  def isactivedata(self):
    "Status Valid(A) or Invalid(V)"
    return 'A' if (self.status_flags & 0x01) else 'V'

  @isactivedata.setter
  def isactivedata(self, value):
    if value == 'A':
      self.status_flags |= 0x01
    else:
      self.status_flags &= ~0x01

  @property
  def sel_mode(self):
    "Selection mode ('A': Automatic, 'M': Manual)"
    return 'A' if (self.status_flags & 0x02) else 'M'

  @sel_mode.setter
  def sel_mode(self, value):
    if value == 'A':
      self.status_flags |= 0x02
    else:
      self.status_flags &= ~0x02

  @property
  def has_fix(self) -> bool:
    """True if a current fix for location information is available."""
    return self.fix_quality >= 1

  @property
  def has_3d_fix(self) -> bool:
    """Returns true if there is a 3d fix available.
    use has_fix to determine if a 2d fix is available,
    passing it the same data"""
    return self.fix_quality_3d >= 2

  @property
  def gsv_msg_total(self):
    return self.gsv_msg_cycle & 0x0F

  @gsv_msg_total.setter
  def gsv_msg_total(self, value):
    # Clear lower 4 bits, then set new value
    clean_val = (value or 0) & 0x0F
    self.gsv_msg_cycle = (self.gsv_msg_cycle & 0xF0) | clean_val

  @property
  def gsv_msg_current(self):
    return (self.gsv_msg_cycle >> 4) & 0x0F

  @gsv_msg_current.setter
  def gsv_msg_current(self, value):
    # Clear upper 4 bits, then set new value (shifted)
    clean_val = ((value or 0) & 0x0F) << 4
    self.gsv_msg_cycle = (self.gsv_msg_cycle & 0x0F) | clean_val

  def __init__(
    self,
    uart: UART,
    *,
    read_buffer_max_size: int = 512,
    debug: bool = False,
    wmm: WMMv2|None = None,
  ) -> None:
    """
    UART should have a receiver_buffer_size of 256-512.
    """
    self._uart = uart
    self.wmm = wmm
    self.buf = bytearray(0x68)
    self._sats = None  # Temporary holder for information from GSV messages
    self.sats = None
    "Information from GSV messages"
    # self.sat_prns = None
    # "Satellite pseudorandom noise code"
    self._mode_indicator = None
    self._magnetic_variation = None
    self.horizontal_dilution = None
    self.pdop = None
    self.debug = debug
    "Print incoming data sentence to the console"
    self.readbuf = bytearray()
    self.read_buffer_max_size = read_buffer_max_size
    self._paramarr = [None] * 19

  def update(self) -> bool:
    """
    Check for updated data from the GPS module and process it
    accordingly, updating the buffer registers. Returns True if
    new data was processed, and False if nothing new was received.
    """
    # Grab a sentence and check its data type to call the appropriate
    # parsing function.
    sentence = self._parse_sentence()
    if not sentence:
      return False
    data_type, raw_args = sentence # raw_args is still the full string
    if len(data_type) < 5:
      self.debug and print(f'bad {data_type=}')
      return False
    dt_bytes = data_type.upper().encode("ascii")
    talker, s_type = _parse_talker(dt_bytes)
    if talker not in CONSTELLATIONS:
      self.debug and print(f'bad {talker=}')
      return False
    self.debug and print(sentence)
    method = getattr(self, f'_parse_{s_type.decode().lower()}', None)
    if not method:
      return False
    try:
      length = _split_into(raw_args, ',', self._paramarr)
    except IndexError:
      return False
    try:
      result = method(talker, self._paramarr, length)
    except ValueError:
      return False
    if result:
      self._update_wmm()
      self._mark_updated()
      print(f'_mode_indicator={self._mode_indicator}')
    return result

  def _mark_updated(self) -> None:
    if not self.data_ready:
      self.data_ready = True
    self.update_counter = (self.update_counter + 1) % 0x10000
    self._update_checksum()

  def _update_checksum(self) -> None:
    """
    Calculates CRC-16 over the first 102 bytes and packs it at 102-103

      def verify_crc16(data):
        crc = 0xFFFF
        polynomial = 0x1021
        
        # Calculate over the telemetry and satellite table (102 bytes)
        for i in range(0x66):
          crc ^= (data[i] << 8)
          for _ in range(8):
            if crc & 0x8000:
              crc = (crc << 1) ^ polynomial
            else:
              crc <<= 1
            crc &= 0xFFFF
        # Unpack the checksum sent by the GPS (Little Endian)
        sent_crc = struct.unpack_from('<H', data, 102)[0]
        return crc == sent_crc
    """
    crc = 0xFFFF
    for i in range(_CHECKSUM_IDX):
      byte = self.buf[i]
      crc ^= (byte << 8)
      for _ in range(8):
        if crc & 0x8000:
          crc = (crc << 1) ^ 0x1021
        else:
          crc <<= 1
        crc &= 0xFFFF
    self.checksum = crc

  def send_command(self, command: bytes, add_checksum: bool = True) -> None:
    """Send a command string to the GPS.  If add_checksum is True (the
    default) a NMEA checksum will automatically be computed and added.
    Note you should NOT add the leading $ and trailing * to the command
    as they will automatically be added!
    """
    self.write(b"$")
    self.write(command)
    if add_checksum:
      checksum = 0
      for char in command:
        checksum ^= char
      self.write(b"*")
      self.write(bytes(f"{checksum:02x}".upper(), "ascii"))
    self.write(b"\r\n")

  def write(self, bytestr: ReadableBuffer) -> int|None:
    """Write a bytestring data to the GPS directly, without parsing
    or checksums"""
    return self._uart.write(bytestr)

  def readline(self) -> bytes|None:
    """
    Non-blocking readline using read buffer
    """
    if (in_waiting := self._uart.in_waiting) > 0:
      clip = self.read_buffer_max_size - in_waiting
      if len(self.readbuf) > clip:
        self.readbuf = self.readbuf[-clip:]
      self.readbuf.extend(self._uart.read(in_waiting))
    idx = self.readbuf.find(b'\n')
    if idx == -1:
      return
    line = bytes(self.readbuf[:idx+1])
    self.readbuf = self.readbuf[idx+1:]
    return line

  def _read_sentence(self) -> str|None:
    line = self.readline() 
    if not line:
      return None
    if len(line) < 9: # $GPGSV*CS\r\n is ~9 min
      return
    # Standard NMEA ends in *CS\r\n (checksum is 5th-to-last byte)
    if line[-5] != 0x2A: # ord('*')
      return None
    # Validate Checksum
    try:
      expected = int(line[-4:-2], 16)
      actual = 0
      # XOR from index 1 ($) to the asterisk
      for i in range(1, len(line) - 5):
        actual ^= line[i]
      if actual != expected:
        if self.debug:
          print(f"bad checksum {actual=} {expected=} {line=}")
        return None
      sentence = line.decode("ascii").strip()
      # self._raw_sentence = sentence
      return sentence
    except (ValueError, UnicodeError) as err:
      return None

  def _parse_sentence(self) -> tuple[str, str]|None:
    sentence = self._read_sentence()
    # sentence is a valid NMEA with a valid checksum
    if sentence is None:
      return None
    # Remove checksum once validated.
    sentence = sentence[:-3]
    # Parse out the type of sentence (first string after $ up to comma)
    # and then grab the rest as data within the sentence.
    delimiter = sentence.find(",")
    if delimiter == -1:
      return None  # Invalid sentence, no comma after data type.
    data_type = sentence[1:delimiter]
    return (data_type, sentence[delimiter + 1 :])

  def _update_timestamp_utc(self, time_utc: str, date: str|None = None) -> None:
    if date is None:
      ts = self.timestamp_utc
      if ts is None:
        return
      day, month, year = ts.tm_mday, ts.tm_mon, ts.tm_year
    else:
      day = int(date[0:2])
      month = int(date[2:4])
      year = 2000 + int(date[4:6])
    try:
      hours = int(time_utc[0:2])
      mins = int(time_utc[2:4])
      secs = int(time_utc[4:6])
      self.timestamp_utc = (year, month, day, hours, mins, secs, 0, 0, -1)
    except (OverflowError, ValueError):
      # Catch any weird GPS glitch dates (like 00/00/00)
      pass

  def _parse_vtg(self, talker: bytes, data: list[str], length: int) -> bool:
    # VTG - Course Over Ground and Ground Speed
    _parse_data_into(_VTG, data, data, length)
    # Track made good, degrees true
    self.track_angle_deg = data[0]
    # Speed over ground, knots
    self.speed_knots = data[4]
    # Speed over ground, kilometers / hour
    self.speed_kmh = data[6]
    # Parse FAA mode indicator
    self._mode_indicator = data[8]
    return True

  def _parse_gll(self, talker: bytes, data: list[str], length: int) -> bool:
    # GLL - Geographic Position - Latitude/Longitude
    _parse_data_into(_GLL, data, data, length)
    # Latitude
    self.latitude = _read_degrees(data[0], data[1])
    # Longitude
    self.longitude = _read_degrees(data[2], data[3])
    # UTC time of position
    self._update_timestamp_utc(data[4])
    # Status Valid(A) or Invalid(V)
    self.isactivedata = data[5]
    # Parse FAA mode indicator
    self._mode_indicator = data[6]
    return True

  def _parse_rmc(self, talker: bytes, data: list[str], length: int) -> bool:
    # RMC - Recommended Minimum Navigation Information
    if length == 12:
      st = _RMC
    elif length == 13:
      st = _RMC_4_1
    else:
      return False # Unexpected number of params.
    try:
      _parse_data_into(st, data, data, length)
    except ValueError:
      self.fix_quality = 0
      return False  # Params didn't parse
    # UTC time of position and date
    self._update_timestamp_utc(data[0], data[8])
    # Status Valid(A) or Invalid(V)
    self.isactivedata = data[1]
    if data[1].lower() == "a":
      if self.fix_quality == 0:
        self.fix_quality = 1
    else:
      self.fix_quality = 0
    # Latitude
    # self.latitude = _read_degrees(data, 2, "s")
    self.latitude = _read_degrees(data[2], data[3])
    # Longitude
    # self.longitude = _read_degrees(data, 4, "w")
    self.longitude = _read_degrees(data[4], data[5])
    # Speed over ground, knots
    self.speed_knots = data[6]
    # Track made good, degrees true
    self.track_angle_deg = data[7]
    # Magnetic variation
    if data[9] is None or data[10] is None:
      self._magnetic_variation = None
    else:
      # self._magnetic_variation = _read_degrees(data, 9, "w")
      self._magnetic_variation = _read_degrees(data[9], data[10])
    # Parse FAA mode indicator
    self._mode_indicator = data[11]
    return True

  def _parse_gga(self, talker: bytes, data: list[str], length: int) -> bool:
    # GGA - Global Positioning System Fix Data
    if length != 14:
      return False  # Unexpected number of params.
    try:
      _parse_data_into(_GGA, data, data, length)
    except ValueError:
      self.fix_quality = 0
      return False  # Params didn't parse
    # UTC time of position
    self._update_timestamp_utc(data[0])
    # Latitude
    self.latitude = _read_degrees(data[1], data[2])
    # Longitude
    self.longitude = _read_degrees(data[3], data[4])
    # GPS quality indicator
    self.fix_quality = data[5]
    # Number of satellites in use, 0 - 12
    self.satellites = data[6]
    # Horizontal dilution of precision
    self.horizontal_dilution = data[7]
    # Antenna altitude relative to mean sea level
    # self.altitude_m = _parse_float(data[8])
    self.altitude_m = data[8]
    # data[9] - antenna altitude unit, always 'M' ???
    # Geoidal separation relative to WGS 84
    # self.height_geoid = _parse_float(data[10])
    self.height_geoid = data[10]
    # data[11] - geoidal separation unit, always 'M' ???
    # data[12] - Age of differential GPS data, can be null
    # data[13] - Differential reference station ID, can be null
    return True

  def _parse_gsa(self, talker: bytes, data: list[str], length: int) -> bool:
    # GSA - GPS DOP and active satellites
    try:
      if length == 17:
        _parse_data_into(_GSA, data, data, length)
      elif length == 18:
        _parse_data_into(_GSA_4_11, data, data, length)
      else:
        return False # Unexpected number of params.
    except ValueError:
      self.fix_quality_3d = 0
      return False  # Params didn't parse
    talker = str(talker, "ascii")
    # Selection mode: 'M' - manual, 'A' - automatic
    self.sel_mode = data[0]
    # Mode: 1 - no fix, 2 - 2D fix, 3 - 3D fix
    self.fix_quality_3d = data[1]
    # self.sat_prns = [f"{talker}{sat}" for sat in filter(None, data[2:-4])]
    # PDOP, dilution of precision
    self.pdop = data[14]
    # HDOP, horizontal dilution of precision
    self.hdop = data[15]
    # VDOP, vertical dilution of precision
    self.vdop = data[16]
    # data[17] - System ID
    return True

  def _parse_gsv(self, talker: bytes, data: list[str], length: int) -> bool:
    # GSV - Satellites in view
    if length == 7:
      st = _GSV7
    elif length == 11:
      st = _GSV11
    elif length == 15:
      st = _GSV15
    elif length == 19:
      st = _GSV19
    else:
      return False
    _parse_data_into(st, data, data, length)
    talker = str(talker, "ascii")
    # Number of messages
    self.gsv_msg_total = data[0]
    # Message number
    self.gsv_msg_current = data[1]
    # Number of satellites in view
    self.satellites = data[2]
    sat_tup = data[3:]
    satlist = []
    timestamp = time.monotonic()
    for i in range(len(sat_tup) // 4):
      j = i * 4
      value = (
        # Satellite number
        f"{talker}{sat_tup[0 + j]}",
        # Elevation in degrees
        sat_tup[1 + j] or 0,
        # Azimuth in degrees
        sat_tup[2 + j] or 0,
        # signal-to-noise ratio in dB
        sat_tup[3 + j] or 0,
        # Timestamp
        timestamp,
      )
      satlist.append(value)

    if self._sats is None:
      self._sats = []
    for value in satlist:
      self._sats.append(value)

    if self.gsv_msg_current == self.gsv_msg_total:
      # Last part of GSV message
      if len(self._sats) == self.satellites:
        # Transfer received satellites to self.sats
        if self.sats is None:
          self.sats = {}
        else:
          # Remove all satellites which haven't
          # been seen for 30 seconds
          timestamp = time.monotonic()
          old = []
          for sat_id, sat_data in self.sats.items():
            if (timestamp - sat_data[4]) > 30:
              old.append(sat_id)
          for i in old:
            self.sats.pop(i)
        for sat in self._sats:
          self.sats[sat[0]] = sat
      self._sats.clear()
      if self.sats is not None:
        self._sync_sats_to_buffer()
    return True

  def _sync_sats_to_buffer(self) -> None:
    """
    Flattens the self.sats dict into 6 fixed 6-byte slots.
    Table starts at 0x42.
    """
    # Sort by SNR, take top 6 to fit 6-byte slots in 36 bytes
    active_sats = sorted(self.sats.values(), key=lambda x: x[3], reverse=True)[:_SATTABLE_MAX]
    for i in range(_SATTABLE_MAX):
      offset = _SATTABLE_IDX + i * 6
      if i < len(active_sats):
        # sat = ("GP01", elev, azim, snr, timestamp)
        name, elev, azim, snr, _ = active_sats[i]
        # Extract talker (first 2 chars) and PRN (rest)
        talker_bytes = name[:2].encode('ascii')
        cid = CONSTELLATIONS.get(talker_bytes, 7)
        prn = int(''.join(filter(str.isdigit, name)))
        # Pack: CID(B), PRN(B), Elev(b), Azim(H), SNR(B)
        struct.pack_into(b'<2BbHB', self.buf, offset, cid, prn, elev, azim, snr)
      else:
        # Clear slot
        struct.pack_into(b'<2BbHB', self.buf, offset, 0, 0, 0, 0, 0)

  def _update_wmm(self) -> None:
    """
    Update calculation of declination angle if needed.
    """
    if self._should_calc_wmm():
      altkm = self.altitude_m / 1000
      res = self.wmm.observe(self.latitude, self.longitude, self.timestamp_utc, altkm)
      self.declination = res.declination
      self.dip_angle = res.dip_angle
      self.intensity = res.intensity
      self.horiz_intensity = res.horizontal_intensity
      self.vert_intensity = res.vertical_intensity

  def _should_calc_wmm(self) -> bool:
    """
    Check if we've moved enough to justify a re-calculation
    of declination angle.
    """
    if not (
      self.wmm and
      self.fix_quality and
      self.latitude and
      self.longitude and
      self.timestamp_epoch
    ):
      return False
    return (
      abs(self.latitude - self.wmm.olat) > _WMM_THRESHOLD or
      abs(self.longitude - self.wmm.olon) > _WMM_THRESHOLD)


# Internal helper parsing functions.
# These handle input that might be none or null and return none instead of
# throwing errors.
def _parse_degrees(value: str) -> int|None:
  # Parse a NMEA lat/long data pair 'dddmm.mmmm' into a pure degrees value.
  # Where ddd is the degrees, mm.mmmm is the minutes.
  if len(value) < 3:
    return None
  idx = value.find('.')
  if idx == -1:
    return
  rawdeg = value[:idx]
  rawmin = value[idx+1:]
  if rawmin.find('.') > -1:
    return
  # To avoid losing precision handle degrees and minutes separately
  # Return the final value as an integer. Further functions can parse
  # this into a float or separate parts to retain the precision
  degrees = int(rawdeg) // 100 * 1_000_000  # the ddd
  minutes = int(rawdeg) % 100  # the mm.
  minutes += int(f'{rawmin[:4]:0<4}') / 10_000
  minutes = int((minutes * 1_000_000) / 60)
  return degrees + minutes

def _parse_int(value: str|bytes) -> int|None:
  return int(value) if value else None

def _parse_float(value: str|bytes) -> float|None:
  return float(value) if value else None

def _read_degrees(value: float, dirchar: str|bytes) -> float:
  return (
    (value / 1_000_000) *
    # -1 for s or w, else 1
    #   (ord(w) is ord(s) + 4, so normalize capitals with |0x20
    #    then make s into w with |x04. works for str and bytes)
    (1 - ((ord(dirchar) | 0x24 == 0x77) << 1))
  )

def _parse_talker(data_type: bytes) -> tuple[bytes, bytes]:
  # Split the data_type into talker and sentence_type
  if data_type[:1] == b'P':  # Proprietary codes
    return (data_type[:1], data_type[1:])
  return (data_type[:2], data_type[2:])

_PARAM_PARSERS = {
  # 'd'
  0x64: _parse_degrees,
  # 'f'
  0x66: _parse_float,
  # 'i'
  0x69: _parse_int,
  # 's'
  0x73: None,
  # 'c'
  0x63: None,
}

def _parse_data_into(sentence_type: int, data: list[str|bytes], target: list, length: int) -> None:
  for i, value in enumerate(_param_parser(sentence_type, data, length)):
    target[i] = value

def _param_parser(sentence_type: int, data: list[str|bytes], length: int) -> Generator[Any]:
  """
  Parse sentence data for the specified sentence type
  """
  try:
    param_types = _SENTENCE_PARAMS[sentence_type]
  except KeyError:
    raise ValueError from None
  if len(param_types) != length:
    # The expected number does not match the number of data items
    raise ValueError
  for i, param_type in enumerate(param_types):
    yield _parse_param(param_type, data[i])

def _parse_param(param_type: int, value: str|bytes) -> Any:
  try:
    parser = _PARAM_PARSERS[param_type|0x20]
  except KeyError:
    raise TypeError(f'Unexpected {param_type=}')
  if not (value or param_type & 0x20):
    return None
  if param_type|0x20 == 0x63 and len(value) != 1: # 'c' or 'C'
    raise ValueError(f'{param_type=} {value=}')
  if parser:
    return parser(value)
  return value

def _split_into(s: str|bytes, d: str|bytes, arr, offset: int = 0) -> int:
  count = 0
  start = 0
  while True:
    try:
      end = s.index(d, start)
      arr[count+offset] = s[start:end]
      start = end + len(d)
    except ValueError:
      arr[count+offset] = s[start:]
      break
    finally:
      count += 1
  return count
