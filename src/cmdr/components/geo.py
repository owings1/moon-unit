from __future__ import annotations

from collections import OrderedDict
from microcontroller import Pin
import board
import busio
import gc
import time
from digitalio import Direction, DigitalInOut, Pull
from utils import Pkr, as_pin, debug, millis, ysleep

from . import CompAttr, Component, DeviceComponent

try:
  from typing import Sequence
except ImportError:
  pass

class GPS(DeviceComponent):
  PKR = Pkr('<')
  ATTRMAP: dict[str, CompAttr] = CompAttr.makeattrs(PKR, OrderedDict(
    fix_quality=dict(fmt='B'),
    latitude=dict(fmt='f'),
    longitude=dict(fmt='f'),
    track_angle_deg=dict(fmt='f'), # RMC
    altitude_m=dict(fmt='f'),      # GGA
    timestamp=dict(fmt='Q'),       # RMC
    wmm_declination=dict(fmt='f'),
    wmm_dip_angle=dict(fmt='f'),
  ))
  SLCINFO_PERSIST = CompAttr.sliceinfo(ATTRMAP, -2, -1)
  PERSIST_NS = 0x2360
  PERSIST_VER = 0x01

  def __init__(
    self,
    bus: busio.I2C|None = None,
    address: int = 0x10,
    refresh_interval: int = 1000,
    refresh_interval_nofix: int|None = None,
    refresh_interval_fix: int|None = None,
    read_timeout: float = 0.05,
  ) -> None:
    from sensors.gtop import Gtop
    super().__init__(bus, address)
    self.sensor = Gtop(
      self.bus,
      address=address,
      timeout=read_timeout)
    self.refresh_interval = refresh_interval
    self.refresh_interval_nofix = refresh_interval_nofix or self.refresh_interval
    self.refresh_interval_fix = refresh_interval_fix or self.refresh_interval
    self.packed = bytearray(self.PKR.size)

  def run(self):
    self.sensor.update()
    super().run()

  def refresh(self) -> bool:
    change = False
    for attr in self.ATTRMAP.values():
      prev = change or self[attr.name]
      value = getattr(self.sensor, attr.name, 0) or 0
      attr.pack_into(self.packed, value)
      change = change or attr.name != 'timestamp' and prev != self[attr.name]
    if self['fix_quality']:
      self.refresh_interval = self.refresh_interval_fix
      if self.sensor.wmm_declination is not None:
        self.app.system_data['gps:wmm_declination'] = self.sensor.wmm_declination
    else:
      self.refresh_interval = self.refresh_interval_nofix
    return change

  def dump_persistent(self):
    if self['fix_quality'] and self['wmm_declination']:
      return self.packed[self.SLCINFO_PERSIST.slc]

  def load_persistent(self, buf):
    slcinfo = self.SLCINFO_PERSIST
    attr = slcinfo.attrs[0]
    value = attr.unpack_from(buf, -slcinfo.slc.start)
    if value and not self[attr.name] and not self.app.system_data.get(f'gps:{attr.name}'):
      self.app.system_data[f'gps:{attr.name}'] = value

class GPS2(DeviceComponent):
  PKR = Pkr('<')
  ATTRMAP: dict[str, CompAttr] = CompAttr.makeattrs(PKR, OrderedDict(
    timestamp_epoch=dict(fmt='Q'),
    latitude=dict(fmt='f'),
    longitude=dict(fmt='f'),
    altitude_m=dict(fmt='f'),
    height_geoid=dict(fmt='f'),
    speed_knots=dict(fmt='f'),
    track_angle_deg=dict(fmt='f'),
    hdop=dict(fmt='f'),
    vdop=dict(fmt='f'),
    fix_data=dict(fmt='B'),
    declination=dict(fmt='f'),
    dip_angle=dict(fmt='f'),
    intensity=dict(fmt='f'),
    horiz_intensity=dict(fmt='f'),
    vert_intensity=dict(fmt='f'),
    status_flags=dict(fmt='B'),
    update_counter=dict(fmt='H'),
    tbd_0x40=dict(fmt='B'),
    gsv_msg_cycle=dict(fmt='B'),
    sat01=dict(fmt='2BbHB'),
    sat02=dict(fmt='2BbHB'),
    sat03=dict(fmt='2BbHB'),
    sat04=dict(fmt='2BbHB'),
    sat05=dict(fmt='2BbHB'),
    sat06=dict(fmt='2BbHB'),
    checksum=dict(fmt='H'),
  ))
  ON = False
  OFF = True
  def __init__(
    self,
    scl: str|Pin|None = None,
    sda: str|Pin|None = None,
    frequency: int = 20_000,
    address: int = 0x11,
    pin_desired: str|Pin = 'D0',
    pin_available: str|Pin = 'D1',
    refresh_interval: int = 5000,
  ) -> None:
    bus = busio.I2C(
      as_pin(scl or board.SCL),
      as_pin(sda or board.SDA),
      frequency=frequency)
    super().__init__(bus, address)
    self.refresh_interval = refresh_interval
    self.packed = bytearray(self.PKR.size)
    self.desired = DigitalInOut(as_pin(pin_desired))
    self.available = DigitalInOut(as_pin(pin_available))
    self.available.direction = Direction.INPUT
    self.available.pull = Pull.UP
    self.desired.direction = Direction.OUTPUT
    self.desired.value = self.OFF
    self.refrit = None
    self.refready = False
    self.crcprev = None

  def run(self) -> None:
    if self.refrit:
      try:
        next(self.refrit)
      except StopIteration:
        pass
    super().run()

  def is_refresh_needed(self) -> bool:
    return self.refready or super().is_refresh_needed()

  def refresh(self):
    if self.refrit:
      try:
        next(self.refrit)
      except StopIteration:
        self.refready = False
        self.refrit = None
        return self.crcprev != self['checksum']
      else:
        return False
    self.refrit = self.refrgen()
    next(self.refrit)
    return False

  def refrgen(self):
    self.refready = False
    self.desired.value = self.ON
    yield from ysleep(0.01)
    if self.available.value is self.OFF:
      debug(f'waiting')
      while self.available.value is self.OFF:
        yield from ysleep(0.005)
    debug(f'reading')
    self.crcprev = self['checksum']
    with self.device as device:
      device.write_then_readinto(b'\x00', self.packed)
    self.desired.value = self.OFF
    yield
    if self.verify_crc16():
      debug(f'checksum passed')
    else:
      print(f'checksum failed')
    self.refready = True

  def verify_crc16(self):
    crc = 0xFFFF
    polynomial = 0x1021
    for i in range(self.ATTRMAP['checksum'].start):
      crc ^= (self.packed[i] << 8)
      for _ in range(8):
        if crc & 0x8000:
          crc = (crc << 1) ^ polynomial
        else:
          crc <<= 1
        crc &= 0xFFFF
    return crc == self['checksum']

  def deinit(self):
    self.desired.value = self.OFF
    self.available.deinit()
    self.desired.deinit()

class GPSTarget(Component):
  ATTRMAP = GPS2.ATTRMAP
  ON = False
  OFF = True
  def __init__(
    self,
    addresses: Sequence[int] = (0x11,),
    scl: str|Pin|None = None,
    sda: str|Pin|None = None,
    tx: str|Pin|None = None,
    rx: str|Pin|None = None,
    receiver_buffer_size: int = 0x200,
    sensor_debug: bool = False,
    pin_desired: str|Pin = 'D0',
    pin_available: str|Pin = 'D1',
    cmd_interval_ms: int = 10_000,
    refresh_interval: int = 100,
  ) -> None:
    self.available = DigitalInOut(as_pin(pin_available))
    self.available.direction = Direction.OUTPUT
    self.available.value = self.OFF
    from sensors.gpsplus import GPS
    from i2ctarget import I2CTarget
    from contrib.wmm import WMMv2
    from contrib.wmmcof import wmm_cof
    self.component_address = (addresses[0] << 0x08) | 0xFE
    self.sensor_uart: busio.UART = busio.UART(
      as_pin(tx or board.TX),
      as_pin(rx or board.RX),
      receiver_buffer_size=receiver_buffer_size)
    self.refresh_interval = refresh_interval
    self.sensor = GPS(
      self.sensor_uart,
      wmm=WMMv2(*wmm_cof()),
      debug=sensor_debug)
    self.device = I2CTarget(
      as_pin(scl or board.SCL),
      as_pin(sda or board.SDA),
      addresses)
    self.ptr = 0x00
    self.wbuf = bytearray(1)
    self.desired = DigitalInOut(as_pin(pin_desired))
    self.desired.direction = Direction.INPUT
    self.desired.pull = Pull.UP
    self.packed = self.sensor.buf
    self.cmd_interval_ms = max(2_000, cmd_interval_ms)
    self.next_cmd_at = millis()
    self.next_cmd_idx = 0
    self.cmds = (
      # b'$PMTK251,9600*17\r\n',
      # b'$PMTK251,115200*1F\r\n',
      b'$PMTK314,0,1,1,1,5,5,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n',
      # b'$PMTK314,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n',
      # b'$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n',
      # b'$PMTK220,1000*1F\r\n',
      b'$PMTK220,5000*1B\r\n',
      b'$PMTK300,1000,0,0,0,0*1C\r\n',
      # b'$PMTK313,1*2E\r\n',
      # b'$PMTK301,2*2E\r\n',
      )
    
  def run(self):
    req = self.device.request()
    if not req:
      if self.desired.value is self.OFF:
        super().run()
      self.available.value = self.ON
      return
    with req:
      if req.is_read:
        self.wbuf[0] = self.packed[self.ptr]
        req.write(self.wbuf)
        self.ptr = (self.ptr + 1) % len(self.packed)
      else:
        data = req.read(1)
        if not data:
          return
        self.ptr = data[0] % len(self.packed)

  def refresh(self):
    if self.desired.value is self.ON:
      return False
    self.available.value = self.OFF
    if self.sensor.debug:
      time.sleep(0.02)
      gc.collect()
      time.sleep(0.02)
    self.check_send_command()
    return self.sensor.update()

  def check_send_command(self):
    if millis() > self.next_cmd_at:
      self.sensor.write(self.cmds[self.next_cmd_idx])
      self.next_cmd_idx = (self.next_cmd_idx + 1) % len(self.cmds)
      self.next_cmd_at = millis() + self.cmd_interval_ms

  def deinit(self):
    try:
      self.device.deinit()
    except AttributeError:
      pass
    try:
      self.available.deinit()
    except AttributeError:
      pass
    try:
      self.desired.deinit()
    except AttributeError:
      pass
    try:
      self.sensor_uart.deinit()
    except AttributeError:
      pass

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
    from sensors.hmc5883l import HMC5883L
    super().__init__(bus, address)
    self.sensor = HMC5883L(self.bus, address)
    self.refresh_interval = refresh_interval
    self.packed = bytearray(self.PKR.size)

  def refresh(self) -> bool:
    change = False
    for attr in self.ATTRMAP.values():
      prev = change or self[attr.name]
      attr.pack_into(self.packed, getattr(self.sensor, attr.name))
      change = change or prev != self[attr.name]
    return change

  @property
  def declination_degrees(self) -> float:
    return self.app.system_data.get('gps:wmm_declination', 0.0)

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
    from sensors.qmc5883l import QMC5883L
    super().__init__(bus, address)
    self.sensor = QMC5883L(self.bus, address)
    self.refresh_interval = refresh_interval
    self.packed = bytearray(self.PKR.size)

  def refresh(self) -> bool:
    change = False
    for attr in self.ATTRMAP.values():
      prev = change or self[attr.name]
      attr.pack_into(self.packed, getattr(self.sensor, attr.name))
      change = change or prev != self[attr.name]
    return change

  @property
  def declination_degrees(self) -> float:
    return self.app.system_data.get('gps:wmm_declination', 0.0)
