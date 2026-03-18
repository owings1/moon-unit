from __future__ import annotations

from collections import OrderedDict

import busio
from utils import Pkr

from . import CompAttr, DeviceComponent


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

  def refresh_if_needed(self) -> int:
    return super().refresh_if_needed() or self.sensor.update() and 0

  def refresh(self) -> bool:
    self.sensor.update()
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
