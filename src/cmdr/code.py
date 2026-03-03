from __future__ import annotations

import board
import busio
from adafruit_bus_device.i2c_device import I2CDevice
from components import Component, geo, motors
from utils import settings, debug


class App:
  i2c: busio.I2C|None = None
  mc: motors.Controller|None = None
  gps: geo.GPS|None = None
  # wip ...
  daddr: dict[int, I2CDevice]|None = None
  caddr: dict[int, Component]|None = None

  def main(self) -> None:
    try:
      self.init()
      print(f'Running loop')
      while True:
        self.loop()
    except KeyboardInterrupt:
      print(f'Stopping from Ctrl-C')
    finally:
      self.deinit()

  def loop(self) -> None:
    if self.mc.refresh_if_needed() == 2:
      for m in self.mc.motors:
        debug(f'---')
        debug(f'{type(m).__name__} id={m.id}')
        for k, v in m.items():
          debug(f'{k}={v}')
    if self.gps and self.gps.refresh_if_needed() == 2:
      debug(f'---')
      debug(f'{type(self.gps).__name__}')
      for k, v in self.gps.items():
        debug(f'{k}={v}')

  def init(self) -> None:
    self.deinit()
    self.daddr = {}
    self.caddr = {}
    self.i2c = board.I2C()
    self.mc = motors.Controller(
      i2c=self.i2c,
      address=settings.mc_address,
      refresh_interval=settings.mc_refresh_interval,
      motors=2)
    self.daddr[self.mc.device_address] = self.mc.device
    self.caddr[self.mc.component_address] = self.mc
    for m in self.mc.motors:
      self.caddr[m.component_address] = m
    if settings.gps_enabled:
      self.gps = geo.GPS(
        i2c=self.i2c,
        address=settings.gps_address,
        refresh_interval=settings.gps_refresh_interval)
      self.daddr[self.gps.device_address] = self.gps.device
      self.caddr[self.gps.component_address] = self.gps

  def deinit(self) -> None:
    self.mc = None
    self.gps = None
    self.daddr = None
    self.caddr = None
    if self.i2c:
      self.i2c.deinit()

app = App()

del(App)

if __name__ == '__main__':
  app.main()
