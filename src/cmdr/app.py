from __future__ import annotations

from collections import OrderedDict

import board
import busio
import components.geo
import components.inertial
import components.motors
import components.stowage
import sdcardio
import storage
from components import Component
from utils import as_pin, debug, settings, millis

try:
  from types import ModuleType
except ImportError:
  pass

component_categories: dict[str, ModuleType] = dict(
  geo=components.geo,
  inertial=components.inertial,
  motors=components.motors,
  stowage=components.stowage)

class App:
  i2c: busio.I2C|None = None
  spi: busio.SPI|None = None
  components: dict[int, Component]|None = None
  sdcard: sdcardio.SDCard|None = None

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

  def iloop(self, duration_ms: int|None = None):
    stop_at = duration_ms and millis() + duration_ms
    try:
      while True:
        self.loop()
        if stop_at and millis() >= stop_at:
          break
    except KeyboardInterrupt:
      print(f'Stopping from Ctrl-C')

  def loop(self) -> None:
    for component in self.components.values():
      if component.refresh_if_needed() == 2:
        if component.debug:
          debug()
          for line in component.debug_lines():
            debug(line)
          debug()

  def init(self) -> None:
    self.deinit()
    if settings.sd_enabled:
      self.spi = board.SPI()
      self.sdcard = sdcardio.SDCard(self.spi, as_pin(settings.sd_cs))
      storage.mount(storage.VfsFat(self.sdcard), settings.sd_mountpath)
    self.components = OrderedDict()
    for name, defn in settings.components.items():
      if defn.get('disabled') or not defn.get('enabled', True):
        continue
      debug(f'Init component {name=}')
      module = component_categories[defn['category']]
      cls: type[Component] = getattr(module, defn['classname'])
      options: dict = defn.get('options', {})
      if cls is components.inertial.IMU6 and options.get('onboard_i2c'):
        i2c = None
      else:
        i2c = self.i2c = board.I2C()
      component = cls(i2c=i2c, **options)
      component.debug = defn.get('debug')
      if component.debug is None:
        component.debug = settings.debug
      component.persist_id = defn.get('persist_id')
      self.add_component(component)
    for component in self.components.values():
      component.app_ready(self)

  def add_component(self, component: Component):
    self.components[component.component_address] = component
    for subcomponent in component.subcomponents():
      self.add_component(subcomponent)

  def deinit(self) -> None:
    if self.components:
      for component in self.components.values():
        component.deinit()
    self.components = None
    if self.i2c:
      self.i2c.deinit()
    self.i2c = None
    if self.sdcard:
      try:
        storage.umount(settings.sd_mountpath)
      except OSError:
        pass
      self.sdcard.deinit()
    self.sdcard = None
    if self.spi:
      self.spi.deinit()
    self.spi = None

app = App()
