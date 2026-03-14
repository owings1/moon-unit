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
  components: dict[int, Component]|None = None
  addrs: dict[str, int] = None
  sdcard: sdcardio.SDCard|None = None
  buses: list[busio.I2C|busio.SPI]|None = None

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

  def init(self, *names) -> None:
    self.deinit()
    self.buses = []
    if settings.sd_enabled:
      spi = board.SPI()
      self.sdcard = sdcardio.SDCard(spi, as_pin(settings.sd_cs))
      storage.mount(storage.VfsFat(self.sdcard), settings.sd_mountpath)
      self.buses.append(spi)
    self.components = OrderedDict()
    self.addrs = OrderedDict()
    for defn in settings.components:
      if defn.get('disabled') or not defn.get('enabled', True):
        continue
      name = defn['name']
      if names and name not in names:
        continue
      debug(f'Init component {name=}')
      module = component_categories[defn['category']]
      cls: type[Component] = getattr(module, defn['classname'])
      options: dict = defn.get('options', {})
      component = cls(**options)
      if component.bus and component.bus not in self.buses:
        self.buses.append(component.bus)
      component.debug = defn.get('debug')
      component.persist_id = defn.get('persist_id')
      self.add_component(component, name)
    for component in self.components.values(): 
      component.app_init(self)
    for component in self.components.values(): 
      component.app_ready(self)

  def get_component(self, ref: int|str) -> Component:
    if isinstance(ref, str):
      ref = self.addrs[ref]
    return self.components[ref]

  def add_component(self, component: Component, name: str):
    if component.component_address in self.components:
      raise ValueError(f'Duplicate address: {component.component_address}')
    if not name:
      raise ValueError(f'Invalid component name: {name}')
    if name in self.addrs:
      raise ValueError(f'Duplicate component name: {name}')
    if component.debug is None:
      component.debug = settings.debug
    self.components[component.component_address] = component
    self.addrs[name] = component.component_address

  def deinit(self) -> None:
    if self.components:
      for component in self.components.values():
        component.deinit()
    self.components = self.addrs = None
    if self.sdcard:
      try:
        storage.umount(settings.sd_mountpath)
      except OSError:
        pass
      self.sdcard.deinit()
    self.sdcard = None
    if self.buses:
      for bus in self.buses:
        bus.deinit()
    self.buses = None

app = App()
