from __future__ import annotations

import board
import busio
from utils import as_pin, debug, settings, millis

try:
  import sdcardio
  from components import Component
except ImportError:
  pass

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
    if not self.components:
      return
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
      import sdcardio
      import storage
      spi = board.SPI()
      self.sdcard = sdcardio.SDCard(spi, as_pin(settings.sd_cs))
      storage.mount(storage.VfsFat(self.sdcard), settings.sd_mountpath)
      self.buses.append(spi)
    for defn in settings.components:
      if defn.get('disabled') or not defn.get('enabled', True):
        continue
      name = defn['name']
      if names and name not in names:
        continue
      debug(f'Init component {name=}')
      cls = get_component_class(defn['category'], defn['classname'])
      options: dict = defn.get('options', {})
      component: Component = cls(**options)
      if component.bus and component.bus not in self.buses:
        self.buses.append(component.bus)
      component.debug = defn.get('debug')
      component.persist_id = defn.get('persist_id')
      self.add_component(component, name)
    if self.components:
      for component in self.components.values(): 
        component.app_init(self)
      for component in self.components.values():
        component.app_ready(self)

  def get_component(self, ref: int|str) -> Component:
    if self.components is None:
      raise KeyError(ref)
    if isinstance(ref, str):
      ref = self.addrs[ref]
    return self.components[ref]

  def add_component(self, component: Component, name: str):
    if self.components is None:
      from collections import OrderedDict
      self.components = OrderedDict()
      self.addrs = OrderedDict()
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
      import storage
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

def get_component_class(category: str, classname: str) -> type[Component]:
  if category == 'motors':
    from components import motors as module
  elif category == 'inertial':
    from components import inertial as module
  elif category == 'geo':
    from components import geo as module
  elif category == 'stowage':
    from components import stowage as module
  else:
    raise ValueError(f'{category=}')
  return getattr(module, classname)

app = App()
