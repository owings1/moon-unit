from __future__ import annotations

import board
import busio
import gc
import utils
from supervisor import ticks_ms
from utils import as_pin, settings

try:
  import sdcardio
  from components import Component
  from typing import Any
except ImportError:
  pass

class App:
  system_data: dict[str, Any]|None = None
  components: tuple[Component, ...]|None = None
  addr_to_indx: dict[int, int]|None = None
  name_to_addr: dict[str, int] = None
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
    stop_at = duration_ms and ticks_ms() + duration_ms
    try:
      while True:
        self.loop()
        if stop_at and ticks_ms() >= stop_at:
          break
    except KeyboardInterrupt:
      print(f'Stopping from Ctrl-C')

  def loop(self) -> None:
    for component in self.components:
      component.run()
    gc.collect()

  def init(self, *names) -> None:
    self.deinit()
    self.system_data = {}
    self.name_to_addr = {}
    self.addr_to_indx = {}
    components = []
    self.buses = []
    if settings.sd_enabled:
      import sdcardio
      import storage
      spi = board.SPI()
      self.sdcard = sdcardio.SDCard(spi, as_pin(settings.sd_cs))
      storage.mount(storage.VfsFat(self.sdcard), settings.sd_mountpath)
      self.buses.append(spi)
    for defn in settings.components:
      defn = dict(defn)
      if defn.pop('disabled', None) or not defn.pop('enabled', True):
        continue
      if names and defn['name'] not in names:
        continue
      component = self.init_component(**defn)
      self.addr_to_indx[component.component_address] = len(components)
      components.append(component)
      gc.collect()
    self.components = tuple(components)
    del components
    gc.collect()
    for component in self.components:
      component.setup(self)
    gc.collect()

  def init_component(
    self,
    *,
    name: str,
    category: str,
    classname: str,
    debug: bool|None = None,
    persist_id: int|None = None,
    options: dict[str, Any]|None = None,
  ):
    if debug is None:
      debug = settings.debug
    if options is None:
      options = {}
    utils.debug(f'Init component {name=}')
    cls = get_component_class(category, classname)
    component: Component = cls(**options)
    component.debug = debug
    component.persist_id = persist_id
    address = component.component_address
    if address in self.addr_to_indx:
      raise ValueError(f'Duplicate address: {address}')
    if not name:
      raise ValueError(f'Invalid component name: {name}')
    if name in self.name_to_addr:
      raise ValueError(f'Duplicate component name: {name}')
    self.name_to_addr[name] = address
    return component

  def get_component(self, ref: int|str) -> Component:
    if isinstance(ref, str):
      ref = self.name_to_addr[ref]
    return self.components[self.addr_to_indx[ref]]

  def deinit(self) -> None:
    if self.components:
      for component in self.components:
        component.deinit()
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
    if self.components:
      for component in self.components:
        if component.bus:
          component.bus.deinit()
    self.components = self.name_to_addr = self.addr_to_indx = None
    self.buses = None
    self.system_data = None

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
