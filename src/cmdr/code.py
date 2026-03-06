from __future__ import annotations

import board
import busio
from components import Component
import components.geo
import components.motors
import components.inertial
from utils import settings, debug
from collections import OrderedDict

class App:
  i2c: busio.I2C|None = None
  components: dict[int, Component]|None = None

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
    for component in self.components.values():
      if component.refresh_if_needed() == 2:
        if component.debug:
          debug()
          for line in component.debug_lines():
            debug(line)
          debug()

  def init(self) -> None:
    self.deinit()
    self.components = OrderedDict()
    for name, defn in settings.components.items():
      if defn.get('disabled') or not defn.get('enabled', True):
        continue
      if defn['category'] == 'motors':
        module = components.motors
      elif defn['category'] == 'geo':
        module = components.geo
      elif defn['category'] == 'inertial':
        module = components.inertial
      else:
        raise ValueError(defn['category'])
      cls: type[Component] = getattr(module, defn['classname'])
      options: dict = defn.get('options', {})
      if cls is components.inertial.IMU6 and options.get('onboard_i2c'):
        i2c = None
      else:
        i2c = self.i2c = board.I2C()
      debug(f'Init {name=} {cls=}')
      component = cls(i2c=i2c, **options)
      component.debug = defn.get('debug')
      if component.debug is None:
        component.debug = settings.debug
      self.add_component(component)

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

app = App()

del(App)

if __name__ == '__main__':
  app.main()
