from __future__ import annotations

import board
import busio
import time
from components import motors
from utils import as_pin, settings

class App:
  i2c: busio.I2C|None = None
  mc: motors.Controller|None = None

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
    for m in self.mc.motors:
      m.readall()
      print(m.asdict())
      time.sleep(5)
    time.sleep(5)

  def init(self) -> None:
    self.deinit()
    self.i2c = board.I2C()
    self.mc = motors.Controller(
      i2c=self.i2c,
      address=settings.mc_address,
      motors=2)
  
  def deinit(self) -> None:
    self.mc = None
    if self.i2c:
      self.i2c.deinit()

app = App()

del(App)

if __name__ == '__main__':
  app.main()
