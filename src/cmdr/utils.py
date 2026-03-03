from __future__ import annotations

import board
import time
from microcontroller import Pin

def as_pin(pin: str|Pin) -> Pin:
  if isinstance(pin, str):
    pin = getattr(board, pin)
  return pin

def millis() -> int:
  return time.monotonic_ns() // 1_000_000

def init_settings(defaults: MT, settings: ModuleType) -> MT:
  for name in defaults.__dict__:
    if not hasattr(settings, name):
      setattr(settings, name, getattr(defaults, name))
  return settings

import defaults
import settings
settings = init_settings(defaults, settings)

def debug(*args, **kw):
  if settings.debug:
    print(*args, **kw)

def i2c_scan():
  i2c = board.I2C()
  while not i2c.try_lock():
    pass
  try:
    for addr in i2c.scan():
      print(f'{hex(addr)}')
  finally:
    i2c.unlock()

# IDE Environment
try:
  from types import ModuleType
  from typing import TypeVar
  MT = TypeVar('MT', bound=ModuleType)
except ImportError:
  pass
