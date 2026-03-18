from __future__ import annotations

import binascii
import struct
import traceback
from collections import OrderedDict

import storage

from . import Component

class Persister(Component):

  def __init__(
    self,
    address: int = 0x5,
    refresh_interval: int = 30 * 1000,
    root: str = '/',
    path: str = '/persist',
    readonly: bool = False,
  ) -> None:
    self.component_address = address
    self.refresh_interval = refresh_interval
    self.root = root
    self.path = path
    self.readonly = readonly
    self.vfs = storage.getmount(self.root)
    if self.vfs.readonly and not self.readonly:
      raise Exception(f'readonly file system {root=}')
    self.datamap: dict[tuple[int, int, int], bytearray|bytes] = OrderedDict()
    self.dirty: set[tuple[int, int, int]] = set()
    self.peristables: list[Component] = []

  def app_ready(self, app):
    for component in app.components:
      if component is not self and component.persistable:
        self.peristables.append(component)
        key = component.persistkey
        kbuf = struct.pack('>HBH', *key)
        dirpath = f'{self.path}/{kbuf[:3].hex()}'
        filepath = f'{dirpath}/{kbuf[3:].hex()}'
        try:
          with self.vfs.open(filepath, 'rb') as fp:
            self.datamap[key] = binascii.a2b_base64(fp.read())
        except OSError as err:
          if err.errno != 2:
            raise
        else:
          component.load_persistent(self.datamap[key])

  def refresh(self):
    if self.readonly:
      return False
    for component in self.peristables:
      value = component.dump_persistent()
      if value is not None:
        self[component.persistkey] = value
    if self.dirty:
      self.save_dirty()
      return True
    return False
  
  def __getitem__(self, key: tuple[int, int, int]):
    return self.datamap.get(key)

  def __setitem__(self, key: tuple[int, int, int], value: bytes|bytearray):
    if not (key in self.datamap and self.datamap[key] == value):
      self.datamap[key] = bytes(value)
      self.dirty.add(key)

  def save_dirty(self) -> tuple[int, int]:
    good, bad = 0, 0
    for key in tuple(self.dirty):
      if self.save(key):
        self.dirty.discard(key)
        good += 1
      else:
        bad += 1
    return good, bad

  def save(self, key: tuple[int, int, int]) -> bool:
    vbuf = self.datamap[key]
    kbuf = struct.pack('>HBH', *key)
    dirpath = f'{self.path}/{kbuf[:3].hex()}'
    filepath = f'{dirpath}/{kbuf[3:].hex()}'
    if not mkdirp(self.vfs, dirpath):
      return False
    try:
      with self.vfs.open(filepath, 'wb') as fp:
        fp.write(binascii.b2a_base64(vbuf))
    except Exception as err:
      traceback.print_exception(err)
      return False
    return True

  def items(self):
    return self.datamap.items()

  def debugitems(self):
    for k, v in self.items():
      yield '/'.join(map(hex, k)), len(v)

def mkdirp(vfs: storage.VfsFat, path: str) -> bool:
  if not path:
    raise ValueError(path)
  if not path.startswith('/'):
    raise ValueError(f'path not absolute: {path}')
  try:
    stat = vfs.stat(path)
  except OSError as err:
    if err.errno != 2:
      traceback.print_exception(err)
      return False
  else:
    if (stat[0] & 0x4000) == 0x4000:
      return True
    err = Exception(f'{path} not a directory {stat=}')
    traceback.print_exception(err)
    return False
  nodes = path[1:].split('/')
  cur = ''
  for node in nodes:
    cur += f'/{node}'
    try:
      vfs.mkdir(cur)
    except OSError as err:
      if err.errno != 17:
        traceback.print_exception(err)
        return False
  return True
