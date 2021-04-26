#  Raspberry Pi Master for Arduino Slave
#  i2c_master_pi.py
#  Connects to Arduino via I2C

#  DroneBot Workshop 2019
#  https://dronebotworkshop.com

from smbus import SMBus
import time

addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/ic2-1

def main():
  while True:
    #bus.write_byte(addr, 0x0)
    this_byte = bus.read_byte(addr)
    if this_byte == 94: # '^'
      chars = []
      while True:
        this_byte = bus.read_byte(addr)
        if this_byte == 94:
          print('double 94')
          return
        if this_byte == 13: # '\n'
          break
        chars.append(this_byte)
      print(chars)
    #print(bus.read_byte(addr))

    time.sleep(1)

main()
