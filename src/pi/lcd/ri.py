import I2C_LCD_driver
import time
from RPi import GPIO
I2C_LCD_driver.I2CBUS = 1

lcd = I2C_LCD_driver.lcd(0x3f)

lcd.lcd_display_string("Hello World!", 1)
lcd.lcd_display_string("Bonjour", 2)
lcd.lcd_display_string("Horses!", 3)
lcd.lcd_display_string("Bananas!", 4)

time.sleep(1)

lcd.backlight(0)

time.sleep(1)

lcd.backlight(1)



# rotary encoder
#  see https://github.com/modmypi/Rotary-Encoder/blob/master/rotary_encoder.py

# GPIO pins
#clk = 17
#dt = 18
clk = 18
#dt = 17
dt = 19

GPIO.setmode(GPIO.BCM)
GPIO.setup(clk, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dt, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

counter = 0
clk_last_state = GPIO.input(clk)

try:
  while True:
    clk_state = GPIO.input(clk)
    dt_state = GPIO.input(dt)
    if clk_state != clk_last_state:
      if dt_state != clk_state:
        counter += 1
      else:
        counter -= 1
      lcd.lcd_clear()
      lcd.lcd_display_string(str(counter), 1)
    clk_last_state = clk_state
    #time.sleep(0.01)
finally:
  GPIO.cleanup()
