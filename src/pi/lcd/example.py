import I2C_LCD_driver
from time import *

mylcd = I2C_LCD_driver.lcd()

mylcd.lcd_display_string("Hello World!", 1)
mylcd.lcd_display_string("Bonjour", 2)
mylcd.lcd_display_string("Horses!", 3)
mylcd.lcd_display_string("Bananas!", 4)

sleep(1)

mylcd.backlight(0)

sleep(1)

mylcd.backlight(1)
