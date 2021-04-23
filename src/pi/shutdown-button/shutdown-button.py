# Adapted from: https://core-electronics.com.au/tutorials/how-to-make-a-safe-shutdown-button-for-raspberry-pi.html
from gpiozero import Button
import time
import os

stop_button_pin = 5
#reset_button_pin = 6
stop_button = Button(stop_button_pin)
#reset_button = Button(reset_button_pin)

def loop():
    if stop_button.is_pressed:
        time.sleep(1)
        if stop_button.is_pressed:
            print('Stop button pressed')
            os.system('shutdown -H now')
    #if reset_button.is_pressed:
    #    time.sleep(1)
    #    if reset_button.is_pressed:
    #        print('Reset button pressed')
    #        os.system('reboot')

def main():
    print(('Listening for stop button press on GPIO pin', stop_button_pin))
    #print(('Listening for reset button press on GPIO pin', reset_button_pin))
    while True:
        loop()
        time.sleep(1)

if __name__ == '__main__':
    main()
