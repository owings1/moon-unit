
### Motor Controller (Uno)

| Pin | Description                 | Color  | Notes
|-----|-----------------------------|--------|------------------
|   0 | RX                          | yellow | via rocker switch to gauger 7
|   2 | TX                          | blue   | via rocker switch to gauger 6
|   3 | m1 limit switch CW          | yellow | inverter 2
|   4 | m1 limit switch ACW         | yellow | inverter 4
|   5 | m1 direction                | blue   |
|   6 | m1 step                     | yellow |
|   7 | m1 enable                   | white  | to motor driver
|   8 | m2 direction                | blue   |
|   9 | m2 step                     | yellow |
|  10 | m2 enable                   | white  | to motor driver
|  11 | m2 limit switch CW          | yellow | inverter 6
|  12 | m2 limit switch ACW         | yellow | inverter 8
|  13 | stop signal                 | yellow | to pi 18, signal to stop all motors when `HIGH`
|  A0 | ready signal                | green  | to gauger 5 and pi 38, ready when `HIGH`
|  A1 |                             |        | 
|  A2 |                             |        |
|  A3 |                             |        |
| SCL |                             |        |
| SDA |                             |        |
| Rst | reset pin                   | blue   | for pi to reset when `LOW`

###  Gauger (Nano)

| Pin | Description                 | Color  | Notes
|-----|-----------------------------|--------|------------------
| RX0 | RX from pi                  | yellow | via rocker switch to pi 8
| TX1 | TX to pi                    | blue   | via rocker switch to pi 10
|   5 | Controller ready            | green  | uno A0 and pi 16
|   6 | RX from controller          | blue   | via rocker switch to uno 1
|   7 | TX to controller            | yellow | via rocker switch to uno 0
|   8 | RX from gps                 | green  |
|   9 | TX to gps (NC)              |        |
|  A4 | I2C SDA                     | white  |
|  A5 | I2C SCL                     | blue   |
| Rst | Reset pin                   | white  | to pi 11

### Pi Zero

See [raspberry pi GPIO pinout image][gpio]. Pin 1 is on the SD card side.

| Pin  | Name    | Description         | Color  | Notes
|------|---------|---------------------|--------|--------
|   2  | 5V      | Power in            | red    | from voltage regulator pre-diode
|   3  | SDA     | I2C SDA             | white  | to LCD/encoder SDA
|   5  | SCL     | I2C SCL             | blue   | to LCD/encoder SCL
|   6  | GND     | Ground              | black  |
|   8  | GPIO 14 | TX to gauger        | yellow | via rocker switch to gauger pin RX0
|  10  | GPIO 15 | RX from gauger      | blue   | via rocker switch to gauger pin TX1
|  11  | GPIO 17 | Gauger reset        | white  | to gauger Rst
|  13  | GPIO 27 | Controller reset    | blue   | to uno Rst
|  15  | GPIO 22 | Encoder reset       | green  | to encoder module Rst
|  16  | GPIO 20 | Controller ready    | green  | to uno A0 and gauger 5
|  18  | GPIO 24 | Controller stop     | yellow | to uno 13
|  29  | GPIO  5 | Shutdown button     | white  | shutdown pi when `LOW` for 2s

### Schmitt Inverter

See [CD40106BE datasheet][schmitt]

| Pin | Name  | Description          | Color  | Notes
|-----|-------|----------------------|--------|--------------------
|   1 | A in  | m1 cw in             | white  |
|   2 | A out | m1 cw out            | yellow | controller pin 3
|   3 | B in  | m1 acw in            | white  |
|   4 | B out | m1 acw out           | yellow | controller pin 4
|   5 | C in  | m2 cw in             | white  |
|   6 | C out | m2 cw out            | yellow | controller pin 11
|   7 | VSS   | Ground               | black  |
|   8 | D out | m2 acw out           | yellow | controller pin 12
|   9 | D in  | m2 acw in            | white  |
|  14 | VDD   | +3.3v                | red    | gauger 3.3v


### Rotary module (Nano)

| Pin | Description                 | Color  | Notes
|-----|-----------------------------|--------|------------------
|   2 | encoder CLK                 | white  |
|   3 | encoder DT                  | yellow |
|   4 | encoder SW                  | blue   |
|  A4 | I2C SDA                     | white  | to pi 3
|  A5 | I2C SCL                     | blue   | to pi 5
| Rst | Reset pin                   | green  | to pi 31

#### Other Links

Software debouncer used in rotary module:
- https://www.pinteric.com/rotary.html

Hardware debouncer (not used, but scope shows it works well):
- https://hackaday.io/project/162207-hardware-debounced-rotary-encoder

Other software debouncer references (not used):
- https://www.best-microcontroller-projects.com/rotary-encoder.html
- https://hackaday.com/2015/12/09/embed-with-elliot-debounce-your-noisy-buttons-part-i/

A post about the BNO055 Euler bias:
- https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/BNO055-Operation-Mode-amp-Euler-Bias/td-p/7535

[gpio]: https://elinux.org/images/5/5c/Pi-GPIO-header.png
[schmitt]: https://www.ti.com/lit/ds/symlink/cd40106b.pdf?ts=1619275906436