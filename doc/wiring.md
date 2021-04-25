
### Motor Controller (Uno)

| Pin | Description                 | Color  | Notes
|-----|-----------------------------|--------|------------------
|   0 | RX                          | yellow | via rocker switch to nano 7
|   2 | TX                          | blue   | via rocker switch to nano 6
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
|  A0 | ready signal                | green  | to nano 5 and pi 38, ready when `HIGH`
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
|   3  | SDA     | I2C SDA             | white  | to LCD SDA                                (wip)
|   5  | SCL     | I2C SCL             | blue   | to LCD SCL                                (wip)
|   6  | GND     | Ground              | black  |
|   8  | GPIO 14 | TX to gauger        | yellow | via rocker switch to nano pin RX0
|  10  | GPIO 15 | RX from gauger      | blue   | via rocker switch to nano pin TX1
|  12  | GPIO 18 | Encoder CLK         | white  | to encoder CLK via schmitt trigger       (wip)
|  11  | GPIO 17 | Gauger reset        | white  | to nano Rst
|  13  | GPIO 27 | Controller reset    | blue   | to uno Rst
|  16  | GPIO 20 | Controller ready    | green  | to uno A0 and nano 5
|  18  | GPIO 24 | Controller stop     | yellow | to uno 13
|  29  | GPIO 5  | Shutdown button     | white  | shutdown pi when `LOW` for 2s
|  33  | GPIO 13 | Encoder button      | blue   | to encoder SW via pull-up resistor        (wip)
|  35  | GPIO 19 | Encoder DT          | yellow | to encoder DT via schmitt trigger        (wip)

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
|  10 | E out | enc dt out           | yellow | to pi 35
|  11 | E in  | enc dt in            | yellow | to encoder DT
|  12 | F out | enc clk out          | white  | to pi 12
|  13 | F in  | enc clk in           | white  | to encoder CLK
|  14 | VDD   | +3.3v                | red    | nano 3.3v


[gpio]: https://elinux.org/images/5/5c/Pi-GPIO-header.png
[schmitt]: https://www.ti.com/lit/ds/symlink/cd40106b.pdf?ts=1619275906436