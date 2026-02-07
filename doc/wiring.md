

## Command Module



I2C SDA
I2C SCL
GPS

### Back Panels


#### Left

> Note: I2C devices are the mag, and the 2 orientation sensors.

Top:

- I2C common
- I2C common
- GPS

Bottom:

- I2C common
- I2C common
- I2C common

#### Right

Top:

- Scope motor
- Scope limit CW/UP (motor side)
- Scope limit ACW/DOWN

Bottom:

- Base motor
- Base limit CW/RIGHT (motor side)
- Base limit ACW/LEFT


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
|  13 | stop signal                 | yellow | NC, signal to stop all motors when `HIGH`
|  A0 | ready signal                | green  | to gauger 5, ready when `HIGH`
|  A1 |                             |        | 
|  A2 |                             |        |
|  A3 |                             |        |
| SCL |                             |        |
| SDA |                             |        |

###  Gauger (Nano)

| Pin | Description                 | Color  | Notes
|-----|-----------------------------|--------|------------------
|   5 | Controller ready            | green  | uno A0
|   6 | RX from controller          | blue   | via rocker switch to uno 1
|   7 | TX to controller            | yellow | via rocker switch to uno 0
|   8 | RX from GPS                 | green  |
|   9 | TX to GPS (NC)              |        |
|  A4 | I2C SDA                     | white  |
|  A5 | I2C SCL                     | blue   |
| Rst | Reset pin                   | white  |

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

#### Other Links

A post about the BNO055 Euler bias:
- https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/BNO055-Operation-Mode-amp-Euler-Bias/td-p/7535

[schmitt]: https://www.ti.com/lit/ds/symlink/cd40106b.pdf?ts=1619275906436
