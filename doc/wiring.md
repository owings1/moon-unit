
motor controller
----------------

### Motors

`m1` - This is the motor that pitches the telescope up and down
`m2` - This is the motor that yaws (rotates) the base


### Arduino pins

| Pin | Description                 | Color  | Notes
|-----|-----------------------------|--------|------------------
|   2 | r. encoder button           | blue   | interrupt for push button
|   3 | m1 limit switch CW          | white  |
|   4 | m1 limit switch ACW         | white  |
|   5 | m1 direction                |        |
|   6 | m1 step                     |        |
|   7 | m1 enable                   | white  | to motor controller
|   8 | m2 direction                |        |
|   9 | m2 step                     |        |
|  10 | m2 enable                   | white  | to motor controller
|  11 | m2 limit switch CW          | white  |
|  12 | m2 limit switch ACW         | white  |
|  13 | stop pin                    | yellow | signal to stop all motors when `HIGH`
|  A0 | state pin 1 (SP1)           | green  | state bit for pi LSB
|  A1 | state pin 2 (SP2)           | white  | state bit for pi MSB
|  A2 | r. encoder pin 1            | yellow |
|  A3 | r. encoder pin 2            | white  |
| SCL | I2C SCL                     | blue   | I2C for orientation sensor*
| SDA | I2C SDA                     | green  | I2C for orientation sensor
| Rst | reset pin                   | blue   | for pi to reset when `LOW`

\* future to use 8x I2C multiplexer

### States

> SP1 is the LSB, SP2 is the MSB, so note the ordering in the table below.

| SP2  | SP1  | State      | Meaning
|------|------|------------|---------------------------------------------
| LOW  | LOW  | Ready      | No motors are moving, ready to read serial
| LOW  | HIGH | Busy       | Motors are moving, not ready to read serial
| HIGH | LOW  | Unassigned |
| HIGH | HIGH | Unassigned |


### Cables

See [raspberry pi GPIO pinout image][gpio]

- 5-pin special I/O from arduino to pi

| Color  | arduino | pi              | Function
|--------|---------|-----------------|--------------------------
| black  | GND     | GND             | 
| blue   | Rst     | GPIO26 - pin 37 | reset arduino when `LOW`
| yellow | 13      | GPIO19 - pin 35 | stop signal
| green  | A0      | GPIO20 - pin 38 | state pin 1 (SP1) LSB
| white  | A1      | GPIO16 - pin 36 | state pin 2 (SP2) MSB


[gpio]: https://elinux.org/images/5/5c/Pi-GPIO-header.png
