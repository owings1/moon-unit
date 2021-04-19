
motor controller
----------------

### Motors

`m1` - This is the motor that pitches the telescope up and down
`m2` - This is the motor that yaws (rotates) the base


### Arduino pins

| Pin | Description                 | Color  | Notes
|-----|-----------------------------|--------|------------------
|   2 |                             |        | 
|   3 | m1 limit switch CW          | white  |
|   4 | m1 limit switch ACW         | white  |
|   5 | m1 direction                | blue   |
|   6 | m1 step                     | yellow |
|   7 | m1 enable                   | white  | to motor controller
|   8 | m2 direction                | blue   |
|   9 | m2 step                     | yellow |
|  10 | m2 enable                   | white  | to motor controller
|  11 | m2 limit switch CW          | white  |
|  12 | m2 limit switch ACW         | white  |
|  13 | stop signal                 | yellow | signal to stop all motors when `HIGH`
|  A0 | ready signal                | green  | ready when `HIGH`
|  A1 |                             |        | 
|  A2 |                             |        |
|  A3 |                             |        |
| SCL |                             |        |
| SDA |                             |        |
| Rst | reset pin                   | blue   | for pi to reset when `LOW`

Nano

| Pin | Description                 | Color  | Notes
|-----|-----------------------------|--------|------------------
|   5 | controller state            | green  | controller A0
|   6 | RX from controller          | blue   | controller 1
|   7 | TX to controller            | yellow | controller 0
|   8 | RX from gps                 | green  |
|   9 | TX to gps (NC)              |        |
|  A4 | I2C SDA                     | white  |
|  A5 | I2C SCL                     | blue   |
| Rst | reset pin                   | white  | connected to 5-pin special I/O to pi
### Cables

See [raspberry pi GPIO pinout image][gpio]

- 5-pin special I/O from arduino to pi

| Color  | arduino | pi              | Function
|--------|---------|-----------------|--------------------------
| black  | GND     | GND             | 
| blue   | Rst     | GPIO26 - pin 37 | reset controller when `LOW`
| yellow | 13      | GPIO19 - pin 35 | stop signal
| green  | A0      | GPIO20 - pin 38 | ready signal
| white  | Rst     | GPIO16 - pin 36 | reset gauger when `LOW`


[gpio]: https://elinux.org/images/5/5c/Pi-GPIO-header.png
