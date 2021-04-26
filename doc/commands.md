# Command Reference

Commands
---------

- **01** - Move single motor n steps in a given direction

    ```
    :01 <motorId> <direction> <steps>;
    ```

- **02** - Set max speed for a motor
   
    ```
    :02 <motorId> <speed>;
    ```

- **03** - Set acceleration for a motor

    ```
    :03 <motorId> <acceleration>;
    ```

- **04** - Move single motor n degrees in a given direction

    ```
    :04 <motorId> <direction> <degrees>;
    ```

- **06** - Home a single motor

    ```
    :06 <motorId>;
    ```

- **07** - Home both motors

    ```
    :07 ;
    ```

- **08** - End a single motor

    ```
    :08 <motorId>;
    ```

- **09** - End both motors

    ```
    :09 ;
    ```

- **10** - Move both motors by steps

    ```
    :10 <direction_1> <steps_1> <direction_2> <steps_2>;
    ```

- **11** - Move both motors by degrees

    ```
    :11 <direction_1> <degrees_1> <direction_2> <degrees_2>;
    ```

- **13** - No response (debug)

    ```
    :13 ;
    ```

    This is for testing the app command response timeout.

- **17** - Set limit switch enablement for a motor

    ```
    :17 <motorId> <T|F>;
    ```

- **18** - Get full controller status

    ```
    :18 ;
    ```

    Indexes
    
    * `0`  : `<position_m1_degrees>`
    * `1`  : `<position_m2_degrees>`
    * `2`  : `<limitsEnabled_m1>`
    * `3`  : `<limitsEnabled_m2>`
    * `4`  : `<degreesPerStep_m1>`
    * `5`  : `<degreesPerStep_m2>`
    * `6`  : `<maxSpeed_m1>`
    * `7`  : `<maxSpeed_m2>`
    * `8`  : `<acceleration_m1>`
    * `9`  : `<acceleration_m2>`
    * `10` : limit states: `<m1_cw><m1_acw><m2_cw><m2_acw>`
    * `11` : `<shouldStop>`

- **71** - Set gauger mode
 
    ```
    :71 <mode>;
    ```

- **72** - Set declination angle

    ```
    :72 <radians>;
    ```

- **73** - Set gauger loop delay

    ```
    :73 <milliseconds>
    ```


## Parameters

- `<motorId>`
    - `1` - scope
    - `2` - base
- `<direction>`
    - `1` - clockwise
    - `2` - anti-clockwise
- `<format>`
    - `1` - steps
    - `2` - degrees


## Response Codes

| Code | Meaning                    | Comment
|------|----------------------------|-----------
| `00` | OK                         |
| `01` | Device closed              | sent by app
| `02` | Command timeout            | sent by app
| `03` | Flush error                | sent by app
| `04` | Device busy                |
| `40` | Missing : before command   |
| `44` | Invalid command            |
| `45` | Invalid motorId            |
| `46` | Invalid direction          |
| `47` | Invalid steps/degrees      |
| `48` | Invalid speed/acceleration |
| `49` | Invalid other parameter    |

## States

- `0` - ready for command
- `1` - running command