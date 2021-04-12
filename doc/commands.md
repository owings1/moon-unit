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

- **05** - Get state of limit switches and stop pin

    ```
    :05 ;
    ```

    example response: `=00;TFFT|F`

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

- **12** - Get motor positions

    ```
    :12 <format>;
    ```

    example responses:

    ```
    =00;8500|1200
    =00;?|130.195
    =00:?|?
    ```
- **13** - No response (debug)

    ```
    :13 ;
    ```

    This is for testing the app command response timeout.

## Parameters

- `<motorId>`
    - `1` - scope
    - `2` - base
- `<direction>`
    - `1` - clockwise
    - `2` - anti-clockwise
- `<format>`
    - 1 - steps
    - 2 - degrees


## Response Codes

| Code | Meaning                    | Comment
|------|----------------------------|-----------
| `00` | OK                         |
| `01` | Device closed              | sent by app
| `02` | Command timeout            | sent by app
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