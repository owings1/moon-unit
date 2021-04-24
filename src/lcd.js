const LCD = require('raspberrypi-liquid-crystal')
const gpio = require('rpi-gpio').promise

const lcd = new LCD(1, 0x3f, 20, 4)

// add pull-up resistors to each of these pins
// TODO: for more precision, investigate debouncing, see
//          https://best-microcontroller-projects.com/rotary-encoder.html
const pinClk = 12 // GPIO 18
const pinDt = 35  // GPIO 19
const pinButton = 33 // GPIO 13

var counter = 0
var clkLastState

// what to do on a forward move
var forwardHandler
// what to do on a backward move
var backwardHandler
// what to do on the next button press
var buttonResolveOnce

async function init() {
    // always use sync methods, async methods are buggy
    lcd.beginSync()
    //await gpio.setup(pinClk, gpio.DIR_IN, gpio.EDGE_BOTH)
    await gpio.setup(pinClk, gpio.DIR_IN, gpio.EDGE_RISING)
    await gpio.setup(pinDt, gpio.DIR_IN)
    await gpio.setup(pinButton, gpio.DIR_IN, gpio.EDGE_RISING)

    clkLastState = await gpio.read(pinClk)

    gpio.on('change', async (pin, value) => {
        if (pin == pinClk) {
            try {
                const clkState = value
                if (clkState != clkLastState) {
                    const dtState = await gpio.read(pinDt)
                    if (dtState != clkState) {
                        counter += 1
                        if (forwardHandler) {
                            console.log('forwardHandler')
                            forwardHandler()
                        }
                    } else {
                        counter -= 1
                        if (backwardHandler) {
                            console.log('backwardHandler')
                            backwardHandler()
                        }
                    }
                }
                clkLastState = clkState
                console.log({counter})
            } finally {

            }
        } else if (pin == pinButton) {
            console.log('button', {value})
            if (value && buttonResolveOnce) {
                console.log('buttonResolveOnce')
                // TODO: debounce? not needed for simple once event
                buttonResolveOnce()
                buttonResolveOnce = null
            }
        }
        //console.log({pin, value})
    })
}

// this assumes choices has max length 4, or will truncate
// declare as async so we can pause if needed
async function writeMenu(choices, selectedIndex) {
    choices = choices.slice(0, 4)
    lcd.clearSync()
    for (var i = 0; i < choices.length; i++) {
        lcd.setCursorSync(0, i)
        console.log({i, selectedIndex})
        lcd.printSync(((i == selectedIndex) ? '> ' : '  ') + choices[i])
    }
}

// just rewrite the prefixes, 0 <= selectedIndex <= 3
async function updateSelectedIndex(selectedIndex) {
    for (var i = 0; i < 4; i++) {
        lcd.setCursorSync(0, i)
        lcd.printSync((i == selectedIndex) ? '> ' : '  ')
    }
}

// display four line menu with > prefix, move up/down with encoder,
// wait for button press, then return choice
async function getMenuChoice() {
    const choices = ['apples', 'bananas', 'candy', 'drink']
    var selectedIndex = 0
    await writeMenu(choices, selectedIndex)
    // set forward/backward handlers
    forwardHandler = () => {
        if (selectedIndex < choices.length - 1) {
            selectedIndex += 1
            updateSelectedIndex(selectedIndex)
        }
    }
    backwardHandler = () => {
        if (selectedIndex > 0) {
            selectedIndex -= 1
            updateSelectedIndex(selectedIndex)
        }
    }
    try {
        // wait for button press
        await new Promise(resolve => buttonResolveOnce = resolve)
    } finally {
        // remove forward/backward handlers
        forwardHandler = null
        backwardHandler = null
    }

    return choices[selectedIndex]
}

function shutdown() {
    lcd.noDisplaySync()
    gpio.destroy()
}
// get menu choice, then say "you selected x", then quit.
async function main() {
    await init()
    const choice = await getMenuChoice()
    await lcd.clear()
    await lcd.setCursor(0, 0)
    await lcd.print('you selected ' + choice)
    await new Promise(resolve => setTimeout(resolve, 5000))
    shutdown()
}

main()

//lcd.noDisplaySync()


async function example1() {
    await init()
    await lcd.clear()
    await new Promise(resolve => setTimeout(resolve, 5000))
    await lcd.print('Hello')
    await new Promise(resolve => setTimeout(resolve, 5000))
    await lcd.setCursor(0, 1)
    await new Promise(resolve => setTimeout(resolve, 5000))
    await lcd.print('World')
    await new Promise(resolve => setTimeout(resolve, 5000))
    await lcd.noDisplay()
    shutdown()
}
