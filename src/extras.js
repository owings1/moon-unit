            //controllerPath     : env.CONTROLLER_PORT,
            //controllerBaudRate : +env.CONTROLLER_BAUD_RATE || 9600, //115200,


            //isControllerConnected     : this.isControllerConnected,
            //controllerConnectedStatus : this.isControllerConnected ? 'Connected' : 'Disconnected',
/*
if (!this.opts.controllerPath) {
    throw new ConfigError('path not set, you can use CONTROLLER_PORT')
}

this.controllerQueue        = []
this.controllerBusy         = false
this.controllerWorkerHandle = null
this.isControllerConnected  = false
*/


/*
this.openController().then(() =>
    this.openGauger()
).then(resolve).catch(reject)
*/

/*
async openController() {
    this.closeController()
    this.log('Opening controller', this.opts.controllerPath)
    this.controller = this.createDevice(this.opts.controllerPath, this.opts.controllerBaudRate)
    await new Promise((resolve, reject) => {
        this.controller.open(err => {
            if (err) {
                reject(err)
                return
            }
            this.isControllerConnected = true
            this.log('Controller opened, delaying', this.opts.openDelay, 'ms')
            this.controllerParser = this.controller.pipe(new Readline)
            setTimeout(() => {
                try {
                    this.initControllerWorker()
                    resolve()
                } catch (err) {
                    reject(err)
                }
            }, this.opts.openDelay)
        })
    })
}

closeController() {
    if (this.controller) {
        this.log('Closing controller')
        this.controller.close()
        this.controller = null
    }
    this.isControllerConnected = false
    this.clearStatus()
    this.drainControllerQueue()
    this.stopControllerWorker()
}
*/

/*
controllerCommand(body, params = {}) {
    return new Promise((resolve, reject) => {
        this.log('Enqueuing controller command', body.trim())
        this.controllerQueue.unshift({isSystem: false, ...params, body, handler: resolve})
    })
}

controllerLoop() {

    if (this.controllerBusy) {
        return
    }

    this.controllerBusy = true

    this.gpio.isControllerReady().then(isReady => {

        if (!isReady) {
            this.controllerBusy = false
            return
        }

        if (this.controllerQueue.length) {
            var {body, handler, isSystem} = this.controllerQueue.pop()
        } else {
            // TODO: various update tasks, e.g. motorSpeed
            var {body, handler, isSystem} = this.getPositionJob()
        }

        this.flushController().then(() => {

            var isComplete = false

            this.controllerParser.once('data', resText => {
                isComplete = true
                // handle device response
                if (!isSystem) {
                    this.log('Receieved response:', resText)
                }
                const status = parseInt(resText.substring(1, 3))
                handler({
                    status,
                    message : DeviceCodes[status],
                    body    : resText.substring(4),
                    raw     : resText
                })
                this.controllerBusy = false
            })

            if (!isSystem) {
                this.log('Sending command', body.trim())
            }

            this.controller.write(Buffer.from(this.opts.mock ? body : body.trim()))

            // TODO: rethink timeout, this is causing errors
            //setTimeout(() => {
            //    if (!isComplete) {
            //        this.error('Command timeout', body.trim())
            //        this.controllerParser.emit('data', '=02;')
            //    }
            //}, this.opts.commandTimeout)
        }).catch(err => {
            this.error('Flush failed', err)
            const status = 3
            handler({
                status,
                message: DeviceCodes[status],
                body   : '',
                raw    : '=03;',
                error  : err.message
            })
        })
    })
}
*/

/*
initControllerWorker() {
    this.log('Initializing controller worker to run every', this.opts.workerDelay, 'ms')
    this.stopControllerWorker()
    this.controllerWorkerHandle = setInterval(() => this.controllerLoop(), this.opts.workerDelay)
}

stopControllerWorker() {
    clearInterval(this.controllerWorkerHandle)
    this.controllerBusy = false
}

drainControllerQueue() {
    while (this.controllerQueue.length) {
        var {handler} = this.controllerQueue.pop()
        this.log('Sending error 1 response to handler')
        handler({status: 1, message: DeviceCodes[1]})
    }
}
*/

/*
async flushController() {
    // TODO: figure out why device.flush does not return a promise
    // commenting out for debug (getting errors)
    //return this.controller.flush()
}
*/

/*
app.post('/controller/disconnect', (req, res) => {
    this.closeController()
    this.status().then(status => {
        res.status(200).json({message: 'Device disconnected', status})
    })
})

app.post('/controller/connect', (req, res) => {
    if (this.isControllerConnected) {
        res.status(400).json({message: 'Device already connected'})
        return
    }
    this.openController().then(() => {
        this.status().then(status => {
            res.status(200).json({message: 'Device connected', status})
        })
    }).catch(error => {
        res.status(500).json({error})
    })
})
*/
/*
app.get('/controller/gpio/state', (req, res) => {
    if (!this.opts.gpioEnabled) {
        res.status(400).json({error: 'gpio not enabled'})
        return
    }
    this.gpio.getControllerState().then(state =>
        res.status(200).json({state})
    ).catch(error => {
        this.error(error)
        res.status(500).json({error})
    })
})
*/
/*

    getPositionJob() {
        return {
            isSystem : true,
            body     : ':15 ;\n',
            handler  : res => {
                if (res.status != 0) {
                    if (!this.opts.mock) {
                        this.error('Failed to get positions', res)
                    }
                    return
                }
               
                const arr = res.body.split('|')
                const floats = Util.floats(arr)
                this.position = [floats[0], floats[1]]
                this.limitsEnabled = [arr[2], arr[3]].map(it => it == 'T')
            }
        }
    }
*/

// lcd.js
/*
// LCD interface test

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

// NB: handlers should catch all errors
// what to do on a forward move
var forwardHandler
// what to do on a backward move
var backwardHandler
// what to do on the next button press
var buttonResolveOnce

function pause(ms) {
    return new Promise(resolve => setTimeout(resolve, ms))
}

async function init() {
    // always use sync methods, async methods are buggy
    lcd.beginSync()
    await gpio.setup(pinClk, gpio.DIR_IN, gpio.EDGE_BOTH)
    //await gpio.setup(pinClk, gpio.DIR_IN, gpio.EDGE_RISING)
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

function shutdown() {
    lcd.noDisplaySync()
    gpio.destroy()
}

class TimeoutError extends Error {}

async function waitForButtonPress(timeout) {

    var isPressed = false

    return new Promise((resolve, reject) => {
        if (timeout > 0) {
            setTimeout(() => {
                if (!isPressed) {
                    buttonResolveOnce = null
                    reject(new TimeoutError('timeout waiting for button press'))
                }
            }, timeout)
        }
        buttonResolveOnce = () => {
            isPressed = true
            resolve()
        }
    })
}

// this assumes choices has max length 4, or will throw an error
// declare as async so we can pause if needed
async function writeMenu(choices, selectedIndex) {
    if (choices.length > 4) {
        throw new Error('too many choices to display')
    }
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
// wait for button press with timeout, then return choice
async function getMenuChoice(choices, timeout) {
    var absoluteSelectedIndex = 0
    var currentSliceStart = 0
    var relativeSelectedIndex = 0
    var currentSlice = choices.slice(currentSliceStart, currentSliceStart + 4)
    await writeMenu(currentSlice, relativeSelectedIndex)
    // set forward/backward handlers
    forwardHandler = () => {
        try {
            if (absoluteSelectedIndex >= choices.length - 1) {
                // we can't go forward anymore
                return
            }
            absoluteSelectedIndex += 1
            if (relativeSelectedIndex < 3) {
                // we can keep the current slice, and just increment the relative index
                relativeSelectedIndex += 1
                updateSelectedIndex(relativeSelectedIndex)
            } else {
                // we must update the slice
                currentSliceStart += 1
                currentSlice = choices.slice(currentSliceStart, currentSliceStart + 4)
                // keep relative index the same since we will be at the end
                // redraw the whole menu
                
                writeMenu(currentSlice, relativeSelectedIndex)
            }
            console.log({currentSlice, relativeSelectedIndex, absoluteSelectedIndex})
        } catch (err) {
            // must handle error
            console.error(err)
        }
        
    }
    backwardHandler = () => {
        try {
            if (absoluteSelectedIndex < 1) {
                // we can't go backward anymore
                return
            }
            absoluteSelectedIndex -= 1
            if (relativeSelectedIndex > 0) {
                // we can keep the current slice, and just decrement the relative index
                relativeSelectedIndex -= 1
                updateSelectedIndex(relativeSelectedIndex)
            } else {
                // we must update the slice
                currentSliceStart -= 1
                currentSlice = choices.slice(currentSliceStart, currentSliceStart + 4)
                // keep relative index the same since we will be at the beginning
                // redraw the whole menu
                writeMenu(currentSlice, relativeSelectedIndex)
            }
            console.log({currentSlice, relativeSelectedIndex, absoluteSelectedIndex})
        } catch (err) {
            // must handle error
            console.error(err)
        }
    }
    try {
        // wait for button press
        await waitForButtonPress(timeout)
    } finally {
        // remove forward/backward handlers
        forwardHandler = null
        backwardHandler = null
    }

    return {
        index: absoluteSelectedIndex,
        value: choices[absoluteSelectedIndex]
    }
}

async function printSelection(value) {
    lcd.clearSync()
    lcd.setCursorSync(0, 0)
    lcd.printSync('enjoy your ' + value)
}

async function printTimeoutError(err) {
    lcd.clearSync()
    lcd.setCursorSync(0, 0)
    lcd.printSync('timeout: ' + err.message)
}

// get menu choice, then say "you selected x", then quit.
async function main() {

    await init()

    try {

        // 4 item menu
        var choice = await getMenuChoice(['apples', 'bananas', 'candy', 'drink'])
        await printSelection(choice.value)
        await pause(5000)

        // 5 item menu
        var choice = await getMenuChoice(['first', 'second', 'third', 'fourth', 'fifth'])
        await printSelection(choice.value)
        await pause(5000)

        // timeout
        try {
            var choice = await getMenuChoice(['there is a god', 'there is no god'], 5000)
            await printSelection(choice.value)
        } catch (err) {
            if (err instanceof TimeoutError) {
                await printTimeoutError(err)
            } else {
                throw err
            }
        }
        await pause(5000)

    } finally {
        shutdown()
    }
}

// run main
main()

*/