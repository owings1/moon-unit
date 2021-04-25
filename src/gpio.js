/*
refer to them by physical position, using the diagrams on this page. So holding
the Raspberry Pi such that the GPIO header runs down the upper-right side of
the board, if you wished to address GPIO4 (which is in column 1 and row 4), you
would setup pin 7. If you wish instead to refer to the pins by their GPIO names
(known as BCM naming), you can use the setMode command described in the API
documentation below.

https://elinux.org/images/5/5c/Pi-GPIO-header.png

*/
//async function main() {
//    await gpio.setup(38, gpio.DIR_IN)
//    setInterval(() => {
//        gpio.read(38).then(console.log)
//    }, 1000)
//}

//const Ri = require('./ri')

var gpio
var LCD

class Gpio {

    constructor(enabled, opts) {
        this.enabled = enabled
        if (!enabled) {
            return
        }
        if (!opts) {
            throw new Error('Invalid argument: opts')
        }
        for (var k of ['controllerReset', 'controllerStop', 'controllerReady', 'gaugerReset', 'encoderClk', 'encoderDt', 'encoderButton', 'lcdAddress']) {
            if (!opts[k]) {
                throw new Error('Missing opt: ' + k)
            }
        }
        this.opts = opts
        this.opts.menuTimeout = 20000
    }

    async open() {

        if (!this.enabled) {
            return
        }

        if (!gpio) {
            gpio = require('rpi-gpio').promise
        }
        if (!LCD) {
            LCD = require('raspberrypi-liquid-crystal')
        }

        await gpio.setup(this.opts.controllerReset, gpio.DIR_HIGH)
        await gpio.setup(this.opts.gaugerReset, gpio.DIR_HIGH)
        await gpio.setup(this.opts.controllerStop, gpio.DIR_LOW)
        await gpio.setup(this.opts.controllerReady, gpio.DIR_IN)

        //this.ri = new Ri(this.opts)
        //await this.ri.open()
        await gpio.setup(this.opts.encoderClk, gpio.DIR_IN, gpio.EDGE_BOTH)
        //await gpio.setup(pinClk, gpio.DIR_IN, gpio.EDGE_RISING)
        await gpio.setup(this.opts.encoderDt, gpio.DIR_IN)
        await gpio.setup(this.opts.encoderButton, gpio.DIR_IN, gpio.EDGE_RISING)
        this.lcd = new LCD(1, this.opts.lcdAddress, 20, 4)
        this.lcd.beginSync()

        // NB: handlers should catch all errors
        // what to do on a forward move
        this.forwardHandler = null
        // what to do on a backward move
        this.backwardHandler = null
        // what to do on the next button press
        this.buttonResolveOnce = null

        this.isMenuActive = false
        this.counter = 0
        this.clkLastState = await gpio.read(this.opts.encoderClk)

        gpio.on('change', async (pin, value) => {
            if (pin == this.opts.encoderClk) {
                if (!this.isMenuActive) {
                    // wake up from sleep
                    this.mainMenu()
                    return
                }
                try {
                    const clkState = value
                    if (clkState != this.clkLastState) {
                        const dtState = await gpio.read(this.opts.encoderDt)
                        if (dtState != clkState) {
                            this.counter += 1
                            if (this.forwardHandler) {
                                this.log('forwardHandler')
                                this.forwardHandler()
                            }
                        } else {
                            this.counter -= 1
                            if (this.backwardHandler) {
                                this.log('backwardHandler')
                                this.backwardHandler()
                            }
                        }
                    }
                    this.clkLastState = clkState
                    this.log({counter: this.counter})
                } finally {

                }
            } else if (pin == this.opts.encoderButton) {
                this.log('button', {value})
                if (!this.isMenuActive) {
                    // wake up from sleep
                    this.mainMenu()
                    return
                }
                if (value && this.buttonResolveOnce) {
                    this.log('buttonResolveOnce')
                    // TODO: debounce? not needed for simple once event
                    this.buttonResolveOnce()
                    this.buttonResolveOnce = null
                }
            }
            //console.log({pin, value})
        })

        this.shouldExit = false
        this.mainMenu()
    }

    async close() {
        if (!this.enabled) {
            return
        }
        this.lcd.noDisplaySync()
        gpio.destroy()
    }

    async isControllerReady() {
        if (!this.enabled) {
            return true
        }

        const value = await gpio.read(this.opts.controllerReady)
        return value
    }

    async getControllerState() {
        const isReady = await this.isControllerReady()
        return isReady ? 'ready' : 'busy'
    }

    async sendControllerStop() {
        if (!this.enabled) {
            return
        }

        await gpio.write(this.opts.controllerStop, true)
        // keep stop pin on for 1 second
        await new Promise((resolve, reject) =>
            setTimeout(() => {
                gpio.write(this.opts.controllerStop, false).then(resolve).catch(reject)
            }, 1000)
        )
    }

    async sendControllerReset() {
        if (!this.enabled) {
            return
        }
        await gpio.write(this.opts.controllerReset, false)
        await new Promise((resolve, reject) =>
            setTimeout(() => {
                gpio.write(this.opts.controllerReset, true).then(resolve).catch(reject)
            }, 100)
        )
    }

    async sendGaugerReset() {
        if (!this.enabled) {
            return
        }
        await gpio.write(this.opts.gaugerReset, false)
        await new Promise((resolve, reject) =>
            setTimeout(() => {
                gpio.write(this.opts.gaugerReset, true).then(resolve).catch(reject)
            }, 100)
        )
    }

    async mainMenu() {
        // dummy test to see how this behaves
        if (this.shouldExit) {
            return
        }
        this.isMenuActive = true
        try {
            var choices = ['first', 'second', 'third', 'fourth', 'fifth']
            var choice = await this.getMenuChoice(choice, this.opts.menuTimeout)

            // handle choice
            this.lcd.clearSync()
            this.lcd.setCursorSync(0, 0)
            this.lcd.printSync('enjoy your ' + value)
            await new Promise(resolve => setTimeout(resolve, 5000))

        } catch (err) {
            if (err instanceof TimeoutError) {
                // sleep
                this.lcd.noDisplaySync()
                this.isMenuActive = false
            } else {
                this.error(err)
            }
        }
    }

    async waitForButtonPress(timeout) {

        var isPressed = false

        return new Promise((resolve, reject) => {
            if (timeout > 0) {
                setTimeout(() => {
                    if (!isPressed) {
                        this.buttonResolveOnce = null
                        reject(new TimeoutError('timeout waiting for button press'))
                    }
                }, timeout)
            }
            this.buttonResolveOnce = () => {
                isPressed = true
                resolve()
            }
        })
    }

    // this assumes choices has max length 4, or will throw an error
    // declare as async so we can pause if needed
    async writeMenu(choices, selectedIndex) {
        if (choices.length > 4) {
            throw new Error('too many choices to display')
        }
        this.lcd.clearSync()
        for (var i = 0; i < choices.length; i++) {
            this.lcd.setCursorSync(0, i)
            this.log({i, selectedIndex})
            this.lcd.printSync(((i == selectedIndex) ? '> ' : '  ') + choices[i])
        }
    }

    // just rewrite the prefixes, 0 <= selectedIndex <= 3
    async updateSelectedMenuIndex(selectedIndex) {
        for (var i = 0; i < 4; i++) {
            this.lcd.setCursorSync(0, i)
            this.lcd.printSync((i == selectedIndex) ? '> ' : '  ')
        }
    }

    // display four line menu with > prefix, move up/down with encoder,
    // wait for button press with timeout, then return choice
    async getMenuChoice(choices, timeout) {
        var absoluteSelectedIndex = 0
        var currentSliceStart = 0
        var relativeSelectedIndex = 0
        var currentSlice = choices.slice(currentSliceStart, currentSliceStart + 4)
        await this.writeMenu(currentSlice, relativeSelectedIndex)
        // set forward/backward handlers
        this.forwardHandler = () => {
            try {
                if (absoluteSelectedIndex >= choices.length - 1) {
                    // we can't go forward anymore
                    return
                }
                absoluteSelectedIndex += 1
                if (relativeSelectedIndex < 3) {
                    // we can keep the current slice, and just increment the relative index
                    relativeSelectedIndex += 1
                    this.updateSelectedMenuIndex(relativeSelectedIndex)
                } else {
                    // we must update the slice
                    currentSliceStart += 1
                    currentSlice = choices.slice(currentSliceStart, currentSliceStart + 4)
                    // keep relative index the same since we will be at the end
                    // redraw the whole menu
                
                    this.writeMenu(currentSlice, relativeSelectedIndex)
                }
                this.log({currentSlice, relativeSelectedIndex, absoluteSelectedIndex})
            } catch (err) {
                // must handle error
                this.error(err)
            }
        
        }
        this.backwardHandler = () => {
            try {
                if (absoluteSelectedIndex < 1) {
                    // we can't go backward anymore
                    return
                }
                absoluteSelectedIndex -= 1
                if (relativeSelectedIndex > 0) {
                    // we can keep the current slice, and just decrement the relative index
                    relativeSelectedIndex -= 1
                    this.updateSelectedMenuIndex(relativeSelectedIndex)
                } else {
                    // we must update the slice
                    currentSliceStart -= 1
                    currentSlice = choices.slice(currentSliceStart, currentSliceStart + 4)
                    // keep relative index the same since we will be at the beginning
                    // redraw the whole menu
                    this.writeMenu(currentSlice, relativeSelectedIndex)
                }
                this.log({currentSlice, relativeSelectedIndex, absoluteSelectedIndex})
            } catch (err) {
                // must handle error
                this.error(err)
            }
        }
        try {
            // wait for button press
            await this.waitForButtonPress(timeout)
        } finally {
            // remove forward/backward handlers
            this.forwardHandler = null
            this.backwardHandler = null
        }

        return {
            index: absoluteSelectedIndex,
            value: choices[absoluteSelectedIndex]
        }
    }

    log(...args) {
        if (!this.opts.quiet) {
            console.log(new Date, ...args)
        }
    }

    error(...args) {
        console.error(new Date, ...args)
    }
}

class TimeoutError extends Error {}

module.exports = Gpio