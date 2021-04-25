/*
refer to them by physical position, using the diagrams on this page. So holding
the Raspberry Pi such that the GPIO header runs down the upper-right side of
the board, if you wished to address GPIO4 (which is in column 1 and row 4), you
would setup pin 7. If you wish instead to refer to the pins by their GPIO names
(known as BCM naming), you can use the setMode command described in the API
documentation below.

https://elinux.org/images/5/5c/Pi-GPIO-header.png

*/

// TODO: for more precision, investigate debouncing, see
//          https://best-microcontroller-projects.com/rotary-encoder.html


// dynamically load modules, else will fail on non-pi system
var gpio
var LCD

class GpioHelper {

    constructor(app) {

        this.app = app // tightly coupled!

        this.enabled = this.app.opts.gpioEnabled

        if (!this.enabled) {
            return
        }

        const requiredKeys = [
            'pinControllerReset',
            'pinControllerStop',
            'pinControllerReady',
            'pinGaugerReset',
            'pinEncoderClk',
            'pinEncoderDt',
            'pinEncoderButton',
            'lcdAddress'
        ]
        for (var k of requiredKeys) {
            if (!this.app.opts[k]) {
                throw new Error('Missing opt: ' + k)
            }
        }
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

        await gpio.setup(this.app.opts.pinControllerReset, gpio.DIR_HIGH)
        await gpio.setup(this.app.opts.pinControllerStop, gpio.DIR_LOW)
        await gpio.setup(this.app.opts.pinControllerReady, gpio.DIR_IN)
        await gpio.setup(this.app.opts.pinGaugerReset, gpio.DIR_HIGH)

        await gpio.setup(this.app.opts.pinEncoderClk, gpio.DIR_IN, gpio.EDGE_BOTH)
        await gpio.setup(this.app.opts.pinEncoderDt, gpio.DIR_IN)
        await gpio.setup(this.app.opts.pinEncoderButton, gpio.DIR_IN, gpio.EDGE_RISING)

        this.lcd = new LCD(1, this.app.opts.lcdAddress, 20, 4)
        this.lcd.beginSync()

        // Display state
        this.isDisplayActive = false
        this.lastDisplayActionMillis = null
        clearInterval(this.checkSleepInterval)
        this.checkSleepInterval = setInterval(() => this.checkSleep(), 1000)

        // NB: handlers should catch all errors
        // what to do on a forward move
        this.forwardHandler = null
        // what to do on a backward move
        this.backwardHandler = null
        // what to do on the next button press
        this.buttonResolve = null

        // Encoder state
        this.counter = 0
        this.clkLastState = await gpio.read(this.app.opts.pinEncoderClk)

        this.isMenuActive = false

        gpio.on('change', (pin, value) => this.handlePinChange(pin, value))

        this.mainMenu()
    }

    async close() {
        if (!this.enabled) {
            return
        }
        clearInterval(this.checkSleepInterval)
        this.lcd.noDisplaySync()
        this.isDisplayActive = false
        gpio.destroy()
    }

    async isControllerReady() {
        if (!this.enabled) {
            return true
        }

        const value = await gpio.read(this.app.opts.pinControllerReady)
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

        await gpio.write(this.app.opts.pinControllerStop, true)
        // keep stop pin on for 1 second
        await new Promise((resolve, reject) =>
            setTimeout(() => {
                gpio.write(this.app.opts.pinControllerStop, false).then(resolve).catch(reject)
            }, 1000)
        )
    }

    async sendControllerReset() {
        if (!this.enabled) {
            return
        }
        await gpio.write(this.app.opts.pinControllerReset, false)
        await new Promise((resolve, reject) =>
            setTimeout(() => {
                gpio.write(this.app.opts.pinControllerReset, true).then(resolve).catch(reject)
            }, 100)
        )
    }

    async sendGaugerReset() {
        if (!this.enabled) {
            return
        }
        await gpio.write(this.app.opts.pinGaugerReset, false)
        await new Promise((resolve, reject) =>
            setTimeout(() => {
                gpio.write(this.app.opts.pinGaugerReset, true).then(resolve).catch(reject)
            }, 100)
        )
    }

    async mainMenu() {

        if (this.isMenuActive) {
            // prevent double-call
            return
        }

        this.isMenuActive = true

        try {
            while (true) {
                var choices = ['first', 'second', 'third', 'fourth', 'fifth']
                var choice = await this.getMenuChoice(choices)

                // handle choice
                this.lcd.clearSync()
                this.lcd.setCursorSync(0, 0)
                this.lcd.printSync('enjoy your ' + choice.value)
                await new Promise(resolve => setTimeout(resolve, 5000))
            }
        } catch (err) {
            if (err instanceof TimeoutError) {

            } else {
                this.error(err)
            }
        } finally {
            this.isMenuActive = false
        }
    }

    async waitForButtonPress() {

        return new Promise((resolve, reject) => {
            this.buttonResolve = () => {
                this.buttonResolve = null
                this.buttonReject = null
                resolve()
            }
            this.buttonReject = err => {
                this.buttonResolve = null
                this.buttonReject = null
                reject(err)
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
            this.lcd.printSync(((i == selectedIndex) ? '> ' : '  ') + choices[i])
        }
        this.registerDisplayAction()
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

        var currentSliceStart = 0
        var relativeSelectedIndex = 0

        var currentSlice = choices.slice(currentSliceStart, currentSliceStart + 4)
        var absoluteSelectedIndex = currentSliceStart + relativeSelectedIndex

        await this.writeMenu(currentSlice, relativeSelectedIndex)

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
                //this.log({currentSlice, relativeSelectedIndex, absoluteSelectedIndex})
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
                //this.log({currentSlice, relativeSelectedIndex, absoluteSelectedIndex})
            } catch (err) {
                // must handle error
                this.error(err)
            }
        }

        try {
            // wait for button press, will throw TimeoutError
            await this.waitForButtonPress()
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

    async handlePinChange(pin, value) {
        if (pin == this.app.opts.pinEncoderClk) {
            if (!this.isDisplayActive) {
                this.registerDisplayAction()
                return
            }
            this.registerDisplayAction()
            this.handleClkChange(value)
        } else if (pin == this.app.opts.pinEncoderButton) {
            if (!this.isDisplayActive) {
                this.registerDisplayAction()
                return
            }
            if (value) {
                this.handleButtonResolve()
            }
        }
    }

    async handleClkChange(clkState) {
        if (clkState != this.clkLastState) {
            const dtState = await gpio.read(this.app.opts.pinEncoderDt)
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
    }

    async handleButtonResolve() {
        if (this.buttonResolve) {
            //this.log('buttonResolve')
            // TODO: debounce? not needed for simple once event
            this.buttonResolve()
            this.buttonResolve = null
        }
    }

    checkSleep() {
        const {displayTimeout} = this.app.opts
        if (displayTimeout > 0 && this.lastDisplayActionMillis + displayTimeout < +new Date) {
            // clear handlers
            this.buttonResolve = null
            if (this.buttonReject) {
                this.log('display sleep timeout')
                this.buttonReject(new TimeoutError)
            }
            this.buttonReject = null
            // sleep display
            this.lcd.noDisplaySync()
            this.isDisplayActive = false
        }
    }

    registerDisplayAction() {
        if (!this.isDisplayActive) {
            this.lcd.displaySync()
            this.isDisplayActive = true
        }
        this.lastDisplayActionMillis = +new Date
        // we can add other logic, e.g. to go to last active screen
        if (!this.isMenuActive) {
            this.mainMenu()
        }
    }

    log(...args) {
        if (!this.app.opts.quiet) {
            console.log(new Date, ...args)
        }
    }

    error(...args) {
        console.error(new Date, ...args)
    }
}

class TimeoutError extends Error {}

module.exports = GpioHelper