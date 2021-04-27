/*
refer to them by physical position, using the diagrams on this page. So holding
the Raspberry Pi such that the GPIO header runs down the upper-right side of
the board, if you wished to address GPIO4 (which is in column 1 and row 4), you
would setup pin 7. If you wish instead to refer to the pins by their GPIO names
(known as BCM naming), you can use the setMode command described in the API
documentation below.

https://elinux.org/images/5/5c/Pi-GPIO-header.png

*/

// TODO: debounce encoder clk
//          https://best-microcontroller-projects.com/rotary-encoder.html
// TODO: better button debounce

// dynamically load modules, else will fail on non-pi system
var gpio
var i2c
var LCD

const EncWriteBuf = Buffer.from([0x0])

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
            'pinGaugerReset'
        ]
        for (var k of requiredKeys) {
            if (!this.app.opts[k]) {
                throw new Error('Missing opt: ' + k)
            }
        }
        if (this.app.opts.lcdEnabled) {
            const lcdRequiredKeys = [
                'encoderAddress',
                'pinEncoderButton',
                'lcdAddress'
            ]
            for (var k of lcdRequiredKeys) {
                if (!this.app.opts[k]) {
                    throw new Error('Missing opt: ' + k)
                }
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

        await gpio.setup(this.app.opts.pinControllerReset, gpio.DIR_HIGH)
        await gpio.setup(this.app.opts.pinControllerStop, gpio.DIR_LOW)
        await gpio.setup(this.app.opts.pinControllerReady, gpio.DIR_IN)
        await gpio.setup(this.app.opts.pinGaugerReset, gpio.DIR_HIGH)

        gpio.on('change', (pin, value) => this.handlePinChange(pin, value))

        if (!this.app.opts.lcdEnabled) {
            return
        }

        if (!i2c) {
            i2c = require('i2c-bus')
        }
        if (!LCD) {
            LCD = require('raspberrypi-liquid-crystal')
        }

        await gpio.setup(this.app.opts.pinEncoderButton, gpio.DIR_IN, gpio.EDGE_RISING)

        await this.openLcd()
        await this.openEncoder()

        // NB: handlers should catch all errors
        // what to do on a forward move
        this.forwardHandler = null
        // what to do on a backward move
        this.backwardHandler = null
        // what to do on the next button press
        this.buttonResolve = null

        this.isMenuActive = false
        this.mainMenu()
    }

    async close() {
        if (!this.enabled) {
            return
        }
        // TODO: collect errors and throw, or ensure we never throw
        try {
            await this.closeLcd()
        } catch (err) {
            this.error('Error closing lcd', err)
        }
        try {
            await this.closeEncoder()
        } catch (err) {
            this.error('Error closing encoder', err)
        }
        gpio.destroy()
    }

    async openLcd() {
        try {
            await this.closeLcd()
        } catch (err) {
            this.error('Error closing lcd', err)
        }
        this.log('Opening LCD on address', '0x' + this.app.opts.lcdAddress.toString(16))
        this.lcd = new LCD(1, this.app.opts.lcdAddress, 20, 4)
        this.lcd.beginSync()
        this.checkSleepInterval = setInterval(() => this.checkSleep(), 1000)
    }

    async closeLcd() {
        clearInterval(this.checkSleepInterval)
        this.isDisplayActive = false
        this.lastDisplayActionMillis = null
        if (this.lcd) {
            this.lcd.noDisplaySync()
            this.lcd.closeSync()
        }
    }

    async openEncoder() {
        try {
            await this.closeEncoder()
        } catch (err) {
            this.error('Error closing encoder', err)
        }
        // NB: set the interval first so we can try to reconnect
        // TODO: setup reset pin to encoder module and send reset
        this.encoderInterval = setInterval(() => this.checkEncoder(), 50)
        this.log('Opening encoder on address', '0x' + this.app.opts.encoderAddress.toString(16))
        this.i2cConn = await i2c.openPromisified(1)
        // clear change counter
        await this._readEncoder()
    }

    async closeEncoder() {
        clearInterval(this.encoderInterval)
        this.encoderBusy = false
        if (this.i2cConn) {
            await this.i2cConn.close()
        }
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
                var choices = ['testNumber', 'testBoolean', 'testNumberDefault', 'fourth', 'fifth']
                var choice = await this.promptMenuChoice(choices)

                switch (choice.value) {
                    case 'testNumber':
                        var value = await this.promptNumber({
                            label         : 'Position',
                            initialValue  : 100.2,
                            decimalPlaces : 2,
                            increment     : 0.01,
                            maxValue      : 200,
                            minValue      : 50,
                            moveType      : 'square'
                        })
                        this.log({value})
                        break
                    case 'testBoolean':
                        var value = await this.promptBoolean()
                        this.log({value})
                        break
                    case 'testNumberDefault':
                        var value = await this.promptNumber()
                        this.log({value})
                        break
                    default:
                        this.lcd.clearSync()
                        this.lcd.setCursorSync(0, 0)
                        this.lcd.printSync('enjoy your ' + choice.value)
                        await new Promise(resolve => setTimeout(resolve, 5000))
                        break
                }
                
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

    // display four line menu with > prefix, move up/down with encoder,
    // wait for button press with timeout, then return choice
    // TODO: support default index - choose slice, relative index differently
    // TODO: support sticky top/bottom
    async promptMenuChoice(choices) {

        var currentSliceStart = 0
        var relativeSelectedIndex = 0

        var currentSlice = choices.slice(currentSliceStart, currentSliceStart + 4)
        var absoluteSelectedIndex = currentSliceStart + relativeSelectedIndex

        await this.writeMenu(currentSlice, relativeSelectedIndex)

        this.forwardHandler = howMuch => {
            try {
                if (absoluteSelectedIndex >= choices.length - 1) {
                    // we can't go forward anymore
                    return
                }
                absoluteSelectedIndex += 1
                if (relativeSelectedIndex < 3) {
                    // we can keep the current slice, and just increment the relative index
                    relativeSelectedIndex += 1
                    this.updateSelectedMenuIndex(currentSlice, relativeSelectedIndex)
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

        this.backwardHandler = howMuch => {
            try {
                if (absoluteSelectedIndex < 1) {
                    // we can't go backward anymore
                    return
                }
                absoluteSelectedIndex -= 1
                if (relativeSelectedIndex > 0) {
                    // we can keep the current slice, and just decrement the relative index
                    relativeSelectedIndex -= 1
                    this.updateSelectedMenuIndex(currentSlice, relativeSelectedIndex)
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
            // will throw TimeoutError
            await this.waitForInput()
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

    async promptBoolean(spec = {}) {
        spec = {
            label : 'Are you sure',
            ...spec
        }
        const promise = this.promptMenuChoice(['Yes', 'No'])
        // cheat by putting label at bottom until sticky top is supported
        this.lcd.setCursorSync(0, 2)
        this.lcd.printSync(spec.label + '?')
        const choice = await promise
        return choice.value == 'Yes'
    }

    async promptNumber(spec = {}) {
        spec = {
            label         : 'Number',
            initialValue  : 0,
            decimalPlaces : 0,
            increment     : 1,
            maxValue      : Infinity,
            minValue      : -Infinity,
            moveType      : 'linear',
            ...spec
        }

        var value = spec.initialValue

        this.lcd.clearSync()
        this.lcd.setCursorSync(0, 0)
        this.lcd.printSync(spec.label)
        
        const writeValue = () => {
            this.lcd.setCursorSync(0, 1)
            this.lcd.printSync(value.toFixed(spec.decimalPlaces).padStart(20))
        }

        this.forwardHandler = async (howMuch) => {
            if (value >= spec.maxValue) {
                return
            }
            value += spec.increment * this.getIncrement(howMuch, spec.moveType)
            writeValue()
        }
        this.backwardHandler = async (howMuch) => {
            if (value <= spec.minValue) {
                return
            }
            value -= spec.increment * this.getIncrement(howMuch, spec.moveType)
            writeValue()
        }

        writeValue()

        try {
            // will throw TimeoutError
            await this.waitForInput()
        } finally {
            this.forwardHandler = null
            this.backwardHandler = null
        }
        return value
    }

    // will throw TimeoutError
    async waitForInput() {

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

    async checkEncoder() {
        // TODO: handle EREMOTEIO error, log, reset, reconnect
        if (this.encoderBusy) {
            return
        }
        this.encoderBusy = true
        try {
            const {change} = await this._readEncoder()
            if (!change) {
                return
            }
            if (Math.abs(change) > 12) {
                this.log('dropped', {change})
                return
            }
            if (!this.isDisplayActive) {
                this.registerDisplayAction()
                return
            }
            this.registerDisplayAction()
            if (change > 0) {
                if (this.forwardHandler) {
                    this.forwardHandler(change)
                }
            } else if (change < 0) {
                if (this.backwardHandler) {
                    this.backwardHandler(-1 * change)
                }
            }
            //this.log({change})
        } catch (err) {
            if (err.code == 'EREMOTEIO') {
                this.error('Failed to read from encoder', err.message)
                // this will clear/reset the interval, so it should keep trying to reconnect
                //try {
                    await this.openEncoder()
                //} catch (err) {
                    //this.error('Failed to reopen encoder', err.message)
                //}
                
            } else {
                throw err
            }

        } finally {
            this.encoderBusy = false
        }
    }

    // TODO: handle EREMOTEIO error
    async _readEncoder() {
        await this.i2cConn.i2cWrite(this.app.opts.encoderAddress, EncWriteBuf.length, EncWriteBuf)
        const data = await this.i2cConn.i2cRead(this.app.opts.encoderAddress, 1, Buffer.alloc(1))
        return this._parseEncoderResponse(data.buffer)
    }

    _parseEncoderResponse(buf) {
        const byte = buf[0]
        // first (MSB) bit is push button
        //const isPressed = (byte & 128) == 128
        // second bit is positive=1 negative=0
        const sign = (byte & 64) == 64 ? 1 : -1
        // last six bits are the amount
        var qty = byte & ~192
        // ignore noise, TODO figure out why this is happening occasionally
        if (qty > 12) {
            this.log('dropped', {qty, byte})
            qty = 0
        }
        //console.log({byte, isPressed, sign, qty, buf: data.buffer.toJSON()})
        //console.log({byte})
        return {
            //isPressed,
            change: qty ? (qty * sign) : 0
        }
    }

    getIncrement(change, type) {
        switch (type) {
            case 'square':
                return change * Math.abs(change)
            case 'linear':
            default:
                return change
        }
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
    async updateSelectedMenuIndex(choices, selectedIndex) {
        if (choices.length > 4) {
            throw new Error('too many choices to display')
        }
        for (var i = 0; i < choices.length; i++) {
            this.lcd.setCursorSync(0, i)
            this.lcd.printSync((i == selectedIndex) ? '> ' : '  ')
        }
    }

    async handlePinChange(pin, value) {
        if (pin == this.app.opts.pinEncoderButton) {
            this.log('button')
            if (!this.isDisplayActive) {
                this.registerDisplayAction()
                return
            }
            if (this.isPressingButton) {
                return
            }
            this.isPressingButton = true
            // debounce
            setTimeout(() => this.isPressingButton = false, 200)
            if (value) {
                this.handleButtonResolve()
            }
        }
    }

    async handleButtonResolve() {
        if (this.buttonResolve) {
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