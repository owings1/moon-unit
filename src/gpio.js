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
var ic2
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

        //await gpio.setup(this.app.opts.pinEncoderClk, gpio.DIR_IN, gpio.EDGE_BOTH)
        //await gpio.setup(this.app.opts.pinEncoderDt, gpio.DIR_IN, gpio.EDGE_BOTH)
        //await gpio.setup(this.app.opts.pinEncoderDt, gpio.DIR_IN)
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

        /*
        // Encoder state
        this.counter = 0
        this.clkLastState = await gpio.read(this.app.opts.pinEncoderClk)
        */

        /*
        // https://www.best-microcontroller-projects.com/rotary-encoder.html
        this.counter1 = 0
        this.prevNextCode = 0
        this.store = 0
        this.rotTable = [0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0]
        */

        /*
        // best debouncing so far, but not as effective as its python impl,
        // probably performance issue
        // https://www.pinteric.com/rotary.html
        this.counter2 = 0
        this.lrmem = 3
        this.lrsum = 0
        this.trans = [0, -1, 1, 14, 1, 0, 14, -1, -1, 14, 0, 1, 14, 1, -1, 0]
        */

        this.isMenuActive = false

        

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
                            minValue      : 50
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
            value += spec.increment
            writeValue()
        }
        this.backwardHandler = async (howMuch) => {
            if (value <= spec.minValue) {
                return
            }
            value -= spec.increment
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

        var isFinished = false
        ;(async () => {
            const conn = await i2c.openPromisified(1)
            try {
                // TODO: handle EREMOTEIO error
                // clear change counter
                await this.readEncoder(conn)
                const startPos = 0
                this.log({startPos})
                var pos = startPos
                while (true) {
                    if (isFinished) {
                        break
                    }
                    var {change} = await this.readEncoder(conn)
                    if (change) {
                        if (Math.abs(change) > 12) {
                            this.log('dropped', {change})
                            continue
                        }
                        if (change > 0) {
                            if (this.forwardHandler) {
                                this.forwardHandler(change)
                            }
                        } else if (change < 0) {
                            if (this.backwardHandler) {
                                this.backwardHandler(-1 * change)
                            }
                        }
                        //var inc = getIncrement(values.change, 'square')
                        //pos += inc
                        this.log({change})
                    }
                    await new Promise(resolve => setTimeout(resolve, 50))
                }
            } finally {
                conn.close()
            }
        })()

        return new Promise((resolve, reject) => {
            this.buttonResolve = () => {
                this.buttonResolve = null
                this.buttonReject = null
                isFinished = true
                resolve()
            }
            this.buttonReject = err => {
                this.buttonResolve = null
                this.buttonReject = null
                isFinished = true
                reject(err)
            }
        })
    }

    // TODO: handle EREMOTEIO error
    async readEncoder(conn) {
        await conn.i2cWrite(this.app.opts.encoderAddress, EncWriteBuf.length, EncWriteBuf)
        const data = await conn.i2cRead(this.app.opts.encoderAddress, 1, Buffer.alloc(1))
        return this.parseEncoderResponse(data.buffer)
    }

    parseEncoderResponse(buf) {
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
            change: qty * sign
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
        /*if (pin == this.app.opts.pinEncoderClk) {
            if (!this.isDisplayActive) {
                this.registerDisplayAction()
                return
            }
            this.registerDisplayAction()
            //this.handleRotChange(value, await gpio.read(this.app.opts.pinEncoderDt))
            this.handleClkChange(value, await gpio.read(this.app.opts.pinEncoderDt))
        } else if (pin == this.app.opts.pinEncoderDt) {

            if (!this.isDisplayActive) {
                this.registerDisplayAction()
                return
            }
            this.registerDisplayAction()
            //this.handleRotChange(await gpio.read(this.app.opts.pinEncoderClk), value)
            this.handleClkChange(await gpio.read(this.app.opts.pinEncoderClk), value)
        } else */if (pin == this.app.opts.pinEncoderButton) {
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

    /*
    async handleClkChange(clkState, dtState) {
        if (clkState != this.clkLastState) {
            //const dtState = await gpio.read(this.app.opts.pinEncoderDt)
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

    handleRotChange(lft, rght) {
        const val = this.readRotary(lft, rght)
        if (val != 0) {
            this.counter2 += val
            // TODO: this has to slow things down when the handlers call printSync on lcd
            if (val > 0 && this.forwardHandler) {
                this.forwardHandler()
            } else if (val < 0 && this.backwardHandler) {
                this.backwardHandler()
            }
        }
        //this.log({counter2: this.counter2})
    }

    // https://www.pinteric.com/rotary.html
    readRotary(lft, rght) {
        this.lrmem = (this.lrmem % 4)*4 + 2*lft + rght
        this.lrsum = this.lrsum + this.trans[this.lrmem]
        if (this.lrsum % 4 != 0) {
            return 0
        }
        if (this.lrsum == 4) {
            this.lrsum = 0
            return 1
        }
        if (this.lrsum == -4) {
            this.lrsum = 0
            return -1
        }
        this.lrsum = 0
        return 0
    }
    */
    /*
    async handleRotChange(clkState) {
        const val = await this.readRotary(clkState)
        this.log({
            val,
            prevNextCode: this.prevNextCode,
            store: this.store
        })
        if (val) {
            this.counter1 += val
            //if (val == 1) {
            //    if (this.forwardHandler) {
            //        this.log('forwardHandler')
            //        this.forwardHandler()
            //    }
            //} else {
            //    if (this.backwardHandler) {
            //        this.log('backwardHandler')
            //        this.backwardHandler()
            //    }
            //}
        }
        this.log({counter1: this.counter1})
    }

    async readRotary(clkState) {
        this.prevNextCode <<= 2
        const dtState = await gpio.read(this.app.opts.pinEncoderDt)
        if (dtState) {
            this.prevNextCode |= 0x02
        }
        if (clkState) {
            this.prevNextCode |= 0x01
        }
        this.prevNextCode &= 0x0f
        if (this.rotTable[this.prevNextCode]) {
            this.store <<= 4
            this.store |= this.prevNextCode
            if ((this.store & 0xff) == 0x2b) {
                return -1
            }
            if ((this.store & 0xff) == 0x17) {
                return 1
            }
        }
        return 0
    }
    */

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