const fetch = require('node-fetch')
// dynamically load modules, else will fail on non-i2c system
var i2c
var LCD

const EncWriteBuf = Buffer.from([0x0])

class I2ciHelper {

    constructor(app) {

        this.app = app // tightly coupled!

        this.enabled = this.app.opts.i2ciEnabled

        if (!this.enabled) {
            return
        }

        const requiredKeys = [
            'encoderAddress',
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
        this.log('Opening I2C interface')
        if (!i2c) {
            i2c = require('i2c-bus')
        }
        if (!LCD) {
            LCD = require('raspberrypi-liquid-crystal')
        }

        await this.openLcd()
        await this.openEncoder()

        // NB: handlers should catch all errors
        this.onEncoderChange = null
        // what to do on the next button press
        this.buttonResolve = null

        this.isMenuActive = false
        this.mainMenu()
    }

    async close() {
        if (!this.enabled) {
            return
        }
        try {
            await this.closeLcd()
        } catch (err) {
            this.error('Error closing LCD', err)
        }
        try {
            await this.closeEncoder()
        } catch (err) {
            this.error('Error closing Encoder', err)
        }
    }

    async openLcd() {
        try {
            await this.closeLcd()
        } catch (err) {
            this.error('Error closing LCD', err)
        }
        this.log('Opening LCD on address', '0x' + this.app.opts.lcdAddress.toString(16))
        // NB: set the interval first so we can try to reconnect
        setTimeout(() => this.checkLcdInterval = setInterval(() => this.checkLcd(), 1000))
        this.lcd = new LCD(1, this.app.opts.lcdAddress, 20, 4)
        this.lcd.beginSync()
        this.isLcdConnected = true
        this.log('LCD opened')
    }

    async closeLcd() {
        clearInterval(this.checkLcdInterval)
        this.isLcdConnected = false
        this.isDisplayActive = false
        this.lastDisplayActionMillis = null
        if (this.lcd && this.isLcdConnected) {
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
        await this.app.gpio.resetEncoder()
        this.log('Opening encoder on address', '0x' + this.app.opts.encoderAddress.toString(16))
        // NB: set the interval first so we can try to reconnect
        setTimeout(() => this.encoderInterval = setInterval(() => this.checkEncoder(), 80))
        this.i2cConn = await i2c.openPromisified(1)
        // clear change counter
        await this._readEncoder()
        this.isPressingButton = false
        this.log('Encoder opened')
    }

    async closeEncoder() {
        clearInterval(this.encoderInterval)
        this.encoderBusy = false
        if (this.i2cConn) {
            await this.i2cConn.close()
        }
    }

    async mainMenu() {

        if (this.isMenuActive) {
            // prevent double-call
            return
        }

        this.isMenuActive = true

        try {
            while (true) {
                var choices = [
                    'Home Motors',
                    'testNumber', 'testBoolean', 'testBoolean2', 'testNumberDefault', 'fourth', 'fifth'
                ]
                var choice = await this.promptMenuChoice(choices)

                switch (choice.value) {
                    case 'Home Motors':
                        await this.homingMenu()
                        break
                    case 'testNumber':
                        var value = await this.promptNumber({
                            label         : 'Position',
                            initialValue  : 100.2,
                            decimalPlaces : 2,
                            increment     : 0.01,
                            maxValue      : 200,
                            minValue      : 50,
                            moveType      : 'natural'
                        })
                        this.log({value})
                        break
                    case 'testBoolean':
                        var value = await this.promptBoolean()
                        this.log({value})
                        break
                    case 'testBoolean2':
                        var value = await this.promptBoolean({label: 'For realz', default: false})
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

            } else if (isLcdWriteError(err)) {
                this.error('LCD write error', err.message)
                this.isLcdConnected = false
                this.handleButtonReject(err)
            } else {
                this.error(err)
            }
        } finally {
            this.isMenuActive = false
        }
    }

    async homingMenu() {
        const choices = [
            '<back>',
            'Home All',
            'Home Scope',
            'Home Base'
        ]
        while (true) {
            var choice = await this.promptMenuChoice(choices)
            var isQuit = false
            var cmd = null
            switch (choice.value) {
                case 'Home All':
                    cmd = ':07 ;\n'
                    break
                case 'Home Scope':
                    cmd = ':06 1;\n'
                    break
                case 'Home Base':
                    cmd = ':06 2;\n'
                    break
                case '<back>':
                default:
                    isQuit = true
                    break
            }
            if (isQuit) {
                break
            }
            await this.doMoveRequest(cmd, choice.value)
            await new Promise(resolve => setTimeout(resolve, 3000))
        }
    }

    async doMoveRequest(command, title) {
        title = title || 'Move'
        this.lcd.clearSync()
        this.lcd.setCursor(0, 0)
        this.lcd.printSync(title + ' ...')
        const res = await this.sendRequest('controller/command/sync', 'POST', {command})
        const body = await res.json()
        this.lcd.clearSync()
        this.lcd.setCursorSync(0, 0)
        this.lcd.printSync(['HTTP', res.status].join(' '))
        this.lcd.setCursorSync(0, 1)
        this.lcd.printSync(['Code', body.response.status].join(' '))
        this.lcd.setCursorSync(0, 2)
        this.lcd.printSync(body.response.message.substring(0, 19))
        return {res, body}
    }

    async sendRequest(uri, method, body) {
        const url = [this.app.localUrl, uri].join('/')
        const opts = {method}
        if (body) {
            opts.headers = {
                'Content-Type' : 'application/json'
            }
            opts.body = JSON.stringify(body)
        }
        return fetch(url, opts)
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

        this.onEncoderChange = async (change) => {
            try {
                if (change > 0) {
                    if (absoluteSelectedIndex >= choices.length - 1) {
                        // we can't go forward anymore
                        return
                    }
                    absoluteSelectedIndex += 1
                    if (relativeSelectedIndex < 3) {
                        // we can keep the current slice, and just increment the relative index
                        relativeSelectedIndex += 1
                        await this.updateSelectedMenuIndex(currentSlice, relativeSelectedIndex)
                        return
                    } else {
                        // we must update the slice
                        currentSliceStart += 1
                        currentSlice = choices.slice(currentSliceStart, currentSliceStart + 4)
                        // keep relative index the same since we will be at the end
                        // redraw the whole menu
                    }
                } else {
                    if (absoluteSelectedIndex < 1) {
                        // we can't go backward anymore
                        return
                    }
                    absoluteSelectedIndex -= 1
                    if (relativeSelectedIndex > 0) {
                        // we can keep the current slice, and just decrement the relative index
                        relativeSelectedIndex -= 1
                        await this.updateSelectedMenuIndex(currentSlice, relativeSelectedIndex)
                        return
                    } else {
                        // we must update the slice
                        currentSliceStart -= 1
                        currentSlice = choices.slice(currentSliceStart, currentSliceStart + 4)
                        // keep relative index the same since we will be at the beginning
                        // redraw the whole menu
                    }
                }
                await this.writeMenu(currentSlice, relativeSelectedIndex)
                //this.log({currentSlice, relativeSelectedIndex, absoluteSelectedIndex})
            } catch (err) {
                if (isLcdWriteError(err)) {
                    // handled in checkEncoder()
                    throw err
                }
                // must handle other errors
                this.error(err)
            }
        }

        try {
            // will throw TimeoutError
            await this.waitForInput()
        } finally {
            // remove change handler
            this.onEncoderChange = null
        }

        return {
            index: absoluteSelectedIndex,
            value: choices[absoluteSelectedIndex]
        }
    }

    async promptBoolean(spec = {}) {
        spec = {
            label : 'Are you sure',
            default : true,
            ...spec
        }
        this.lcd.clearSync()
        this.lcd.setCursorSync(0, 0)
        this.lcd.printSync(spec.label + '?')
        this.lcd.setCursorSync(2, 1)
        this.lcd.printSync('Yes')
        this.lcd.setCursorSync(2, 2)
        this.lcd.printSync('No')
        
        var value = null
        const select = newValue => {
            if (value === newValue) {
                return
            }
            value = newValue
            this.lcd.setCursorSync(0, 1)
            this.lcd.printSync(value ? '>' : ' ')
            this.lcd.setCursorSync(0, 2)
            this.lcd.printSync(value ? ' ' : '>')
        }

        this.onEncoderChange = change => select(change < 0)
        try {
            select(!!spec.default)
            await this.waitForInput()
        } finally {
            this.onEncoderChange = null
        }
        return value
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
        
        const writeValue = async () => {
            this.lcd.setCursorSync(0, 1)
            this.lcd.printSync(value.toFixed(spec.decimalPlaces).padStart(20))
        }

        this.onEncoderChange = change => {
            if (change > 0) {
                if (value >= spec.maxValue) {
                    return
                }
                value += spec.increment * this.getIncrement(change, spec.moveType)
                value = +Math.min(value, spec.maxValue).toFixed(spec.decimalPlaces)
            } else {
                if (value <= spec.minValue) {
                    return
                }
                value -= spec.increment * this.getIncrement(Math.abs(change), spec.moveType)
                value = +Math.max(value, spec.minValue).toFixed(spec.decimalPlaces)
            }
            return writeValue()
        }

        try {
            writeValue()
            // will throw TimeoutError
            await this.waitForInput()
        } finally {
            this.onEncoderChange = null
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
        if (this.encoderBusy) {
            return
        }
        this.encoderBusy = true
        try {
            const {change, isPressed} = await this._readEncoder()
            if (!change && !isPressed) {
                this.isPressingButton = false
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
            
            if (isPressed) {
                // we shouldn't need to debounce since this runs at an interval of > 50ms
                // but we do want to wait for the button to be released before we consider
                // is pressed again.
                if (!this.isPressingButton) {
                    this.isPressingButton = true
                    this.handleButtonResolve()
                    return
                }
            } else {
                this.isPressingButton = false
            }
            // don't call encoderChange if no menu is active
            if (this.onEncoderChange && this.isMenuActive) {
                await this.onEncoderChange(change)
            }
            //this.log({change})
        } catch (err) {
            // Here we have to handle both encoder and LCD I/O errors, since
            // encoder callbacks generally have LCD write methods.
            if (err.code == 'EREMOTEIO') {
                this.error('Failed to read from encoder', err.message)
                // this will clear/reset the interval, so it should keep trying to reconnect
                try {
                    await this.openEncoder()
                    this.log('Encoder reopened')
                } catch (err) {
                    if (err.code != 'EREMOTEIO') {
                        throw err
                    }
                }
            } else if (isLcdWriteError(err)) {
                // LCD write error
                this.error('Failed to write to LCD', err.message)
                // trigger reconnect on next interval
                this.isLcdConnected = false
                this.handleButtonReject(err)
            } else {
                throw err
            }
        } finally {
            this.encoderBusy = false
        }
    }

    async checkLcd() {
        //console.log('checkLcd')
        if (!this.isLcdConnected) {
            try {
                await this.openLcd()
                this.log('Reopened LCD')
                // so the display timeout will trigger
                this.registerDisplayAction()
            } catch (err) {
                this.error('Failed to reopen LCD', err.message)
                this.isMenuActive = false
                this.handleButtonReject(err)
            }
            
        }
        const {displayTimeout} = this.app.opts
        if (displayTimeout > 0 && this.lastDisplayActionMillis + displayTimeout < +new Date) {
            // clear handlers
            this.handleButtonReject(new TimeoutError)
            // sleep display
            if (this.isDisplayActive) {
                this.log('Input timeout')
                if (this.isLcdConnected) {
                    try {
                        this.lcd.noDisplaySync()
                    } catch (err) {
                        if (isLcdWriteError(err)) {
                            this.isLcdConnected = false
                            this.error('LCD Write error', err.message)
                        } else {
                            throw err
                        }
                    }
                }
                this.isDisplayActive = false
            }
        }
    }

    async _readEncoder() {
        await this.i2cConn.i2cWrite(this.app.opts.encoderAddress, EncWriteBuf.length, EncWriteBuf)
        const data = await this.i2cConn.i2cRead(this.app.opts.encoderAddress, 1, Buffer.alloc(1))
        return this._parseEncoderResponse(data.buffer)
    }

    _parseEncoderResponse(buf) {
        const byte = buf[0]
        // first (MSB) bit is push button
        const isPressed = (byte & 128) == 128
        // second bit is positive=1 negative=0
        const sign = (byte & 64) == 64 ? 1 : -1
        // last six bits are the amount
        var qty = byte & ~192
        // ignore noise, TODO figure out why this is happening occasionally
        // update: haven't seen it in a while....
        if (qty > 12) {
            this.log('WARN', 'dropped', {qty, byte})
            qty = 0
        }
        //console.log({byte, isPressed, sign, qty, buf: data.buffer.toJSON()})
        //console.log({byte})
        return {
            isPressed,
            change: qty ? (qty * sign) : 0
        }
    }

    getIncrement(change, type) {
        const abs = Math.abs(change)
        const mult = change < 0 ? -1 : 1
        switch (type) {
            case 'square':
                return change * abs
            case 'cube':
                return Math.pow(change, 3)
            case 'natural':
                // this is ambitiously called 'natural', but it's just experimental
                if (abs < 4) {
                    return change ** 2 * mult
                }
                if (abs == 4) {
                    return change ** 3
                }
                return change ** 4
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
        if (!this.isLcdConnected) {
            return
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

    async handleButtonResolve() {
        if (this.buttonResolve) {
            this.buttonResolve()
            this.buttonResolve = null
        } else {
            this.log('no buttonResolve')
        }
    }

    async handleButtonReject(err) {
        if (this.buttonReject) {
            this.buttonReject(err)
            this.buttonReject = null
            this.buttonResolve = null
        }
    }

    registerDisplayAction() {
        if (!this.isDisplayActive && this.isLcdConnected) {
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

function isLcdWriteError(err) {
    return err.errno == 121
}

module.exports = I2ciHelper