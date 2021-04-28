/*
refer to them by physical position, using the diagrams on this page. So holding
the Raspberry Pi such that the GPIO header runs down the upper-right side of
the board, if you wished to address GPIO4 (which is in column 1 and row 4), you
would setup pin 7. If you wish instead to refer to the pins by their GPIO names
(known as BCM naming), you can use the setMode command described in the API
documentation below.

https://elinux.org/images/5/5c/Pi-GPIO-header.png

*/

// TODO: better button debounce

// dynamically load modules, else will fail on non-pi system
var gpio

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

        if (this.app.opts.i2ciEnabled) {
            const i2ciRequiredKeys = ['pinEncoderReset']
            for (var k of i2ciRequiredKeys) {
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

        this.log('Opening GPIO interface')

        if (!gpio) {
            gpio = require('rpi-gpio').promise
        }

        const {opts} = this.app
        // Avoid EACCES errors
        const gpioRetries = 10
        for (var i = 0; i < gpioRetries; i++) {
            try {
                await gpio.setup(opts.pinControllerReset, gpio.DIR_HIGH)
                await gpio.setup(opts.pinControllerStop, gpio.DIR_LOW)
                await gpio.setup(opts.pinControllerReady, gpio.DIR_IN)
                await gpio.setup(opts.pinGaugerReset, gpio.DIR_HIGH)
                if (this.app.opts.i2ciEnabled) {
                    await gpio.setup(opts.pinEncoderReset, gpio.DIR_HIGH)
                }
            } catch (err) {
                if (i >= (gpioRetries - 1)) {
                    throw err
                }
                if (err.code == 'EACCES') {
                    this.error('Failed to open GPIO', err.message)
                    await new Promise(resolve => setTimeout(resolve, 3000))
                    this.log('Retrying to open GPIO')
                }
            }
        }
    }

    async close() {
        if (!this.enabled) {
            return
        }        
        try {
            gpio.destroy()
        } catch (err) {
            this.error('Error closing GPIO', err)
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

    async resetController() {
        if (!this.enabled) {
            return
        }
        await this._sendReset(this.app.opts.pinControllerReset)
    }

    async resetGauger() {
        if (!this.enabled) {
            return
        }
        await this._sendReset(this.app.opts.pinGaugerReset)
        await new Promise(resolve => setTimeout(resolve, this.app.opts.resetDelay))
    }

    async resetEncoder() {
        if (!this.enabled) {
            return
        }
        await this._sendReset(this.app.opts.pinEncoderReset)
        await new Promise(resolve => setTimeout(resolve, 2000))
    }

    async _sendReset(pin) {
        await gpio.write(pin, false)
        await new Promise((resolve, reject) => {
            setTimeout(() => {
                gpio.write(pin, true).then(resolve).catch(reject)
            }, 100)
        })
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

module.exports = GpioHelper