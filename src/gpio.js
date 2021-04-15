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

var gpio

class Gpio {

    constructor(enabled, pins) {
        this.enabled = enabled
        if (!enabled) {
            return
        }
        if (!pins) {
            throw new Error('Invalid argument: pins')
        }
        for (var k of ['controllerReset', 'controllerStop', 'controllerReady']) {
            if (!pins[k]) {
                throw new Error('Missing pin: ' + k)
            }
        }
        this.pins = pins
    }

    async open() {

        if (!this.enabled) {
            return
        }

        if (!gpio) {
            gpio = require('rpi-gpio').promise
        }

        await gpio.setup(this.pins.controllerReset, gpio.DIR_HIGH)
        await gpio.setup(this.pins.controllerStop, gpio.DIR_LOW)
        await gpio.setup(this.pins.controllerReady, gpio.DIR_IN)
    }

    async close() {
        if (!this.enabled) {
            return
        }
        gpio.destroy()
    }

    async isControllerReady() {
        if (!this.enabled) {
            return true
        }

        const value = await gpio.read(this.pins.controllerReady)
        return !value
    }

    async getControllerState() {
        const isReady = await this.isControllerReady()
        return isReady ? 'ready' : 'busy'
    }

    async sendControllerStop() {
        if (!this.enabled) {
            return
        }

        await gpio.write(this.pins.controllerStop, true)
        // keep stop pin on for 1 second
        await new Promise((resolve, reject) =>
            setTimeout(() => {
                gpio.write(this.pins.controllerStop, false).then(resolve).catch(reject)
            }, 1000)
        )
    }

    async sendControllerReset() {
        if (!this.enabled) {
            return
        }
        await gpio.write(this.pins.controllerReset, false)
        await new Promise((resolve, reject) =>
            setTimeout(() => {
                gpio.write(this.pins.controllerReset, true).then(resolve).catch(reject)
            }, 100)
        )
    }
}

module.exports = Gpio