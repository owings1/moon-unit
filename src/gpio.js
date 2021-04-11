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
        for (var k of ['reset', 'stop', 'state1', 'state2']) {
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

        await gpio.setup(this.pins.reset, gpio.DIR_HIGH)
        await gpio.setup(this.pins.stop, gpio.DIR_LOW)

        await gpio.setup(this.pins.state1, gpio.DIR_IN)
        await gpio.setup(this.pins.state2, gpio.DIR_IN)
    }

    async close() {
        if (!this.enabled) {
            return
        }
        gpio.destroy()
    }

    async getState() {
        if (!this.enabled) {
            return 0
        }

        const sp1 = await gpio.read(this.pins.state1)
        const sp2 = await gpio.read(this.pins.state2)

        return sp1 + sp2 * 2
    }

    async sendStop() {
        if (!this.enabled) {
            return
        }

        await gpio.write(this.pins.stop, true)
        await new Promise((resolve, reject) =>
            setTimeout(() => {
                gpio.write(this.pins.stop, false).then(resolve).catch(reject)
            }, 100)
        )
    }

    async sendReset() {
        if (!this.enabled) {
            return
        }
        await gpio.write(this.pins.reset, false)
        await new Promise((resolve, reject) =>
            setTimeout(() => {
                gpio.write(this.pins.reset, true).then(resolve).catch(reject)
            }, 100)
        )
    }
}

module.exports = Gpio