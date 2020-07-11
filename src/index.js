

const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')
const merge = require('merge')

class SerialDevice {

    defaults() {
        return {
            baudRate: 115200,
            autoOpen: false.
            openDelay: 2000,
            workerDelay: 100
        }
    }

    constructor(path, opts) {
        this.opts = merge(this.defaults(), opts)
        this.device = new SerialPort(path, opts)
        this.parser = this.device.pipe(new Readline)
        this.queue = []
        this.busy = false
        this.workerHandle = null
    }

    open() {
        return new Promise((resolve, reject) => {
            this.device.open(err => {
                if (err) {
                    reject(err)
                    return
                }
                this.initWorker()
                console.log('Opened, delaying', openDelay, 'ms')
                setTimeout(resolve, this.opts.openDelay)
            })
        })
    }

    close() {
        return new Promise(resolve => {
            console.log('Closing')
            this.device.close(resolve)
        })
    }

    request(body) {
        return new Promise((resolve, reject) => {
            console.log('Enqueuing command', body)
            this.queue.unshift({body, handler: resolve})
        })
    }

    loop() {
        if (this.busy || !this.queue.length) {
            return
        }
        this.busy = true
        const {body, handler} = this.queue.pop()
        this.parser.once('data', resText => {
            // handle device response
            console.log('Receieved response:', resText)
            handler({
                status : parseInt(resText.substring(1))
            })
            this.busy = false
        })
        this.device.write(body)
    }

    initWorker() {
        clearInterval(this.workerHandle)
        this.workerHandle = setInterval(() => this.loop(), this.opts.workerDelay)
    }
}


const device = new SerialDevice(process.env.MC_SERIAL_PORT)

device.open().then(() => {

    device.request(':01 1 1 1600;').then(res => {
        console.log(1, res)
    })

    device.request(':01 1 2 1600;'.then(res => {
        console.log(2, res)
    })

    device.request(':01 1 1 1600;'.then(res => {
        console.log(3, res)
    })

    device.request(':01 1 2 1600;'.then(res => {
        console.log(4, res)
        setTimeout(() => device.close(), 2000)
    })
})

