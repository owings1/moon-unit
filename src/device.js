// Serial device command HTTP service

// TODO:
//    - auto-reconnect to serial port

const SerialPort = require('serialport')
const Readline   = require('@serialport/parser-readline')
const merge      = require('merge')
const bodyParser = require('body-parser')
const express    = require('express')

class DeviceService {

    defaults() {
        return {
            baudRate    : 115200,
            autoOpen    : false,
            openDelay   : 2000,
            workerDelay : 100
        }
    }

    constructor(path, opts) {

        this.opts = merge(this.defaults(), opts)

        this.device = new SerialPort(path, this.opts)
        this.parser = this.device.pipe(new Readline)

        this.queue        = []
        this.busy         = false
        this.workerHandle = null
        this.app          = express()
        this.httpServer   = null
        this.port         = null

        this.initApp(this.app)
    }

    listen(port) {
        return new Promise((resolve, reject) => {
            try {
                this.device.open(err => {
                    if (err) {
                        reject(err)
                        return
                    }
                    this.initWorker()
                    console.log('Opened, delaying', this.opts.openDelay, 'ms')
                    setTimeout(() => {
                        try {
                            this.httpServer = this.app.listen(port, () => {
                                this.port = this.httpServer.address().port
                                console.log('Listening on port', this.port)
                                resolve()
                            })
                        } catch (err) {
                            reject(err)
                        }
                    }, this.opts.openDelay)
                })
            } catch (err) {
                reject(err)
            }
        })
    }

    close() {
        return new Promise(resolve => {
            console.log('Closing')
            if (this.httpServer) {
                this.httpServer.close()
            }
            clearInterval(this.workerHandle)
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

    initApp(app) {

        app.post('/', bodyParser.json(), (req, res) => {
            if (!req.body.command) {
                res.status(400).json({error: 'missing command'})
                return
            }
            this.request(req.body.command)
                .then(response => res.status(200).json({response}))
                .catch(error => {
                    console.error(error)
                    res.status(500).json({error})
                })
        })

        app.use((req, res) => res.status(404).json({error: 'not found'}))
    }
}

module.exports = DeviceService

if (require.main === module) {
    new DeviceService(process.env.DEVICE_SERIAL_PORT).listen(process.env.HTTP_LISTEN_PORT || '8080')
}
