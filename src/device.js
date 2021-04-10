// Serial device command HTTP service

// TODO:
//    - auto-reconnect to serial port
const merge      = require('merge')
const bodyParser = require('body-parser')
const express    = require('express')
const prom       = require('prom-client')

const MockBinding = require('@serialport/binding-mock')
const SerPortFull = require('serialport')
const SerPortMock = require('@serialport/stream')
const Readline    = require('@serialport/parser-readline')

prom.collectDefaultMetrics()

class DeviceService {

    defaults(env) {
        env = env || process.env
        return {
            path        : env.DEVICE_SERIAL_PORT,
            mock        : !!env.MOCK,
            port        : env.HTTP_PORT || 8080,
            quiet       : !!env.QUIET,
            openDelay   : +env.OPEN_DELAY || 2000,
            workerDelay : +env.WORKER_DELAY || 100,
            baudRate    : +env.BAUD_RATE || 115200
        }
    }

    constructor(opts, env) {

        this.opts = merge(this.defaults(env), opts)

        if (!this.opts.path) {
            throw new ConfigError('path not set, you can use DEVICE_SERIAL_PORT')
        }
        this.device = this.createDevice()
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
        port = port || this.opts.port
        return new Promise((resolve, reject) => {
            try {
                this.device.open(err => {
                    if (err) {
                        reject(err)
                        return
                    }
                    this.initWorker()
                    this.log('Opened, delaying', this.opts.openDelay, 'ms')
                    setTimeout(() => {
                        try {
                            this.httpServer = this.app.listen(port, () => {
                                this.port = this.httpServer.address().port
                                this.log('Listening on port', this.port)
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
            this.log('Closing')
            if (this.httpServer) {
                this.httpServer.close()
            }
            clearInterval(this.workerHandle)
            this.device.close(resolve)
        })
    }

    command(body) {
        return new Promise((resolve, reject) => {
            this.log('Enqueuing command', body)
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
            this.log('Receieved response:', resText)
            handler({
                status : parseInt(resText.substring(1))
            })
            this.busy = false
        })
        this.log('Sending command')
        this.device.write(Buffer.from(body))
    }

    initWorker() {
        clearInterval(this.workerHandle)
        this.workerHandle = setInterval(() => this.loop(), this.opts.workerDelay)
    }

    initApp(app) {

        app.set('view engine', 'ejs')

        app.use('/static', express.static(__dirname + '/static'))

        app.get('/', (req, res) => {
            res.render('index')
        })

        app.post('/command/sync', bodyParser.json(), (req, res) => {
            if (!req.body.command) {
                res.status(400).json({error: 'missing command'})
                return
            }
            try {
                this.command(req.body.command)
                    .then(response => res.status(200).json({response}))
                    .catch(error => {
                        this.error(error)
                        res.status(500).json({error})
                    })
            } catch (error) {
                this.error(error)
                res.status(500).json({error})
            }
        })

        app.get('/metrics', (req, res) => {
            res.setHeader('Content-Type', prom.register.contentType)
            prom.register.metrics().then(metrics => res.writeHead(200).end(metrics))
        })

        app.use((req, res) => res.status(404).json({error: 'not found'}))
    }

    createDevice() {
        var SerialPort = SerPortFull
        if (this.opts.mock) {
            SerPortMock.Binding = MockBinding
            var SerialPort = SerPortMock
            // TODO: mock response
            MockBinding.createPort(this.opts.path, {echo: true, readyData: []})
        }
        return new SerialPort(this.opts.path, {baudRate: this.opts.baudRate, autoOpen: false})
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

class BaseError extends Error {
    constructor(...args) {
        super(...args)
        this.name = this.constructor.name
    }
}

class ConfigError extends BaseError {}

module.exports = DeviceService

if (require.main === module) {
    new DeviceService().listen()
}
