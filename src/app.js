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

const DeviceCodes = {
     0: 'OK',
    40: 'Missing : before command',
    44: 'Invalid command',
    45: 'Invalid motorId',
    46: 'Invalid direction',
    47: 'Invalid steps/degrees',
    48: 'Invalid speed/acceleration',
    49: 'Invalid other parameter'
}

const Gpio = require('./gpio')

class App {

    defaults(env) {
        env = env || process.env
        return {
            path        : env.DEVICE_SERIAL_PORT,
            mock        : !!env.MOCK,
            port        : env.HTTP_PORT || 8080,
            quiet       : !!env.QUIET,
            openDelay   : +env.OPEN_DELAY || 2000,
            workerDelay : +env.WORKER_DELAY || 100,
            baudRate    : +env.BAUD_RATE || 115200,
            gpioEnabled : !!env.GPIO_ENABLED,
            pinReset    : +env.PIN_RESET || 37,
            pinStop     : +env.PIN_STOP || 35,
            pinState1   : +env.PIN_STATE1 || 38,
            pinState2   : +env.PIN_STATE2 || 36
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
                this.log('Opening device', this.opts.path)
                this.device.open(err => {
                    if (err) {
                        reject(err)
                        return
                    }
                    this.initGpio().then(() => {
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
                    }).catch(reject)
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
            this.device.close()
            resolve()
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
            const status = parseInt(resText.substring(1))
            const cidx = resText.indexOf(';')
            if (cidx) {
                var resBody = resText.substring(cidx + 1)
            }
            handler({
                status,
                message: DeviceCodes[status],
                body: resBody
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
        app.set('views', __dirname + '/views')

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
                this.gpio.getState().then(state => {
                    if (state != 0) {
                        res.status(503).json({error: 'not ready', state})
                        return
                    }
                    this.command(req.body.command)
                        .then(response => res.status(200).json({response}))
                        .catch(error => {
                            this.error(error)
                            res.status(500).json({error})
                        })
                }).catch(error => {
                    this.error(error)
                    res.status(500).json({error})
                })
            } catch (error) {
                this.error(error)
                res.status(500).json({error})
            }
        })

        app.get('/gpio/state', (req, res) => {
            if (!this.opts.gpioEnabled) {
                res.status(400).json({error: 'gpio not enabled'})
                return
            }
            this.gpio.getState().then(
                state => res.status(200).json({state})
            ).catch(error => {
                this.error(error)
                res.status(500).json({error})
            })
        })

        app.post('/gpio/reset', (req, res) => {
            if (!this.opts.gpioEnabled) {
                res.status(400).json({error: 'gpio not enabled'})
                return
            }
            // TODO: gracefully close serial port or catch error and delay then repoen
            this.gpio.sendReset().then(() => {
                res.status(200).json({message: 'reset sent'})
                //this.device.close()
                //setTimeout(() => this.device.open().then(() => this.log('Reopened')).catch(err => this.error(err)), 4000)
            }).catch(error => {
                this.error(error)
                res.status(500).json({error})
            })
        })

        app.post('/gpio/stop', (req, res) => {
            if (!this.opts.gpioEnabled) {
                res.status(400).json({error: 'gpio not enabled'})
                return
            }
            this.gpio.sendStop().then(() => {
                res.status(200).json({message: 'stop sent'})
            }).catch(error => {
                this.error(error)
                res.status(500).json({error})
            })
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

    async initGpio() {

        this.log('Gpio is', this.opts.gpioEnabled ? 'enabled' : 'disabled')

        this.gpio = new Gpio(this.opts.gpioEnabled, {
            reset    : this.opts.pinReset, 
            stop     : this.opts.pinStop,
            state1   : this.opts.pinState1,
            state2   : this.opts.pinState2
        })
        await this.gpio.open()
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

module.exports = App
