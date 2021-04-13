// Serial device command HTTP service

// TODO:
//    - API to disconnect/reconnect to serial port
const fs         = require('fs')
const merge      = require('merge')
const bodyParser = require('body-parser')
const express    = require('express')
const path       = require('path')
const prom       = require('prom-client')
const showdown   = require('showdown')

const MockBinding = require('@serialport/binding-mock')
const SerPortFull = require('serialport')
const SerPortMock = require('@serialport/stream')
const Readline    = require('@serialport/parser-readline')

prom.collectDefaultMetrics()

const DeviceCodes = {
     0: 'OK',
     1: 'Device closed',
     2: 'Command timeout',
     3:  'Flush error',
    40: 'Missing : before command',
    44: 'Invalid command',
    45: 'Invalid motorId',
    46: 'Invalid direction',
    47: 'Invalid steps/degrees',
    48: 'Invalid speed/acceleration',
    49: 'Invalid other parameter',
    50: 'Orientation unavailable'
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
            pinState2   : +env.PIN_STATE2 || 36,
            // how long to wait after reset to reopen device
            resetDelay  : +env.RESET_DELAY || 5000,
            commandTimeout : +env.COMMAND_TIMEOUT || 5000
        }
    }

    constructor(opts, env) {

        this.opts = merge(this.defaults(env), opts)

        if (!this.opts.path) {
            throw new ConfigError('path not set, you can use DEVICE_SERIAL_PORT')
        }

        this.queue        = []
        this.busy         = false
        this.workerHandle = null
        this.app          = express()
        this.httpServer   = null
        this.position     = [null, null]
        this.orientation  = [null, null, null]

        this.isConnected = false

        this.initApp(this.app)
    }

    async status() {
        const state = this.gpio ? (await this.gpio.getState()) : null
        return {
            state,
            position    : this.position,
            orientation : this.orientation,
            isConnected : this.isConnected,
            connectedStatus: this.isConnected ? 'Connected' : 'Disconnected'
        }
    }

    listen() {
        return new Promise((resolve, reject) => {
            try {
                this.initGpio().then(() => {
                    this.httpServer = this.app.listen(this.opts.port, () => {
                        this.log('Listening on', this.httpServer.address())
                        this.openDevice().then(resolve).catch(reject)
                    })
                }).catch(reject)
            } catch (err) {
                reject(err)
            }
        })
    }

    async openDevice() {
        this.closeDevice()
        this.log('Opening device', this.opts.path)
        this.device = this.createDevice()
        await new Promise((resolve, reject) => {
            this.device.open(err => {
                if (err) {
                    reject(err)
                    return
                }
                this.isConnected = true
                this.log('Connected, delaying', this.opts.openDelay, 'ms')
                this.parser = this.device.pipe(new Readline)
                setTimeout(() => {
                    try {
                        this.initWorker()
                        resolve()
                    } catch (err) {
                        reject(err)
                    }
                }, this.opts.openDelay)
            })
        })
    }

    closeDevice() {
        if (this.device) {
            this.log('Closing device')
            this.device.close()
            this.device = null
        }
        this.isConnected = false
        this.position    = [null, null]
        this.orientation = [null, null, null]
        this.drainQueue()
        this.stopWorker()
    }

    close() {
        return new Promise(resolve => {
            this.log('Shutting down')
            this.closeDevice()
            if (this.httpServer) {
                this.httpServer.close()
            }
            resolve()
        })
    }

    commandSync(body, params = {}) {
        return new Promise((resolve, reject) => {
            this.log('Enqueuing command', body.trim())
            this.queue.unshift({isSystem: false, ...params, body, handler: resolve})
        })
    }

    loop() {

        if (this.busy) {
            return
        }

        this.busy = true

        this.gpio.getState().then(state => {

            if (state != 0) {
                this.busy = false
                return
            }

            if (this.queue.length) {
                var {body, handler, isSystem} = this.queue.pop()
            } else {
                // TODO: various update tasks, e.g. motorSpeed
                var {body, handler, isSystem} = this.getPositionJob()
            }

            this.flushDevice().then(() => {

                var isComplete = false

                this.parser.once('data', resText => {
                    isComplete = true
                    // handle device response
                    if (!isSystem) {
                        this.log('Receieved response:', resText)
                    }
                    const status = parseInt(resText.substring(1, 3))
                    handler({
                        status,
                        message : DeviceCodes[status],
                        body    : resText.substring(4),
                        raw     : resText
                    })
                    this.busy = false
                })

                if (!isSystem) {
                    this.log('Sending command', body.trim())
                }

                this.device.write(Buffer.from(body))

                setTimeout(() => {
                    if (!isComplete) {
                        this.error('Command timeout', body.trim())
                        this.parser.emit('data', '=02;')
                    }
                }, this.opts.commandTimeout)
            }).catch(err => {
                this.error('Flush failed', err)
                const status = 3
                handler({
                    status,
                    message: DeviceCodes[status],
                    body   : '',
                    raw    : '=03;',
                    error  : err.message
                })
            })
        })
    }

    initWorker() {
        this.log('Initializing worker to run every', this.opts.workerDelay, 'ms')
        this.stopWorker()
        this.workerHandle = setInterval(() => this.loop(), this.opts.workerDelay)
    }

    stopWorker() {
        clearInterval(this.workerHandle)
        this.busy = false
    }

    drainQueue() {
        while (this.queue.length) {
            var {handler} = this.queue.pop()
            this.log('Sending error 1 response to handler')
            handler({status: 1, message: DeviceCodes[1]})
        }
    }

    async flushDevice() {
        return this.device.flush()
    }

    initApp(app) {

        app.set('view engine', 'ejs')
        app.set('views', __dirname + '/views')

        app.use('/static', express.static(__dirname + '/static'))

        app.get('/', (req, res) => {
            this.status().then(status => {
                res.render('index', {
                    title: 'MoonUnit',
                    status
                })
            })
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
                    this.commandSync(req.body.command)
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

        app.get('/status', (req, res) => {
            this.status().then(status => res.status(200).json({status}))
        })

        app.post('/disconnect', (req, res) => {
            this.closeDevice()
            this.status().then(status => {
                res.status(200).json({message: 'Device disconnected', status})
            })
        })

        app.post('/connect', (req, res) => {
            if (this.isConnected) {
                res.status(400).json({message: 'Device already connected'})
                return
            }
            this.openDevice().then(() => {
                this.status().then(status => {
                    res.status(200).json({message: 'Device connected', status})
                })
            }).catch(error => {
                res.status(500).json({error})
            })
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
            this.closeDevice()
            this.log('Sending reset')
            this.gpio.sendReset().then(() => {
                res.status(200).json({message: 'reset sent'})
                this.log('Reset sent, delaying', this.opts.resetDelay, 'to reopen')
                setTimeout(() => {
                    this.openDevice().catch(err => this.error(err))
                }, this.opts.resetDelay)
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

        app.get('/doc/:filename', (req, res) => {
            const file = path.resolve(__dirname, '../doc', path.basename(req.params.filename) + '.md')
            fs.readFile(file, 'utf-8', (error, text) => {
                if (error) {
                    if (error.code == 'ENOENT') {
                        res.status(404)
                    } else {
                        res.status(400)
                    }
                    res.json({error})
                    return
                }
                const converter = new showdown.Converter({
                    tables: true
                })
                const html = converter.makeHtml(text)
                res.render('doc', {html})
            })
        })

        app.use((req, res) => res.status(404).json({error: 'not found'}))
    }

    createDevice() {
        var SerialPort = SerPortFull
        if (this.opts.mock) {
            SerPortMock.Binding = MockBinding
            var SerialPort = SerPortMock
            // TODO: mock response
            //  see: https://serialport.io/docs/api-binding-mock
            //  see: https://github.com/serialport/node-serialport/blob/master/packages/binding-mock/lib/index.js
            MockBinding.createPort(this.opts.path, {echo: true, readyData: []})
        }
        return new SerialPort(this.opts.path, {baudRate: this.opts.baudRate, autoOpen: false})
    }

    getPositionJob() {
        return {
            isSystem : true,
            body     : ':15 ;\n',
            handler  : res => {
                if (res.status != 0) {
                    if (!this.opts.mock) {
                        this.error('Failed to get positions', res)
                    }
                    return
                }
                // normalize NaN, undefined, etc. to null
                const nums = JSON.parse(
                    JSON.stringify(
                        res.body.split('|').map(parseFloat)
                    )
                )
                this.position = [nums[0], nums[1]]
                this.orientation = [nums[2], nums[3], nums[4]]
            }
        }
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
