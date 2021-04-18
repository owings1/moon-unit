// Serial device command HTTP service

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
    50: 'Orientation unavailable',
    51: 'Limits unavailable'
}

const Gpio = require('./gpio')

class App {

    defaults(env) {
        env = env || process.env
        return {
            //controllerPath     : env.CONTROLLER_PORT,
            //controllerBaudRate : +env.CONTROLLER_BAUD_RATE || 9600, //115200,
            gaugerPath         : env.GAUGER_PORT,
            gaugerBaudRate     : +env.GAUGER_BAUD_RATE || 9600, //115200,
            mock        : !!env.MOCK,
            port        : env.HTTP_PORT || 8080,
            quiet       : !!env.QUIET,
            openDelay   : +env.OPEN_DELAY || 2000,
            workerDelay : +env.WORKER_DELAY || 100,
            
            
            gpioEnabled : !!env.GPIO_ENABLED,
            pinControllerReset  : +env.PIN_CONTROLLER_RESET || 37,
            pinControllerStop   : +env.PIN_CONTROLLER_STOP || 35,
            pinControllerReady  : +env.PIN_CONTROLLER_READY || 38,
            pinGaugerReset      : +env.PIN_GAGUER_RESET || 36,
            //pinState2   : +env.PIN_STATE2 || 36,
            // how long to wait after reset to reopen device
            resetDelay     : +env.RESET_DELAY || 5000,
            commandTimeout : +env.COMMAND_TIMEOUT || 5000
        }
    }

    constructor(opts, env) {

        this.opts = merge(this.defaults(env), opts)

        /*
        if (!this.opts.controllerPath) {
            throw new ConfigError('path not set, you can use CONTROLLER_PORT')
        }

        this.controllerQueue        = []
        this.controllerBusy         = false
        this.controllerWorkerHandle = null
        this.isControllerConnected  = false
        */

        this.gaugerJobs         = {}
        this.gaugerQueue        = []
        this.gaugerBusy         = false
        this.gaugerWorkerHandle = null
        this.isGaugerConnected  = false
        
        this.app        = express()
        this.httpServer = null

        this.clearStatus()
        this.clearGauges()
        this.initApp(this.app)
    }

    clearStatus() {
        this.position      = [null, null]
        this.limitsEnabled = [null, null]
    }

    clearGauges() {
        this.gpsCoords = [null, null]
        this.magHeading = null
        this.declinationAngle = null
        this.orientation   = [null, null, null]
        this.isOrientationCalibrated = null
    }

    async status() {
        const controllerState = this.gpio ? (await this.gpio.getControllerState()) : null
        return {
            controllerState,
            position                  : this.position,
            orientation               : this.orientation,
            //isControllerConnected     : this.isControllerConnected,
            limitsEnabled             : this.limitsEnabled,
            isGaugerConnected         : this.isGaugerConnected,
            isOrientationCalibrated   : this.isOrientationCalibrated,
            //controllerConnectedStatus : this.isControllerConnected ? 'Connected' : 'Disconnected',
            gaugerConnectedStatus     : this.isGaugerConnected ? 'Connected' : 'Disconnected',
            gpsCoords                 : this.gpsCoords,
            magHeading                : this.magHeading,
            declinationAngle          : this.declinationAngle
        }
    }

    listen() {
        return new Promise((resolve, reject) => {
            try {
                this.initGpio().then(() => {
                    this.httpServer = this.app.listen(this.opts.port, () => {
                        this.log('Listening on', this.httpServer.address())
                        this.openGauger().then(resolve).catch(reject)
                        /*
                        this.openController().then(() =>
                            this.openGauger()
                        ).then(resolve).catch(reject)
                        */
                    })
                }).catch(reject)
            } catch (err) {
                reject(err)
            }
        })
    }

    /*
    async openController() {
        this.closeController()
        this.log('Opening controller', this.opts.controllerPath)
        this.controller = this.createDevice(this.opts.controllerPath, this.opts.controllerBaudRate)
        await new Promise((resolve, reject) => {
            this.controller.open(err => {
                if (err) {
                    reject(err)
                    return
                }
                this.isControllerConnected = true
                this.log('Controller opened, delaying', this.opts.openDelay, 'ms')
                this.controllerParser = this.controller.pipe(new Readline)
                setTimeout(() => {
                    try {
                        this.initControllerWorker()
                        resolve()
                    } catch (err) {
                        reject(err)
                    }
                }, this.opts.openDelay)
            })
        })
    }

    closeController() {
        if (this.controller) {
            this.log('Closing controller')
            this.controller.close()
            this.controller = null
        }
        this.isControllerConnected = false
        this.clearStatus()
        this.drainControllerQueue()
        this.stopControllerWorker()
    }
    */

    async openGauger() {
        this.closeGauger()
        this.log('Opening gauger', this.opts.gaugerPath)
        this.gauger = this.createDevice(this.opts.gaugerPath, this.opts.gaugerBaudRate)
        await new Promise((resolve, reject) => {
            this.gauger.open(err => {
                if (err) {
                    reject(err)
                    return
                }
                this.isGaugerConnected = true
                this.log('Gauger opened, delaying', this.opts.openDelay, 'ms')
                this.gaugerParser = this.gauger.pipe(new Readline)
                setTimeout(() => {
                    try {
                        this.initGaugerWorker()
                        this.gauger.flush()
                        this.gaugerParser.on('data', data => {
                            if (data.indexOf('ACK:') == 0) {
                                this.handleGaugerAckData(data)
                            } else {
                                this.handleGaugeData(data)
                            }
                        })
                        this.log('Setting gauger to streaming mode')
                        this.gaugerCommand(':71 2;\n').then(res => {
                            if (res.status != 0) {
                                this.error('Failed to set gauger to streaming mode', res)
                                return
                            }
                            this.log('Gauger acknowledges streaming mode')
                        })
                        resolve()
                    } catch (err) {
                        reject(err)
                    }
                }, this.opts.openDelay)
            })
        })
    }

    handleGaugerAckData(data) {
        const [ack, id, resText] = data.split(':')
        const status = parseInt(resText.substring(1, 3))
        if (this.gaugerJobs[id]) {
            this.log('Gauger ACK job', id)
            this.gaugerJobs[id].handler({
                status,
                message : DeviceCodes[status],
                body    : resText.substring(4),
                raw     : resText
            })
        } else {
            this.log('Unknown gauger job ackd', id)
        }
    }

    handleGaugeData(data) {
        const [module, text] = data.split(':')
        const values = text.split('|')
        const floats = Util.floats(values)
        switch (module) {
            case 'GPS':
                this.gpsCoords = values.map(parseFloat)
                break
            case 'MAG':
                const values = values.map(parseFloat)
                this.magHeading = floats[0]
                this.declinationAngle = floats[4]
                break
            case 'ORI':
                // x|y|z|cal_system|cal_gyro|cal_accel|cal_mag|isCalibrated
                this.orientation = floats.slice(0, 3)
                this.isOrientationCalibrated = values[7] == 'T'
            case 'MOD':
                // names the modules available
                break
            default:
                this.log('Unknown module', module)
                break
        }
    }

    closeGauger() {
        if (this.gauger) {
            this.log('Closing gauger')
            this.gauger.close()
            this.gauger = null
        }
        this.isGaugerConnected = false
        this.drainGaugerQueue()
        
        this.clearStatus()
        this.stopGaugerWorker()
    }

    drainGaugerQueue() {
        this.gaugerJobs = {}
        this.gaugerQueue = []
    }

    close() {
        return new Promise(resolve => {
            this.log('Shutting down')
            //this.closeController()
            this.closeGauger()
            if (this.httpServer) {
                this.httpServer.close()
            }
            resolve()
        })
    }

    /*
    controllerCommand(body, params = {}) {
        return new Promise((resolve, reject) => {
            this.log('Enqueuing controller command', body.trim())
            this.controllerQueue.unshift({isSystem: false, ...params, body, handler: resolve})
        })
    }

    controllerLoop() {

        if (this.controllerBusy) {
            return
        }

        this.controllerBusy = true

        this.gpio.isControllerReady().then(isReady => {

            if (!isReady) {
                this.controllerBusy = false
                return
            }

            if (this.controllerQueue.length) {
                var {body, handler, isSystem} = this.controllerQueue.pop()
            } else {
                // TODO: various update tasks, e.g. motorSpeed
                var {body, handler, isSystem} = this.getPositionJob()
            }

            this.flushController().then(() => {

                var isComplete = false

                this.controllerParser.once('data', resText => {
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
                    this.controllerBusy = false
                })

                if (!isSystem) {
                    this.log('Sending command', body.trim())
                }

                this.controller.write(Buffer.from(this.opts.mock ? body : body.trim()))

                // TODO: rethink timeout, this is causing errors
                //setTimeout(() => {
                //    if (!isComplete) {
                //        this.error('Command timeout', body.trim())
                //        this.controllerParser.emit('data', '=02;')
                //    }
                //}, this.opts.commandTimeout)
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
    */
    gaugerLoop() {
        if (this.gaugerBusy) {
            return
        }

        if (!this.gaugerQueue.length) {
            // TODO: get controller position
            return   
        }

        this.gaugerBusy = true

        const {id, body, handler} = this.gaugerQueue.pop()
        this.gaugerJobs[id] = {
            handler: res => {
                this.gaugerBusy = false
                if (handler) {
                    handler(res)
                }
            }
        }
        this.gauger.write(Buffer.from(this.opts.mock ? body : body.trim()))
    }

    gaugerCommand(body, params = {}) {
        const id = this._newGaugerJobId()
        body = ':' + id + body
        return new Promise((resolve, reject) => {
            this.log('Enqueuing gauger command', body.trim())
            this.gaugerQueue.unshift({isSystem: false, ...params, body, id, handler: resolve})
        })
    }

    _newGaugerJobId() {
        if (!this._gid || this._gid > 2 * 1000 * 1000 * 1000) {
            this._gid = 0
        }
        return ++this._gid
    }

    /*
    initControllerWorker() {
        this.log('Initializing controller worker to run every', this.opts.workerDelay, 'ms')
        this.stopControllerWorker()
        this.controllerWorkerHandle = setInterval(() => this.controllerLoop(), this.opts.workerDelay)
    }

    stopControllerWorker() {
        clearInterval(this.controllerWorkerHandle)
        this.controllerBusy = false
    }

    drainControllerQueue() {
        while (this.controllerQueue.length) {
            var {handler} = this.controllerQueue.pop()
            this.log('Sending error 1 response to handler')
            handler({status: 1, message: DeviceCodes[1]})
        }
    }
    */

    initGaugerWorker() {
        this.log('Initializing gauger worker to run every', this.opts.workerDelay, 'ms')
        this.stopGaugerWorker()
        this.gaugerWorkerHandle = setInterval(() => this.gaugerLoop(), this.opts.workerDelay)
    }

    stopGaugerWorker() {
        clearInterval(this.gaugerWorkerHandle)
        this.gaugerBusy = false
    }

    /*
    async flushController() {
        // TODO: figure out why device.flush does not return a promise
        // commenting out for debug (getting errors)
        //return this.controller.flush()
    }
    */

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

        app.get('/status', (req, res) => {
            this.status().then(status => res.status(200).json({status}))
        })

        app.post('/controller/command/sync', bodyParser.json(), (req, res) => {
            if (!req.body.command) {
                res.status(400).json({error: 'missing command'})
                return
            }
            try {
                this.gpio.isControllerReady().then(isReady => {
                    if (!isReady) {
                        res.status(503).json({error: 'not ready', isReady})
                        return
                    }
                    //this.controllerCommand(req.body.command)
                    this.gaugerCommand(req.body.command)
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

        /*
        app.post('/controller/disconnect', (req, res) => {
            this.closeController()
            this.status().then(status => {
                res.status(200).json({message: 'Device disconnected', status})
            })
        })

        app.post('/controller/connect', (req, res) => {
            if (this.isControllerConnected) {
                res.status(400).json({message: 'Device already connected'})
                return
            }
            this.openController().then(() => {
                this.status().then(status => {
                    res.status(200).json({message: 'Device connected', status})
                })
            }).catch(error => {
                res.status(500).json({error})
            })
        })
        */

        app.post('/gauger/command/sync', (req, res) => {
            if (!req.body.command) {
                res.status(400).json({error: 'missing command'})
                return
            }
            try {
                this.gaugerCommand(req.body.command)
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

        app.post('/gauger/disconnect', (req, res) => {
            this.closeGauger()
            this.status().then(status => {
                res.status(200).json({message: 'Device disconnected', status})
            })
        })

        app.post('/gauger/connect', (req, res) => {
            if (this.isGaugerConnected) {
                res.status(400).json({message: 'Device already connected'})
                return
            }
            this.openGauger().then(() => {
                this.status().then(status => {
                    res.status(200).json({message: 'Device connected', status})
                })
            }).catch(error => {
                res.status(500).json({error})
            })
        })

        app.get('/controller/gpio/state', (req, res) => {
            if (!this.opts.gpioEnabled) {
                res.status(400).json({error: 'gpio not enabled'})
                return
            }
            this.gpio.getControllerState().then(state =>
                res.status(200).json({state})
            ).catch(error => {
                this.error(error)
                res.status(500).json({error})
            })
        })

        app.post('/controller/gpio/reset', (req, res) => {
            if (!this.opts.gpioEnabled) {
                res.status(400).json({error: 'gpio not enabled'})
                return
            }
            this.closeController()
            this.log('Sending reset')
            this.gpio.sendControllerReset().then(() => {
                res.status(200).json({message: 'reset sent'})
                this.log('Reset sent, delaying', this.opts.resetDelay, 'to reopen')
                setTimeout(() => {
                    this.openController().catch(err => this.error(err))
                }, this.opts.resetDelay)
            }).catch(error => {
                this.error(error)
                res.status(500).json({error})
            })
        })

        app.post('/controller/gpio/stop', (req, res) => {
            if (!this.opts.gpioEnabled) {
                res.status(400).json({error: 'gpio not enabled'})
                return
            }
            this.gpio.sendControllerStop().then(() => {
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

    createDevice(devicePath, baudRate) {
        var SerialPort = SerPortFull
        if (this.opts.mock) {
            SerPortMock.Binding = MockBinding
            var SerialPort = SerPortMock
            // TODO: mock response
            //  see: https://serialport.io/docs/api-binding-mock
            //  see: https://github.com/serialport/node-serialport/blob/master/packages/binding-mock/lib/index.js
            MockBinding.createPort(devicePath, {echo: true, readyData: []})
        }
        return new SerialPort(devicePath, {baudRate, autoOpen: false})
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
               
                const arr = res.body.split('|')
                const floats = Util.floats(arr)
                this.position = [floats[0], floats[1]]
                this.limitsEnabled = [arr[2], arr[3]].map(it => it == 'T')
            }
        }
    }

    async initGpio() {

        this.log('Gpio is', this.opts.gpioEnabled ? 'enabled' : 'disabled')

        this.gpio = new Gpio(this.opts.gpioEnabled, {
            controllerReset    : this.opts.pinControllerReset, 
            controllerStop     : this.opts.pinControllerStop,
            controllerReady   : this.opts.pinControllerReady
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
