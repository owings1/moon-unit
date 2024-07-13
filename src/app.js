// Serial device command HTTP service

// TODO: garbage collect unacked gauger jobs

const fs         = require('fs')
const bodyParser = require('body-parser')
const express    = require('express')
const merge      = require('merge')
const os         = require('os')
const path       = require('path')
const prom       = require('prom-client')
const showdown   = require('showdown')
const Util       = require('./util')

const MockBinding = require('@serialport/binding-mock')
const SerPortFull = require('serialport')
const SerPortMock = require('@serialport/stream')
const Readline    = require('@serialport/parser-readline')

const DEG_NULL = 1000

prom.collectDefaultMetrics()

const DeviceCodes = {
     0: 'OK',
     1: 'Device closed',
     2: 'Command timeout',
     3: 'Flush error',
    40: 'Missing : before command',
    44: 'Invalid command',
    45: 'Invalid motorId',
    46: 'Invalid direction',
    47: 'Invalid steps/degrees',
    48: 'Invalid speed/acceleration',
    49: 'Invalid other parameter'
}

const GpioHelper = require('./gpio')
const I2ciHelper = require('./i2ci')
const WpaHelper  = require('./wpa')

class App {

    defaults(env) {
        env = env || process.env
        return {
            gaugerPath     : env.GAUGER_PORT,
            gaugerBaudRate : +env.GAUGER_BAUD_RATE || 9600,
            mock           : !!env.MOCK,
            port           : env.HTTP_PORT || 8080,
            quiet          : !!env.QUIET,
            openDelay      : +env.OPEN_DELAY || 4000,
            workerDelay    : +env.WORKER_DELAY || 100,

            gpioEnabled        : !!env.GPIO_ENABLED,
            pinControllerReset : +env.PIN_CONTROLLER_RESET || 13,
            pinControllerStop  : +env.PIN_CONTROLLER_STOP || 18,
            pinControllerReady : +env.PIN_CONTROLLER_READY || 16,
            pinGaugerReset     : +env.PIN_GAUGER_RESET || 11,

            // how long to wait after reset to reopen device
            resetDelay     : +env.RESET_DELAY || 5000,
            commandTimeout : +env.COMMAND_TIMEOUT || 5000,

            netInfoIface   : env.NETINFO_IFACE,
            wpaEnabled     : !!env.WPA_ENABLED,
            wpaConf        : env.WPA_CONF || '/etc/wpa_supplicant/wpa_supplicant.conf'
        }
    }

    constructor(opts, env) {

        this.opts = merge(this.defaults(env), opts)

        this.gaugerJobs         = {}
        this.gaugerQueue        = []
        this.gaugerBusy         = false
        this.gaugerWorkerHandle = null
        this.isGaugerConnected  = false
        
        this.app        = express()
        this.httpServer = null

        this.clearGauges()
        this.templateHelper = new TemplateHelper
        this.initApp(this.app)

        this.netInfo = {ip: null}
        this.wpa = new WpaHelper(this)
        this.declinationData = {}
        this.declinationAngle = null
        this.declinationSource = null
    }

    clearGauges() {

        this.isMcInit      = null
        this.isMciInit     = null
        this.position      = [null, null]
        this.limitsEnabled = [null, null]
        this.limitStates   = [null, null, null, null]
        this.maxSpeeds     = [null, null]
        this.accelerations = [null, null]

        this.isGpsInit = null
        this.gpsCoords = [null, null]

        this.isMagInit  = null
        this.magHeading = null

        this.declinationAngle  = null
        this.declinationSource = null

        this.isOrientationInit       = null
        this.orientation             = [null, null, null, null, null, null, null]
        this.temperature             = null
        this.orientationCalibration  = [null, null, null, null]
        this.isOrientationCalibrated = null

        this.isBaseOrientationInit       = null
        this.baseOrientation             = [null, null, null, null, null, null, null]
        this.baseTemperature             = null
        this.baseOrientationCalibration  = [null, null, null, null]
        this.isBaseOrientationCalibrated = null

    }

    async status() {
        const controllerState = (this.opts.gpioEnabled && this.gpio) ? (await this.gpio.getControllerState()) : null
        return {
            controllerState,
            gpioEnabled                 : this.opts.gpioEnabled,
            isGaugerConnected           : this.isGaugerConnected,
            gaugerConnectedStatus       : this.isGaugerConnected ? 'Connected' : 'Disconnected',

            isMcInit                    : this.isMcInit,
            isMciInit                   : this.isMciInit,
            position                    : this.position,
            limitsEnabled               : this.limitsEnabled,
            limitStates                 : this.limitStates,
            maxSpeeds                   : this.maxSpeeds,
            accelerations               : this.accelerations,

            isOrientationInit           : this.isOrientationInit,
            isOrientationCalibrated     : this.isOrientationCalibrated,
            orientation                 : this.orientation,
            temperature                 : this.temperature,
            orientationCalibration      : this.orientationCalibration,

            isBaseOrientationInit       : this.isBaseOrientationInit,
            isBaseOrientationCalibrated : this.isBaseOrientationCalibrated,
            baseOrientation             : this.baseOrientation,
            baseTemperature             : this.baseTemperature,
            baseOrientationCalibration  : this.baseOrientationCalibration,

            isGpsInit                   : this.isGpsInit,
            gpsCoords                   : this.gpsCoords,

            isMagInit                   : this.isMagInit,
            magHeading                  : this.magHeading,

            ipAddress                   : this.netInfo.ip,
            declinationAngle            : this.declinationAngle,
            declinationSource           : this.declinationSource
        }
    }

    async open() {
        return new Promise((resolve, reject) => {
            try {
                this.initGpio().then(() => {
                    this.httpServer = this.app.listen(this.opts.port, () => {
                        this.log('Listening on', this.httpServer.address())
                        this.localUrl = 'http://localhost:' + this.httpServer.address().port
                        this.miscInterval = setInterval(() => this.miscLoop(), 1000)
                        this.openGauger().then(resolve).catch(reject)
                    })
                }).catch(reject)
            } catch (err) {
                reject(err)
            }
        })
    }

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
                        this.gauger.flush()
                        this.initGaugerWorker()
                        this.gaugerParser.on('data', data => {
                            try {
                                data = data.trim().replace(/^[^a-zA-Z0-9=]+/, '')
                                if (!data.length) {
                                    return
                                }
                                if (data.indexOf('ACK:') == 0) {
                                    this.handleGaugerAckData(data)
                                } else {
                                    this.handleGaugeData(data)
                                }
                            } catch (err) {
                                this.error('Exception while handling response data', err)
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
        if (this.gaugerJobs[id]) {
            this.log('Gauger ACK job', {id, resText})
            try {
                const status = parseInt(resText.substring(1, 3))
                var res = {
                    status,
                    message : DeviceCodes[status],
                    body    : resText.substring(4),
                    raw     : resText
                }
            } catch (error) {
                var res = {error}
            }
            this.gaugerJobs[id].handler(res)
        } else {
            this.log('Unknown gauger job ACKd', {id, resText})
        }
    }

    handleGaugeData(data) {
        const [module, text] = data.split(':')
        const values = (text || '').split('|')
        const floats = Util.floats(values)
        switch (module) {
            case 'GPS':
                this.gpsCoords = floats.map(v => v == DEG_NULL ? null : v)
                break
            case 'MAG':
                this.magHeading = floats[0] == DEG_NULL ? null : floats[0]
                break
            case 'ORI':
                // x|y|z|qw|qx|qy|qz|temp|cal_system|cal_gyro|cal_accel|cal_mag|isCalibrated|isInit
                this.orientation = floats.slice(0, 7).map(v => v == DEG_NULL ? null : v)
                this.temperature = floats[7]
                this.orientationCalibration = floats.slice(8, 12)
                this.isOrientationCalibrated = values[12] == 'T'
                break
            case 'ORF':
                this.baseOrientation = floats.slice(0, 7).map(v => v == DEG_NULL ? null : v)
                this.baseTemperature = floats[7]
                this.baseOrientationCalibration = floats.slice(8, 12)
                this.isBaseOrientationCalibrated = values[12] == 'T'
                break
            case 'MCC':
                // motor controller status
                // only do position from here is we do not have I2C status
                if (!this.isMciInit) {
                    this.position = [
                        floats[0] == DEG_NULL ? null : floats[0],
                        floats[1] == DEG_NULL ? null : floats[1]
                    ]
                }
                this.limitsEnabled = [
                    values[2] == 'T',
                    values[3] == 'T'
                ]
                this.maxSpeeds = floats.slice(6, 8)
                this.accelerations = floats.slice(8, 10)
                if (values[10]) {
                    // possible to get TypeError for 
                    this.limitStates = values[10].split('').map(it => it == 'T')
                }
                break
            case 'MCI':
                this.position = [
                    floats[0] == DEG_NULL ? null : floats[0],
                    floats[1] == DEG_NULL ? null : floats[1]
                ]
                break
            case 'MOD':
                // names the modules available
                // TODO: make efficient
                this.isOrientationInit = values.indexOf('ORI') > -1
                this.isBaseOrientationInit = values.indexOf('ORF') > -1
                this.isMcInit = values.indexOf('MCC') > -1
                this.isGpsInit = values.indexOf('GPS') > -1
                this.isMagInit = values.indexOf('MAG') > -1
                this.isMciInit = values.indexOf('MCI') > -1
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
        
        this.clearGauges()
        this.stopGaugerWorker()
    }

    drainGaugerQueue() {
        this.gaugerJobs = {}
        this.gaugerQueue = []
    }

    close() {
        return new Promise(resolve => {
            this.log('Shutting down')
            clearInterval(this.miscInterval)
            this.closeGauger()
            if (this.gpio) {
                this.gpio.close()
            }
            if (this.httpServer) {
                this.httpServer.close()
            }
            resolve()
        })
    }

    gaugerLoop() {
        if (this.gaugerBusy) {
            return
        }

        if (!this.gaugerQueue.length) {
            return   
        }

        this.gaugerBusy = true

        const {id, body, handler} = this.gaugerQueue.pop()
        // TODO: garbage collect unacked jobs
        this.gaugerJobs[id] = {
            handler: res => {
                this.gaugerBusy = false
                if (handler) {
                    handler(res)
                }
            }
        }
        // TODO: try catch and reject
        this.gauger.write(Buffer.from(this.opts.mock ? body : body.trim()))
    }

    async miscLoop() {
        if (this.miscBusy) {
            return
        }
        this.miscBusy = true
        try {

            // TODO
            if (false) {
                await this.refreshDeclinationAngle()
            }

            await this.refreshNetInfo()
        } catch (err) {
            this.error(err)
        } finally {
            this.miscBusy = false
        }
    }

    gaugerCommand(body, params = {}) {
        const id = this._newGaugerJobId()
        body = ':' + id + body
        return new Promise((resolve, reject) => {
            this.log('Enqueuing gauger command', {id, body: body.trim()})
            this.gaugerQueue.unshift({isSystem: false, ...params, body, id, handler: resolve})
        })
    }

    _newGaugerJobId() {
        if (!this._gid || this._gid > 2 * 1000 * 1000 * 1000) {
            this._gid = 0
        }
        return ++this._gid
    }

    initGaugerWorker() {
        this.log('Initializing gauger worker to run every', this.opts.workerDelay, 'ms')
        this.stopGaugerWorker()
        this.gaugerWorkerHandle = setInterval(() => this.gaugerLoop(), this.opts.workerDelay)
    }

    stopGaugerWorker() {
        clearInterval(this.gaugerWorkerHandle)
        this.gaugerBusy = false
    }

    initApp(app) {

        app.set('view engine', 'ejs')
        app.set('views', __dirname + '/views')

        app.use('/static', express.static(__dirname + '/static'))

        app.get('/', (req, res) => {
            this.status().then(status => {
                res.render('index', {
                    title: 'MoonUnit',
                    helper: this.templateHelper,
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

        app.post('/controller/gpio/reset', (req, res) => {
            if (!this.opts.gpioEnabled) {
                res.status(400).json({error: 'gpio not enabled'})
                return
            }
            this.log('Sending reset to controller')
            this.gpio.resetController().then(() => {
                res.status(200).json({message: 'controller reset sent'})
                this.log('Controller reset sent')
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

        app.post('/gauger/gpio/reset', (req, res) => {
            if (!this.opts.gpioEnabled) {
                res.status(400).json({error: 'gpio not enabled'})
                return
            }
            this.closeGauger()
            this.log('Sending reset to gauger')
            this.gpio.resetGauger().then(() => {
                res.status(200).json({message: 'gauger reset'})
                this.log('Gauger reset sent, delaying', this.opts.resetDelay, 'to reopen')
                this.openGauger().catch(err => this.error(err))
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

    async refreshDeclinationAngle() {
        // TODO
    }

    async refreshNetInfo() {
        const interfaces = os.networkInterfaces()
        var ifaceName
        if (this.opts.netInfoIface) {
            ifaceName = this.opts.netInfoIface
        } else {
            // if no iface specified, find first non-local iface
            for (var [name, nets] of Object.entries(interfaces)) {
                for (var net of nets) {
                    if (!net.internal) {
                        ifaceName = name
                        break
                    }
                }
                
            }
        }
        if (!interfaces[ifaceName]) {
            return
        }
        for (var net of interfaces[ifaceName]) {
            if (net.family == 'IPv4') {
                this.netInfo.ip = net.address
                return
            }
        }
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

    async initGpio() {
        this.log('GPIO is', this.opts.gpioEnabled ? 'enabled' : 'disabled')
        this.gpio = new GpioHelper(this)
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

class TemplateHelper {
    fixedSafe(val, n) {
        if (typeof val == 'number' && !isNaN(val)) {
            return val.toFixed(n)
        }
        return '' + val
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
