// Serial device command HTTP service

// TODO: garbage collect unacked gauger jobs

const fs = require('fs')
const express = require('express')
const merge = require('merge')
const path = require('path')
const showdown = require('showdown')
const Util = require('./util')
const bodyParser = require('body-parser')

const MockBinding = require('@serialport/binding-mock')
const SerPortFull = require('serialport')
const SerPortMock = require('@serialport/stream')
const Readline = require('@serialport/parser-readline')

const DEG_NULL = 1000


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

class App {

    defaults(env) {
        env = env || process.env
        return {
            gaugerPath: env.GAUGER_PORT,
            gaugerBaudRate: +env.GAUGER_BAUD_RATE || 115200,
            mock: !!env.MOCK,
            port: env.HTTP_PORT || 8080,
            quiet: !!env.QUIET,
            openDelay: +env.OPEN_DELAY || 1_000,
            workerDelay: +env.WORKER_DELAY || 100,
            miscDelay: +env.MISC_DELAY || 10000,
            commandTimeout: +env.COMMAND_TIMEOUT || 5_000,
        }
    }

    constructor(opts, env) {

        this.opts = merge(this.defaults(env), opts)

        this.gaugerJobs = {}
        this.gaugerQueue = []
        this.gaugerBusy = false
        this.gaugerWorkerHandle = null
        this.isGaugerConnected = false
        this.shouldGaugerAutoconnect = true

        this.app = express()
        this.httpServer = null

        this.clearGauges()
        this.templateHelper = new TemplateHelper
        this.initApp(this.app)

        this.declinationData = {}
        this.declinationAngle = null
        this.declinationSource = null
    }

    clearGauges() {

        this.mcBusy = null
        this.isMciInit = null
        this.position = [null, null]
        this.limitsEnabled = [null, null]
        this.limitStates = [null, null, null, null]
        this.maxSpeeds = [null, null]
        this.accelerations = [null, null]

        this.isGpsInit = null
        this.gpsCoords = [null, null]

        this.isMagInit = null
        this.magHeading = null

        this.declinationAngle = null
        this.declinationSource = null

        this.isOrientationInit = null
        this.orientation = [null, null, null, null, null, null, null]
        this.temperature = null
        this.orientationCalibration = [null, null, null, null]
        this.isOrientationCalibrated = null

        this.isBaseOrientationInit = null
        this.baseOrientation = [null, null, null, null, null, null, null]
        this.baseTemperature = null
        this.baseOrientationCalibration = [null, null, null, null]
        this.isBaseOrientationCalibrated = null

    }

    async status() {
        return {
            isGaugerConnected: this.isGaugerConnected,

            mcBusy: this.mcBusy,
            isMciInit: this.isMciInit,
            position: this.position,
            limitsEnabled: this.limitsEnabled,
            limitStates: this.limitStates,
            maxSpeeds: this.maxSpeeds,
            accelerations: this.accelerations,

            isOrientationInit: this.isOrientationInit,
            isOrientationCalibrated: this.isOrientationCalibrated,
            orientation: this.orientation,
            temperature: this.temperature,
            orientationCalibration: this.orientationCalibration,

            isBaseOrientationInit: this.isBaseOrientationInit,
            isBaseOrientationCalibrated: this.isBaseOrientationCalibrated,
            baseOrientation: this.baseOrientation,
            baseTemperature: this.baseTemperature,
            baseOrientationCalibration: this.baseOrientationCalibration,

            isGpsInit: this.isGpsInit,
            gpsCoords: this.gpsCoords,

            isMagInit: this.isMagInit,
            magHeading: this.magHeading,

            declinationAngle: this.declinationAngle,
            declinationSource: this.declinationSource,
        }
    }

    async open() {
        return new Promise((resolve, reject) => {
            try {
                this.httpServer = this.app.listen(this.opts.port, () => {
                    this.log('Listening on', this.httpServer.address())
                    this.localUrl = 'http://localhost:' + this.httpServer.address().port
                    this.miscInterval = setInterval(() => this.miscLoop(), this.opts.miscDelay)
                    this.openGauger().then(resolve).catch(err => {
                        this.error(err)
                        resolve()
                    })
                })
            } catch (err) {
                reject(err)
            }
        })
    }

    async openGauger() {
        this.closeGauger()
        this.log('Opening gauger', this.opts.gaugerPath)
        this.gauger = this.createDevice(this.opts.gaugerPath, this.opts.gaugerBaudRate)
        this.debug(`Created device`)
        await new Promise((resolve, reject) => {
            this.gauger.open(err => {
                if (err) {
                    this.debug('Failed to open gauger')
                    reject(err)
                    return
                }
                this.isGaugerConnected = true
                this.shouldGaugerAutoconnect = true
                this.log('Gauger opened, delaying', this.opts.openDelay, 'ms')
                this.gaugerParser = this.gauger.pipe(new Readline)
                setTimeout(resolve, this.opts.openDelay)
            })
        })
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
    }

    handleGaugerAckData(data) {
        const [ack, id, resText] = data.split(':')
        if (this.gaugerJobs[id]) {
            this.log('Gauger ACK job', { id, resText })
            try {
                const status = parseInt(resText.substring(1, 3))
                var res = {
                    status,
                    message: DeviceCodes[status],
                    body: resText.substring(4),
                    raw: resText
                }
            } catch (error) {
                var res = { error }
            }
            this.gaugerJobs[id].handler(res)
        } else {
            this.log('Unknown gauger job ACKd', { id, resText })
        }
    }

    handleGaugeData(data) {
        const [module, text] = String(data).split(':')
        const values = (text || '').split('|')
        const floats = Util.floats(values)
        switch (module) {
            case 'GPS':
                this.gpsCoords = floats.map(v => v === DEG_NULL ? null : v)
                break
            case 'MAG':
                this.magHeading = floats[0] === DEG_NULL ? null : floats[0]
                break
            case 'ORI':
                // x|y|z|qw|qx|qy|qz|temp|cal_system|cal_gyro|cal_accel|cal_mag|isCalibrated|isInit
                this.orientation = floats.slice(0, 7).map(v => v === DEG_NULL ? null : v)
                this.temperature = floats[7]
                this.orientationCalibration = floats.slice(8, 12)
                this.isOrientationCalibrated = values[12] === 'T'
                break
            case 'ORF':
                this.baseOrientation = floats.slice(0, 7).map(v => v === DEG_NULL ? null : v)
                this.baseTemperature = floats[7]
                this.baseOrientationCalibration = floats.slice(8, 12)
                this.isBaseOrientationCalibrated = values[12] === 'T'
                break
            case 'MCI':
                let i = 0
                const statusFlag = parseInt(values[i], 0x10) || 0
                const checkBit = bit => Util.flagBitSet(bit, statusFlag)
                this.mcBusy = checkBit(0)
                this.limitStates = [2, 4, 3, 5].map(checkBit)
                this.limitsEnabled = [12, 13].map(checkBit)
                this.position = [
                    floats[++i] === DEG_NULL ? null : floats[0],
                    floats[++i] === DEG_NULL ? null : floats[1]
                ]
                this.maxSpeeds = [
                    parseInt(values[++i], 0x10) || null,
                    parseInt(values[++i], 0x10) || null,
                ]
                this.accelerations = [
                    parseInt(values[++i], 0x10) || null,
                    parseInt(values[++i], 0x10) || null,
                ]
                break
            case 'MOD':
                // names the modules available
                const modSet = new Set(values)
                this.isOrientationInit = modSet.has('ORI')
                this.isBaseOrientationInit = modSet.has('ORF')
                this.isGpsInit = modSet.has('GPS')
                this.isMagInit = modSet.has('MAG')
                this.isMciInit = modSet.has('MCI')
                break
            default:
                this.log('Unknown module', module)
                break
        }
    }

    closeGauger() {
        if (this.gauger) {
            if (this.gauger.isOpen) {
                this.log('Closing gauger')
                this.gauger.close()
                this.gauger = null
            } else {
                this.log('Gauger not open')
            }
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

        const { id, body, handler } = this.gaugerQueue.pop()
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
        // return
        if (this.miscBusy) {
            return
        }
        this.miscBusy = true
        this.isGaugerConnected &= Boolean(this.gauger?.isOpen)
        try {
            if (!this.isGaugerConnected && this.shouldGaugerAutoconnect) {
                await this.openGauger()
            }
            // TODO
            if (false) {
                await this.refreshDeclinationAngle()
            }

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
            this.log('Enqueuing gauger command', { id, body: body.trim() })
            this.gaugerQueue.unshift({ isSystem: false, ...params, body, id, handler: resolve })
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
            this.status().then(status => res.status(200).json({ status }))
        })

        app.post('/command', bodyParser.json(), (req, res) => {
            if (!req.body.command) {
                res.status(400).json({ error: 'missing command' })
                return
            }
            try {
                this.gaugerCommand(req.body.command)
                    .then(response => res.status(200).json({ response }))
                    .catch(error => {
                        this.error(error)
                        res.status(500).json({ error })
                    })
            } catch (error) {
                this.error(error)
                res.status(500).json({ error })
            }
        })

        app.post('/disconnect', (req, res) => {
            this.closeGauger()
            this.shouldGaugerAutoconnect = false
            this.status().then(status => {
                res.status(200).json({ message: 'Device disconnected', status })
            })
        })

        app.post('/connect', (req, res) => {
            if (this.isGaugerConnected) {
                res.status(400).json({ message: 'Device already connected' })
                return
            }
            this.openGauger().then(() => {
                this.status().then(status => {
                    res.status(200).json({ message: 'Device connected', status })
                })
            }).catch(error => {
                res.status(500).json({ error })
            })
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
                    res.json({ error })
                    return
                }
                const converter = new showdown.Converter({
                    tables: true
                })
                const html = converter.makeHtml(text)
                res.render('doc', { html })
            })
        })

        app.use((req, res) => res.status(404).json({ error: 'not found' }))
    }

    async refreshDeclinationAngle() {
        // TODO
    }

    createDevice(devicePath, baudRate) {
        var SerialPort = SerPortFull
        if (this.opts.mock) {
            SerPortMock.Binding = MockBinding
            var SerialPort = SerPortMock
            // TODO: mock response
            //  see: https://serialport.io/docs/api-binding-mock
            //  see: https://github.com/serialport/node-serialport/blob/master/packages/binding-mock/lib/index.js
            MockBinding.createPort(devicePath, { echo: true, readyData: [] })
        }
        return new SerialPort(devicePath, { baudRate, autoOpen: false })
    }

    log(...args) {
        if (!this.opts.quiet) {
            console.log(new Date, ...args)
        }
    }

    debug(...args) {
        if (!this.opts.quiet) {
            console.debug(new Date, ...args)
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
    connectedStr(val) {
        return val ? 'Connected' : 'Disconnected'
    }
    mcBusyStr(val) {
        if (val == null) {
            return '?'
        }
        return val ? 'Busy' : 'Ready'
    }
}

module.exports = App
