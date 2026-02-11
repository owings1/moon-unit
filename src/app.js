// Serial device command HTTP service

// TODO: garbage collect unacked gauger jobs

import express from 'express'
import Util from './util.js'
import bodyParser from 'body-parser'

import SerialPort from 'serialport'
import Readline from '@serialport/parser-readline'

const DEG_NULL = 1000
const POS_NULL = 10_000_000
const MOD_KNOWN = new Set(['MCI', 'ORI', 'ORF', 'GPS', 'MAG'])

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
            devicePath: env.GAUGER_PORT,
            deviceBaudRate: +env.GAUGER_BAUD_RATE || 115200,
            port: env.HTTP_PORT || 8080,
            quiet: !!env.QUIET,
            openDelay: +env.OPEN_DELAY || 1_000,
            commandWorkerDelay: +env.WORKER_DELAY || 100,
            miscWorkerDelay: +env.MISC_DELAY || 10_000,
            commandTimeout: +env.COMMAND_TIMEOUT || 5_000,
        }
    }

    constructor(opts, env) {
        this.opts = Object.assign(this.defaults(env), opts || {})
        this.commandJobs = {}
        this.commandQueue = []
        this.commandWorkerBusy = false
        this.commandWorkerHandle = null
        this.isDeviceConnected = false
        this.shouldDeviceAutoconnect = true
        this.app = express()
        this.httpServer = null
        this.clearGauges()
        // this.templateHelper = new TemplateHelper
        this.initApp(this.app)
        this.declinationData = {}
        this.declinationAngle = null
        this.declinationSource = null
        this.device = new SerialPort(this.opts.devicePath, { baudRate: this.opts.deviceBaudRate, autoOpen: false })
    }

    clearGauges() {
        this.modarr = []
        this.modmap = new Map

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
            isDeviceConnected: this.isDeviceConnected,
            mod: this.modarr,

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
                    this.miscInterval = setInterval(() => this.miscLoop(), this.opts.miscWorkerDelay)
                    this.openDevice().then(resolve).catch(err => {
                        this.error(err)
                        resolve()
                    })
                })
            } catch (err) {
                reject(err)
            }
        })
    }

    async openDevice() {
        this.closeDevice()
        this.log('Opening device', this.opts.devicePath)
        await new Promise((resolve, reject) => {
            this.device.open(err => {
                if (err) {
                    this.debug('Failed to open device')
                    reject(err)
                    return
                }
                this.isDeviceConnected = true
                this.shouldDeviceAutoconnect = true
                this.log('Device opened, delaying', this.opts.openDelay, 'ms')
                this.deviceParser = this.device.pipe(new Readline)
                setTimeout(resolve, this.opts.openDelay)
            })
        })
        this.device.flush()
        this.initCommandWorker()
        this.deviceParser.on('data', data => {
            try {
                data = data.trim().replace(/^[^a-zA-Z0-9=]+/, '')
                if (!data.length) {
                    return
                }
                if (data.indexOf('ACK:') == 0) {
                    this.handleCommandAckData(data)
                } else {
                    this.handleDeviceStreamData(data)
                }
            } catch (err) {
                this.error('Exception while handling response data', err)
            }

        })
        this.log('Setting device to streaming mode')
        this.enqueueCommand(':71 2;\n').then(res => {
            if (res.status != 0) {
                this.error('Failed to set gauger to streaming mode', res)
                return
            }
            this.log('Device acknowledges streaming mode')
        })
    }

    handleCommandAckData(data) {
        const [ack, id, resText] = data.split(':')
        if (this.commandJobs[id]) {
            this.log('Command ACK job', { id, resText })
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
            this.commandJobs[id].done(res)
        } else {
            this.log('Unknown gauger job ACKd', { id, resText })
        }
    }

    handleDeviceStreamData(data) {
        const [module, text] = String(data).split(':')
        const now = new Date
        if (this.modmap.has(module)) {
            const mod = this.modmap.get(module)
            if (mod.raw !== text) {
                mod.raw = text
                mod.updatedAt = now
            }
            mod.receivedAt = now
        }
        const values = (text || '').split('|')
        const floats = Util.parseFloats(values)
        switch (module) {
            case 'GPS':
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
                break
            case 'MOD':
                // names the modules available
                for (const label of values) {
                    if (MOD_KNOWN.has(label) && !this.modmap.has(label)) {
                        const obj = {label}
                        this.modarr.push(obj)
                        this.modmap.set(label, obj)
                    }
                }
                const modSet = new Set(values)
                this.isOrientationInit = modSet.has('ORI')
                this.isBaseOrientationInit = modSet.has('ORF')
                this.isMagInit = modSet.has('MAG')
                break
            default:
                this.log('Unknown module', module)
                break
        }
    }

    closeDevice() {
        if (this.device) {
            if (this.device.isOpen) {
                this.log('Closing gauger')
                this.device.close()
                // this.gauger = null
            } else {
                this.log('Gauger not open')
            }
        }
        this.isDeviceConnected = false
        this.drainCommandQueue()
        this.clearGauges()
        this.stopCommandWorker()
    }

    drainCommandQueue() {
        this.commandJobs = {}
        this.commandQueue = []
    }

    close() {
        return new Promise(resolve => {
            this.log('Shutting down')
            clearInterval(this.miscInterval)
            this.closeDevice()
            if (this.httpServer) {
                this.httpServer.close()
            }
            resolve()
        })
    }

    workerLoop() {
        if (this.commandWorkerBusy) {
            return
        }

        if (!this.commandQueue.length) {
            return
        }

        this.commandWorkerBusy = true

        const { id, body, handler } = this.commandQueue.pop()
        const timeoutId = setTimeout(
            () => this.commandJobs[id]?.done({error: 'Command timeout'}),
            this.opts.commandTimeout,
        )
        this.commandJobs[id] = {
            done: res => {
                clearTimeout(timeoutId)
                this.commandWorkerBusy = false
                delete this.commandJobs[id]
                if (handler) {
                    handler(res)
                }
            }
        }
        // TODO: try catch and reject
        this.device.write(Buffer.from(body.trim()))
    }

    async miscLoop() {
        // return
        if (this.miscBusy) {
            return
        }
        this.miscBusy = true
        this.isDeviceConnected &= Boolean(this.device?.isOpen)
        try {
            if (!this.isDeviceConnected && this.shouldDeviceAutoconnect) {
                await this.openDevice()
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

    enqueueCommand(body, params = {}) {
        const id = this._newCommandId()
        body = ':' + id + body
        return new Promise((resolve, reject) => {
            this.log('Enqueuing command', { id, body: body.trim() })
            this.commandQueue.unshift({ isSystem: false, ...params, body, id, handler: resolve })
        })
    }

    _newCommandId() {
        if (!this._gid || this._gid > 2 * 1000 * 1000 * 1000) {
            this._gid = 0
        }
        return ++this._gid
    }

    initCommandWorker() {
        this.log('Initializing command worker to run every', this.opts.commandWorkerDelay, 'ms')
        this.stopCommandWorker()
        this.commandWorkerHandle = setInterval(() => this.workerLoop(), this.opts.commandWorkerDelay)
    }

    stopCommandWorker() {
        clearInterval(this.commandWorkerHandle)
        this.commandWorkerBusy = false
    }

    initApp(app) {

        app.set('view engine', 'ejs')
        app.set('views', import.meta.dirname + '/views')

        app.use('/static', express.static(import.meta.dirname + '/static'))

        app.get('/', (req, res) => {
            this.status().then(status => {
                res.render('index', {
                    title: 'MoonUnit',
                    // helper: this.templateHelper,
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
                this.enqueueCommand(req.body.command)
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
            this.closeDevice()
            this.shouldDeviceAutoconnect = false
            this.status().then(status => {
                res.status(200).json({ message: 'Device disconnected', status })
            })
        })

        app.post('/connect', (req, res) => {
            if (this.isDeviceConnected) {
                res.status(400).json({ message: 'Device already connected' })
                return
            }
            this.openDevice().then(() => {
                this.status().then(status => {
                    res.status(200).json({ message: 'Device connected', status })
                })
            }).catch(error => {
                res.status(500).json({ error })
            })
        })
        app.use((req, res) => res.status(404).json({ error: 'not found' }))
    }

    async refreshDeclinationAngle() {
        // TODO
    }

    // createDevice(devicePath, baudRate) {
    //     return new SerialPort(devicePath, { baudRate, autoOpen: false })
    // }

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

// class TemplateHelper {
//     fixedSafe(val, n) {
//         if (typeof val == 'number' && !isNaN(val)) {
//             return val.toFixed(n)
//         }
//         return '' + val
//     }
//     connectedStr(val) {
//         return val ? 'Connected' : 'Disconnected'
//     }
//     mcBusyStr(val) {
//         if (val == null) {
//             return '?'
//         }
//         return val ? 'Busy' : 'Ready'
//     }
// }

export default App
