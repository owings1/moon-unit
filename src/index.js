

const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')
const merge = require('merge')

class SerialDevice {

    defaults() {
        return {
            baudRate: 115200,
            autoOpen: false,
            openDelay: 2000,
            workerDelay: 100
        }
    }

    constructor(path, opts) {
        this.opts = merge(this.defaults(), opts)
        this.device = new SerialPort(path, this.opts)
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
                console.log('Opened, delaying', this.opts.openDelay, 'ms')
                setTimeout(resolve, this.opts.openDelay)
            })
        })
    }

    close() {
        return new Promise(resolve => {
            console.log('Closing')
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
}

const bodyParser = require('body-parser')

class Server {

    defaults() {
        return {
            
        }
    }

    constructor(mc, opts) {
        this.mc = mc
        this.opts = merge(this.defaults(), opts)
        this.app = express()
        this.initApp(app)
        this.httpServer = null
        this.port = null
    }

    initApp(app) {

        app.post('/devices/motor-controller', bodyParser.json(), (req, res) => {
            if (!req.body.command) {
                res.status(400).json({error: 'missing command'})
                return
            }
            this.mc.request(req.body)
                .then(resp => res.status(200).json(resp))
                .catch(error => {
                    console.error(error)
                    res.status(500).json({error})
                })
        })

        app.use((req, res) => res.status(404).json({error: 'not found'}))
    }

    listen(port) {
        return new Promise((resolve, reject) => {
            try {
                this.httpServer = this.app.listen(port, () => {
                    this.port = this.httpServer.address().port
                    console.log('Listening on port', this.port)
                    resolve()
                })
            } catch (err) {
                reject(err)
            }
        })
        
    }

    close() {
        if (this.httpServer) {
            this.httpServer.close()
        }
    }
}

const mc = new SerialDevice(process.env.MC_SERIAL_PORT)

mc.open().then(() => {

    mc.request(':01 1 1 1600;').then(res => {
        console.log(1, res)
    })

    const server = new Server(mc)
    server.listen(process.env.HTTP_LISTEN_PORT || '8080')
    //device.request(':01 1 2 1600;').then(res => {
    //    console.log(2, res)
    //})
    //
    //device.request(':01 1 1 1600;').then(res => {
    //    console.log(3, res)
    //})
    //
    //device.request(':01 1 2 1600;').then(res => {
    //    console.log(4, res)
    //    device.close()
    //})
})

