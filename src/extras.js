            //controllerPath     : env.CONTROLLER_PORT,
            //controllerBaudRate : +env.CONTROLLER_BAUD_RATE || 9600, //115200,


            //isControllerConnected     : this.isControllerConnected,
            //controllerConnectedStatus : this.isControllerConnected ? 'Connected' : 'Disconnected',
/*
if (!this.opts.controllerPath) {
    throw new ConfigError('path not set, you can use CONTROLLER_PORT')
}

this.controllerQueue        = []
this.controllerBusy         = false
this.controllerWorkerHandle = null
this.isControllerConnected  = false
*/


/*
this.openController().then(() =>
    this.openGauger()
).then(resolve).catch(reject)
*/

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

/*
async flushController() {
    // TODO: figure out why device.flush does not return a promise
    // commenting out for debug (getting errors)
    //return this.controller.flush()
}
*/

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

/*

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
*/