

const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')

const device = new SerialPort(process.env.MC_SERIAL_PORT, {
    baudRate: 115200,
    autoOpen: false
})

const parser = device.pipe(new Readline)
const openDelay = 2000
const workerDelay = 100

const queue = []

var isBusy = false

const workerHandle = setInterval(() => {

    if (isBusy || !queue.length) {
        return
    }

    isBusy = true

    job = queue.pop()

    parser.once('data', resText => {
        // handle device response
        console.log('Receieved response:', resText)
        job.handler({
            status: parseInt(resText.substring(1))
        })
        isBusy = false
    })

    // send command to device
    device.write(job.body)

}, workerDelay)

function enqueue(body, handler) {
    console.log('Enqueuing command', body)
    queue.unshift({body, handler})
}

device.open(err => {
    if (err) {
        console.error(err)
        return
    }
    console.log('Opened, delaying', openDelay, 'ms')
    setTimeout(() => {
        enqueue(':01 1 1 1600;', res => {
            console.log(1, res)
        })
        enqueue(':01 1 2 1600;', res => {
            console.log(2, res)
        })
        enqueue(':01 1 1 1600;', res => {
            console.log(3, res)
        })
        enqueue(':01 1 2 1600;', res => {
            console.log(4, res)
            setTimeout(() => {
                console.log('Closing')
                clearInterval(workerHandle)
                device.close()
            }, 2000)
        })
    }, openDelay)
})
