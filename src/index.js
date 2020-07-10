console.log('hello world')

const SerialPort = require('serialport')

var mc = new SerialPort(process.env.MC_SERIAL_PORT, {
    baudRate: 115200,
    autoOpen: false
})

mc.open(err => {
    if (err) {
        console.error(err)
        return
    }
    console.log('Opened, delaying 2 seconds')
    setTimeout(() => {
        mc.write(':01 1 1 1600;')
        mc.close()
    }, 2000)
})
