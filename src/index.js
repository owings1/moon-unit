console.log('hello world')

const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')

const mc = new SerialPort(process.env.MC_SERIAL_PORT, {
    baudRate: 115200,
    autoOpen: false
})

const parser_mc = mc.pipe(new Readline)

mc.open(err => {
    if (err) {
        console.error(err)
        return
    }
    console.log('Opened, delaying 2 seconds')
    setTimeout(() => {
        console.log('Writing command')
        parser_mc.once('data', resText => {
            console.log('Received response:', resText)
        })
        mc.write(':01 1 1 1600;', () => {
            setTimeout(() => mc.close(), 2000)
            //mc.close()
        })
        
    }, 2000)
})
