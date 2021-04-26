
const i2c = require('i2c-bus')

const ADDR = 0x8

// writing 0x0 means clear after send
//const wbuf = Buffer.from([8, 8, 8, 8, 8, 8, 8])
const wbuf = Buffer.from([0x0])
const rbuf = Buffer.alloc(1)
//const rbuf = Buffer.alloc(16)


async function main() {
    const conn = await i2c.openPromisified(1)
    try {
        // TODO: handle undefined
        // TODO: handle EREMOTEIO error
        const initialValues = await readValues(conn)
        //const startPos = initialValues.change
        //const startPos = initialValues.pos
        console.log({initialValues})
        const startPos = 0
        var pos = startPos
        while (true) {
            var values = await readValues(conn)
            console.log(values)
            //if (!values) {
            //    continue
            //}
            if (values.change) {
                pos += values.change
                console.log({pos, change: values.change})
            }
            /*
            if (values.pos != pos) {
                pos = values.pos
                console.log({pos})
            }
            */
            if (values.isPressed) {
                console.log('pressed')
                break
            }
            await new Promise(resolve => setTimeout(resolve, 100))
        }
    } finally {
        conn.close()
    }
}

async function readValues(conn) {
    await conn.i2cWrite(ADDR, wbuf.length, wbuf)
    const data = await conn.i2cRead(ADDR, rbuf.length, rbuf)
    return getValues(data)
}

function getValues(data) {
    const byte = data.buffer[0]
    // first (MSB) bit is push button
    const isPressed = (byte & 128) == 128
    // second bit is positive=1 negative=0
    const sign = (byte & 64) == 64 ? 1 : -1
    // last six bits are the amount
    var qty = byte & ~192
    // ignore noise, TODO figure out why this is happening occasionally
    if (qty > 12) {
        qty = 0
    }
    //console.log({byte, isPressed, sign, qty, buf: data.buffer.toJSON()})
    console.log({byte})
    return {
        isPressed,
        change: qty * sign
    }
}


main()

/*
function getValuesOld(data) {
    const resStr = data.buffer.toString().split('\n')[0]
    if (resStr[0] == '^') {
        const parts = resStr.substring(1).split('|')
        return {
            pos : +parts[0],
            isPressed: !!+parts[1]
        }
    }
}
*/
/*
i2c.openPromisified(1)
    .then(conn => {
        conn.i2cWrite(ADDR, wbuf.length, wbuf)
         .then(() => conn.i2cRead(ADDR, rbuf.length, rbuf))
         .then(data => console.log(data.buffer.toString().split('\n')[0]))
         .then(() => conn.close())
     })
    .catch(console.error)
*/
