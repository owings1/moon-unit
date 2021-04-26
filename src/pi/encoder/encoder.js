
const i2c = require('i2c-bus')

const ADDR = 0x8

const wbuf = Buffer.from([0])
const rbuf = Buffer.alloc(16)


async function main() {
    const conn = await i2c.openPromisified(1)
    try {
        // TODO: handle undefined
        // TODO: handle EREMOTEIO error
        const initialValues = await readValues(conn)
        const startPos = initialValues.pos
        console.log({startPos})
        var pos = startPos
        while (true) {
            var values = await readValues(conn)
            if (!values) {
                continue
            }
            if (values.pos != pos) {
                pos = values.pos
                console.log({pos})
            }
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
    const resStr = data.buffer.toString().split('\n')[0]
    if (resStr[0] == '^') {
        const parts = resStr.substring(1).split('|')
        return {
            pos : +parts[0],
            isPressed: !!+parts[1]
        }
    }
}

main()
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
