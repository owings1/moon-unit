const child_process = require('child_process')

const NetTemplate = [
    '{',
    '    ssid="SSID"',
    '    psk="PSK"',
    '    priority=PRIORITY',
    '}'
].join('\n')

class WpaHelper {

    constructor(app) {
        this.app = app // tightly coupled !
        this.enabled = this.app.opts.wpaEnabled
    }

    // return currently connected info
    async fetchConnInfo() {
        /*

        # wpa_cli -i wlan0 status

        bssid=9c:3d:cf:cb:98:9c
        freq=2422
        ssid=caesarno
        id=0
        mode=station
        pairwise_cipher=CCMP
        group_cipher=CCMP
        key_mgmt=WPA2-PSK
        wpa_state=COMPLETED
        ip_address=192.168.34.29
        p2p_device_address=ba:27:eb:b1:84:ab
        address=b8:27:eb:b1:84:ab
        uuid=61fd5e62-3abe-5aca-9a5b-b79b38cc8410
        */
    }

    // scan for wifi ssids
    async scanSsids() {
        if (!this.enabled) {
            return []
        }
        const cmd = 'wpa_cli'
        var args = ['-i', this.app.opts.netInfoIface, 'scan']
        
        var result = await this._exec(cmd, args)
        if (result.status != 0) {
            this.error(result.stderr.toString('utf-8'))
            throw new Error(cmd + ' exited with status ' + result.status)
        }
        this.log(result)
        var args = ['-i', this.app.opts.netInfoIface, 'scan_results']
        var result = await this._exec(cmd, args)
        if (result.status != 0) {
            this.error(result.stderr.toString('utf-8'))
            throw new Error(cmd + ' exited with status ' + result.status)
        }
        return result.stdout.toString('utf-8')
                // get lines
                .split('\n')
                // remove header
                .slice(1)
                // split line on tab, get last or empty string
                .map(it => (it.split('\t')[4] || '').trim())
                // filter out empty and \\x00
                .filter(it => it.length && it.indexOf('\\x00') < 0)
    }

    // should return success true/false
    async connect(ssid, key) {
        
    }

    // add an ssid to config
    async addSsidInfo(ssid, key, priority) {
        
    }

    // signal to wpa_supplicant to reload config
    async reloadConfig() {
        
    }

    async _exec(cmd, args, opts) {

        const child = child_process.spawn(cmd, args)
        const promise = new Promise((resolve, reject) => {
            child.addListener('error', reject)
            child.addListener('exit', resolve)
        })

        var obuf = Buffer.alloc(0)
        var ebuf = Buffer.alloc(0)

        child.stdout.on('data', data => obuf = Buffer.concat([obuf, data]))

        child.stderr.on('data', data => ebuf = Buffer.concat([ebuf, data]))

        await promise

        return {
            status: child.exitCode,
            stdout : obuf,
            stderr : ebuf
        }
    }

    log(...args) {
        if (!this.app.opts.quiet) {
            console.log(new Date, ...args)
        }
    }

    error(...args) {
        console.error(new Date, ...args)
    }
}

module.exports = WpaHelper

// testing
async function main() {
    // mock app
    const wpa = new WpaHelper({opts: {wpaEnabled: true, netInfoIface: 'wlan0'}})
    const ssids = await wpa.scanSsids()
    console.log(ssids)
}

if (require.main === module) {
    main()
}
