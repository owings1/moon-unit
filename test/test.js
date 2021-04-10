const {expect} = require('chai')
const fetch = require('node-fetch')
const fs = require('fs')
const fse = require('fs-extra')
const merge = require('merge')
const path = require('path')
const {resolve} = path


const DeviceService = require('../src/device')

describe('DeviceService', () => {

    it('should instantiate', () => {
        new DeviceService({path: '/tmp/test-instantiate'})
    })

    describe('server', () => {

        var svc
        var svcUrl

        beforeEach(async () => {
            svc = new DeviceService({
                path: '/tmp/test-' + +new Date,
                port: null,
                openDelay: 1,
                quiet: true,
                mock: true
            })
            await svc.listen()
            svcUrl = 'http://localhost:' + svc.port
        })

        afterEach(async () => {
            await svc.close()
        })

        it('should serve metrics', async () => {
            const res = await fetch(svcUrl + '/metrics')
            expect(res.status).to.equal(200)
        })

        it('should send sync command', async () => {
            const res = await fetch(svcUrl + '/command/sync', {
                method: 'POST',
                body: JSON.stringify({command: ':01 2 1 1600;\n'}),
                headers: {'Content-Type': 'application/json'}
            })
            const body = await res.json()
            expect(res.status).to.equal(200)
            expect(body.response).to.have.key('status')
        })
    })
})