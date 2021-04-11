const {expect}  = require('chai')
const fetch     = require('node-fetch')
const fs        = require('fs')
const fse       = require('fs-extra')
const merge     = require('merge')
const path      = require('path')
const {resolve} = path


const App = require('../src/app')

describe('App', () => {

    it('should instantiate', () => {
        new App({path: '/tmp/test-instantiate'})
    })

    describe('server', () => {

        var app
        var appUrl

        beforeEach(async () => {
            app = new App({
                path: '/tmp/test-' + +new Date,
                port: null,
                openDelay: 1,
                quiet: true,
                mock: true
            })
            await app.listen()
            appUrl = 'http://localhost:' + app.port
        })

        afterEach(async () => {
            await app.close()
        })

        it('should serve metrics', async () => {
            const res = await fetch(appUrl + '/metrics')
            expect(res.status).to.equal(200)
        })

        it('should send sync command', async () => {
            const res = await fetch(appUrl + '/command/sync', {
                method: 'POST',
                body: JSON.stringify({command: ':01 2 1 1600;\n'}),
                headers: {'Content-Type': 'application/json'}
            })
            const body = await res.json()
            expect(res.status).to.equal(200)
            expect(Object.keys(body.response)).to.contain('status')
        })
    })
})