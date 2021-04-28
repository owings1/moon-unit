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
        new App({controllerPath: '/tmp/test-instantiate', gaugerPath: '/tmp/test-instantiate-1'})
    })

    describe('server', () => {

        var app
        var appUrl

        beforeEach(async () => {
            app = new App({
                gaugerPath: '/tmp/test1-' + +new Date,
                port: null,
                openDelay: 1,
                quiet: true,
                mock: true
            })
            await app.open()
            appUrl = 'http://localhost:' + app.httpServer.address().port
        })

        afterEach(async () => {
            await app.close()
        })

        it('should serve metrics', async () => {
            const res = await fetch(appUrl + '/metrics')
            expect(res.status).to.equal(200)
        })

        // we have to rethink how to mock the response since now it is waiting for an ACK
        it.skip('should send sync command', async () => {
            const res = await fetch(appUrl + '/controller/command/sync', {
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

const Util = require('../src/util')
describe('Util', () => {
    describe('calcDefaultDeclinationRad', () => {
        it('should return degrees between +/-23.45', () => {
            const rad = Util.calcDefaultDeclinationRad()
            const deg = Util.degrees(rad)
            expect(deg).to.be.greaterThan(-23.45)
            expect(deg).to.be.lessThan(23.45)
        })
    })
})