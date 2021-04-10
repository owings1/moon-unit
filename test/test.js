const {expect} = require('chai')
const fetch = require('node-fetch')
const fs = require('fs')
const fse = require('fs-extra')
const merge = require('merge')
const path = require('path')
const {resolve} = path

process.env.MOCK = '1'

const DeviceService = require('../src/device')
describe('DeviceService', () => {
    it('should instantiate', () => {
        new DeviceService({path: '/tmp/test-instantiate'})
    })
})