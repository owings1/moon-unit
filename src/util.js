const fetch = require('node-fetch')

const DeclinationServiceUrl = 'https://www.ngdc.noaa.gov/geomag-web/calculators/calculateDeclination'

class Util {

    static floats(arr) {
         // normalize NaN, undefined, etc. to null
        return JSON.parse(
            JSON.stringify(
                arr.map(parseFloat)
            )
        )
    }

    static radians(degrees) {
        return degrees * Math.PI / 180
    }

    static degrees(radians) {
        return radians * 180 / Math.PI
    }

    // adapted from: https://www.30secondsofcode.org/js/s/day-of-year
    // License: https://creativecommons.org/publicdomain/zero/1.0/ (Public Domain)
    static dayOfYear(dateRef) {
        dateRef = dateRef || new Date
        return Math.floor((dateRef - new Date(dateRef.getFullYear(), 0, 0)) / 1000 / 60 / 60 / 24)
    }

    // see: https://www.pveducation.org/pvcdrom/properties-of-sunlight/declination-angle
    static calcDefaultDeclinationRad(dateRef) {
        dateRef = dateRef || new Date
        return Util.radians(-23.45 * Math.cos(360/365 * (Util.dayOfYear(dateRef) + 10)))
    }

    /*
    Request URL: https://www.ngdc.noaa.gov/geomag-web/calculators/calculateDeclination?browserRequest=true&magneticComponent=d&lat1=37%C2%B0+48%27+5%22&lat1Hemisphere=N&lon1=122%C2%B0+14%27+54%22&lon1Hemisphere=W&model=WMM&startYear=2021&startMonth=4&startDay=15&resultFormat=json
    
    Request URL: https://www.ngdc.noaa.gov/geomag-web/calculators/calculateDeclination?browserRequest=true&magneticComponent=d&lat1=37.801815&lat1Hemisphere=N&lon1=122.248558&lon1Hemisphere=W&model=WMM&startYear=2021&startMonth=4&startDay=20&resultFormat=json
    
    */
    static fetchDeclinationRad(lat, lon, dateRef) {
        dateRef = dateRef || new Date
        const q = {
            magneticComponent : 'd',
            lat1: '',
            lat1Hemis
        }
    }
}
module.exports = Util