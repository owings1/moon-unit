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

    // https://www.pveducation.org/pvcdrom/properties-of-sunlight/declination-angle
    static calcDefaultDeclinationRad() {
        const d = new Date
        const dn = Math.floor((d - new Date(d.getFullYear(), 0, 0)) / 1000 / 60 / 60 / 24)
        return Util.radians(-23.45 * Math.cos(360/365 * (dn + 10)))
    }
    /*
    Request URL: https://www.ngdc.noaa.gov/geomag-web/calculators/calculateDeclination?browserRequest=true&magneticComponent=d&lat1=37%C2%B0+48%27+5%22&lat1Hemisphere=N&lon1=122%C2%B0+14%27+54%22&lon1Hemisphere=W&model=WMM&startYear=2021&startMonth=4&startDay=15&resultFormat=json
    */
}
module.exports = Util