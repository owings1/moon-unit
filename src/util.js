class Util {

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
}
module.exports = Util