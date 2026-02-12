const DEG_NULL = 1000
const POS_NULL = 10_000_000

class MotorController {

  constructor({motors}) {
    this.values = []
    this.isInit = false
    this.motors = motors.map((m, i) => new Motor({...m, mc: this, id: i + 1}))
  }

  update(values) {
    if (typeof values === 'string') {
      values = values.split('|').map(x => parseInt(x, 16))
    }
    Object.assign(this.values, values)
    this.isInit = true
  }

  get statusFlag() {
    return this.values[0]
  }

  get busy() {
    return this.statusFlag == null
      ? null
      : (this.statusFlag & 1) === 1
  }
}

class Motor {

  constructor({mc, id, label}) {
    this.mc = mc
    this.id = id
    this.label = label
  }

  get flag() {
    return this.mc.values[0] >> (4 + 8 * (this.id - 1))
  }

  get isLimit_cw() {
    return this.checkBit(0)
  }

  get isLimit_acw() {
    return this.checkBit(1)
  }

  get isMoving() {
    return this.checkBit(2)
  }

  get isActive() {
    return this.checkBit(3)
  }

  get hasHomed() {
    return this.checkBit(4)
  }

  get limitsEnabled() {
    return this.checkBit(5)
  }

  get isHoming() {
    return this.checkBit(6)
  }

  get isEnding() {
    return this.checkBit(7)
  }

  get pos() {
    const value = this.mc.values[this.id]
    return value === POS_NULL
      ? null
      : value
  }

  get maxSpeed() {
    return this.valueAt(1)
  }

  get acceleration() {
    return this.valueAt(2)
  }

  get millistepsPerDegree() {
    return this.valueAt(3)
  }

  get maxDegrees() {
    return this.valueAt(4)
  }

  get defaultSpeed() {
    return this.valueAt(5)
  }

  get absMaxSpeed() {
    return this.valueAt(6)
  }

  get maxAcceleration() {
    return this.valueAt(7)
  }

  get stepsPerDegree() {
    return this.millistepsPerDegree / 1000 || null
  }

  get degreesPerStep() {
    const {stepsPerDegree} = this
    return stepsPerDegree
      ? 1 / stepsPerDegree
      : null
  }

  get posDegrees() {
    const {pos, degreesPerStep} = this
    return (pos == null || degreesPerStep == null)
      ? null
      : pos * degreesPerStep
  }

  checkBit(bit) {
    const {flag} = this
    const n = 1 << bit
    return flag == null
      ? null
      : (flag & n) === n
  }

  valueAt(i) {
    return this.mc.values[7 * (this.id - 1) + this.mc.motors.length + i]
  }
}

class Gps {

  constructor() {
    this.values = []
    this.isInit = false
  }

  update(values) {
    if (typeof values === 'string') {
      values = values.split('|').map(x => parseFloat(x))
    }
    Object.assign(this.values, values)
    this.isInit = true
  }

  get fix() {
    return Boolean(this.values[0])
  }

  get lat() {
    return this.fix ? this.values[1] : null
  }

  get lon() {
    return this.fix ? this.values[2] : null
  }

  get angle() {
    return this.fix ? this.values[3] : null
  }
}

class Orientation {

  constructor({label}) {
    this.label = label
    this.values = []
    this.isInit = false
  }

  update(values) {
    if (typeof values === 'string') {
      values = values.split('|').map(x => parseFloat(x))
    }
    Object.assign(this.values, values)
    this.isInit = true
  }

  get x() {
    return degNullSafe(this.values[0])
  }

  get y() {
    return degNullSafe(this.values[1])
  }

  get z() {
    return degNullSafe(this.values[2])
  }

  get qw() {
    return degNullSafe(this.values[3])
  }

  get qx() {
    return degNullSafe(this.values[4])
  }

  get qy() {
    return degNullSafe(this.values[5])
  }

  get qz() {
    return degNullSafe(this.values[6])
  }

  get temp() {
    return this.values[7]
  }

  get cal_system() {
    return this.values[8]
  }

  get cal_gyro() {
    return this.values[9]
  }

  get cal_accel() {
    return this.values[10]
  }

  get cal_mag() {
    return this.values[11]
  }

  get isCalibrated() {
    return Boolean(this.values[12])
  }

  get calibration() {
    return [this.cal_system, this.cal_gyro, this.cal_accel, this.cal_mag]
  }
}

class Magnetometer {

  constructor({label}) {
    this.label = label
    this.values = []
    this.isInit = false
  }

  update(values) {
    if (typeof values === 'string') {
      values = values.split('|').map(x => parseFloat(x))
    }
    Object.assign(this.values, values)
    this.isInit = true
  }

  get heading() {
    return degNullSafe(this.values[0])
  }

  get x() {
    return degNullSafe(this.values[1])
  }

  get y() {
    return degNullSafe(this.values[2])
  }

  get z() {
    return degNullSafe(this.values[3])
  }
}

const mc = new MotorController({
  motors: [
    {label: 'scope'},
    {label: 'base'},
  ]
})
const gps = new Gps
const ori = new Orientation({label: 'Scope'})
const orf = new Orientation({label: 'Base'})
const mag = new Magnetometer({label: 'Base'})

// return 'Error' if undefined
function ed(value) {
  return typeof value == 'undefined' ? 'Error' : value
}

function fixedSafe(val, n) {
  if (typeof val == 'number' && !isNaN(val)) {
    return val.toFixed(n)
  }
  return '' + val
}

function degNullSafe(value) {
  return value === DEG_NULL ? null : value
}

$(() => {
  let requestBusy = false
  let refreshBusy = false
  let refreshInterval

  setRefreshInterval()
  setTimeout(refreshStatus)

  $('form').on('click', function (e) {
    const target = $(e.target)
    if (target.hasClass('go')) {
      e.preventDefault()
      clearOutputs()
      if (requestBusy || target.hasClass('disabled') || target.prop('disabled')) {
        return
      }
      let cmd
      try {

        if (target.is('#stopsignal')) {
          cmd = getStopCommand()
        } if (target.is('#home_1')) {
          cmd = getHomeSingleCommand(1)
        } else if (target.is('#home_2')) {
          cmd = getHomeSingleCommand(2)
        } else if (target.is('#home_all')) {
          cmd = getHomeAllCommand()
        } else if (target.is('#end_1')) {
          cmd = getEndSingleCommand(1)
        } else if (target.is('#end_2')) {
          cmd = getEndSingleCommand(2)
        } else if (target.is('#end_all')) {
          cmd = getEndAllCommand()
        } else if (target.is('#go_up')) {
          cmd = getMoveSingleCommand(1, 1)
        } else if (target.is('#go_down')) {
          cmd = getMoveSingleCommand(1, 2)
        } else if (target.is('#go_left')) {
          cmd = getMoveSingleCommand(2, 2)
        } else if (target.is('#go_right')) {
          cmd = getMoveSingleCommand(2, 1)
        } else if (target.is('#go_both')) {
          cmd = getMoveBothCommand()
        } else if (target.is('#reset_motorcontroller')) {
          cmd = getResetMotorContollerCommand()
        } else if (target.is('#go_raw')) {
          cmd = getRawCommand()
        }

        sendRequest('command', 'POST', { command: cmd })

      } catch (err) {
        console.error(err)
      }

    } else if (target.is('#refresh_status')) {
      e.preventDefault()
      refreshStatus()
    } else if (target.is('#gauger_connected_status')) {
      e.preventDefault()
      handleGaugerConnectButton()
    }
  })
  $('form').on('change', function (e) {
    const $target = $(e.target)
    if ($target.is('#refresh_interval')) {
      setRefreshInterval()
    }
  })

  $('#clear_outputs').on('click', clearOutputs)

  async function handleGaugerConnectButton() {
    const $target = $('#gauger_connected_status')
    if ($target.hasClass('disabled')) {
      return
    }
    $target.addClass('disabled')
    const action = $target.hasClass('connected') ? 'disconnect' : 'connect'
    try {
      if (confirm('Are you sure you want to ' + action + '?')) {
        clearRefreshInterval()
        $target.text(action + 'ing...')
        const res = await fetch('/' + action, { method: 'POST' })
        const { status } = await res.json()
        writeStatus(status)
      }
    } catch (err) {
      writeStatus()
      console.error(action, 'failed', err)
    } finally {
      setRefreshInterval()
      $target.removeClass('disabled')
    }
  }

  // TODO: give ui feedback that refresh is working, last update
  async function refreshStatus() {
    if (refreshBusy) {
      return
    }
    refreshBusy = true
    try {
      const res = await fetch('status')
      const { status } = await res.json()
      writeStatus(status)
    } catch (err) {
      writeStatus()
      console.error('Status refresh failed', err)
    } finally {
      refreshBusy = false
    }
  }

  function clearRefreshInterval() {
    clearInterval(refreshInterval)
  }

  function setRefreshInterval() {
    clearRefreshInterval()
    const seconds = parseInt($('#refresh_interval').val())
    if (!isNaN(seconds) && seconds > 0) {
      refreshInterval = setInterval(refreshStatus, seconds * 1000)
    }
  }

  function clearOutputs() {
    $('.output').text('')
  }

  function sendRequest(uri, method, body) {
    if (requestBusy) {
      console.log('Busy, ignoring')
      return
    }
    requestBusy = true
    $('.go').addClass('disabled').prop('disabled', true)
    clearOutputs()
    const opts = { method }
    if (body) {
      opts.headers = {
        'Content-Type': 'application/json'
      }
      opts.body = JSON.stringify(body)
    }
    //console.log('Sending', req)
    var reqText = [method, uri].join(' ')
    if (body) {
      reqText = [reqText, '\n', JSON.stringify(body, null, 2)].join('\n')
    }
    $('#request_output').text(reqText)
    return fetch(uri, opts).then(res => {
      requestBusy = false
      $('.go').removeClass('disabled').prop('disabled', false)
      res.json().then(resBody => {
        //console.log(resBody)
        $('#response_output').text(JSON.stringify(resBody, null, 2))
      }).catch(err => {
        console.error(err)
        $('#response_output').text(err)
      })
    }).catch(err => {
      requestBusy = false
      $('.go').removeClass('disabled').prop('disabled', false)
      console.error(err)
      $('#response_output').text(err)
    })
  }


  function writeStatus(status) {
    const modmap = new Map(
      (status.mod || []).map(x => [x.label, x])
    )
    if (modmap.has('MCI')) {
      mc.update(modmap.get('MCI').raw)
    }
    if (modmap.has('GPS')) {
      gps.update(modmap.get('GPS').raw)
    }
    if (modmap.has('ORI')) {
      ori.update(modmap.get('ORI').raw)
    }
    if (modmap.has('ORF')) {
      orf.update(modmap.get('ORF').raw)
    }
    if (modmap.has('MAG')) {
      mag.update(modmap.get('MAG').raw)
    }

    const [m1, m2] = mc.motors
    const oriNames = ['x', 'y', 'z', 'qw', 'qx', 'qy', 'qz']

    $('#controller_state').text(
      mc.busy == null
        ? 'null'
        : (
          mc.busy
            ? 'Busy'
            : 'Ready'
        )
    )
    $('#position_m1').text(fixedSafe(m1.posDegrees, 2))
    $('#position_m2').text(fixedSafe(m2.posDegrees, 2))

    const gaugerConnectedStatus = status.isDeviceConnected
      ? 'Connected'
      : 'Disconnected'
    $('#gauger_connected_status').text(ed(gaugerConnectedStatus))
      .removeClass('connected disconnected')
      .addClass(ed(gaugerConnectedStatus).toLowerCase())

    $('#is_mc_init').text('' + ed(status.isMccInit))
    $('#limitsEnabled_m1').text('' + ed(m1.limitsEnabled))
    $('#limitsEnabled_m2').text('' + ed(m2.limitsEnabled))
    $('#limitState_m1_cw').text('' + ed(m1.isLimit_cw))
    $('#limitState_m1_acw').text('' + ed(m1.isLimit_acw))
    $('#limitState_m2_cw').text('' + ed(m2.isLimit_cw))
    $('#limitState_m2_acw').text('' + ed(m2.isLimit_acw))
    $('#maxSpeed_m1').text('' + ed(m1.maxSpeed))
    $('#maxSpeed_m2').text('' + ed(m2.maxSpeed))
    $('#acceleration_m1').text('' + ed(m1.acceleration))
    $('#acceleration_m2').text('' + ed(m2.acceleration))

    $('#is_orientation_init').text('' + ed(ori.isInit))
    for (const name of oriNames) {
      $('#orientation_' + name).text(fixedSafe(ed(ori[name]), 4))
    }
    $('#temperature').text('' + ed(ori.temp))
    $('#orienation_calibration').text(ori.calibration.map(v => '' + ed(v)).join('|'))
    $('#is_orientation_calibrated').text('' + ed(ori.isCalibrated))

    $('#is_base_orientation_init').text('' + ed(orf.isInit))
    for (const name of oriNames) {
      $('#base_orientation_' + name).text(fixedSafe(ed(orf[name]), 4))
    }
    $('#base_temperature').text('' + ed(orf.temp))
    $('#base_orienation_calibration').text(orf.calibration.map(v => '' + ed(v)).join('|'))
    $('#is_base_orientation_calibrated').text('' + ed(orf.isCalibrated))

    $('#is_gps_init').text('' + ed(gps.isInit))
    $('#gps_lat').text('' + fixedSafe(ed(gps.lat), 6))
    $('#gps_long').text('' + fixedSafe(ed(gps.lon), 6))
    $('#gps_angle').text('' + fixedSafe(ed(gps.angle), 6))

    $('#is_mag_init').text('' + ed(mag.isInit))
    $('#mag_heading').text('' + fixedSafe(ed(mag.heading), 4))

  }

  // :04 <motorId> <direction> <degrees>;
  function getMoveSingleCommand(motorId, direction) {
    const howMuch = $('#in_howmuch').val()
    const unit = $('#in_units').val()
    if (isNaN(parseFloat(howMuch))) {
      throw new Error('Invalid input: ' + howMuch)
    }
    return [unit == 'steps' ? ':01' : ':04', motorId, direction, howMuch].join(' ') + ';\n'
  }

  // :06 <motorId>;
  function getHomeSingleCommand(motorId) {
    return [':06', motorId].join(' ') + ';\n'
  }

  // :07 ;
  function getHomeAllCommand() {
    return ':07 ;\n'
  }

  // :08 <motorId>;
  function getEndSingleCommand(motorId) {
    return [':08', motorId].join(' ') + ';\n'
  }

  // :09 ;
  function getEndAllCommand() {
    return ':09 ;\n'
  }

  // :11 <direction_1> <degrees_1> <direction_2> <degrees_2>;
  function getMoveBothCommand() {
    const unit = $('#in_units2').val()
    const dir1 = parseInt($('#in_dir1').val())
    const dir2 = parseInt($('#in_dir2').val())
    const howMuch1 = parseFloat($('#in_howmuch1').val())
    const howMuch2 = parseFloat($('#in_howmuch2').val())
    const isSameTime = $('#in_sametime').is(':checked') ? 'T' : 'F'
    if (dir1 !== 1 && dir1 !== 2) {
      throw new Error('Invalid direction_1 value: ' + dir1)
    }
    if (dir2 !== 1 && dir2 !== 2) {
      throw new Error('Invalid direction_2 value: ' + dir2)
    }
    if (isNaN(howMuch1)) {
      throw new Error('Invalid howmuch_1 value')
    }
    if (isNaN(howMuch2)) {
      throw new Error('Invalid howmuch_2 value')
    }
    return [unit == 'steps' ? ':10' : ':11', dir1, howMuch1, dir2, howMuch2, isSameTime].join(' ') + ';\n'
  }

  function getStopCommand() {
    return ':76 ;\n'
  }

  function getResetMotorContollerCommand() {
    return ':77 ;\n'
  }

  function getRawCommand() {
    const text = $('#in_raw').val().trim()
    if (!text.length) {
      throw new Error('Empty input')
    }
    return text + '\n'
  }
})
