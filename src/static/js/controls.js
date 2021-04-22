$(document).ready(function() {

    var requestBusy = false
    var refreshBusy = false
    var refreshInterval

    setRefreshInterval()

    $('form').on('click', function(e) {

        const $target = $(e.target)

        if ($target.hasClass('go')) {

            e.preventDefault()

            clearOutputs()

            if (requestBusy || $target.hasClass('disabled') || $target.prop('disabled')) {
                return
            }

            var cmd
            try {

                if ($target.is('#home_1')) {
                    cmd = getHomeSingleCommand(1)
                } else if ($target.is('#home_2')) {
                    cmd = getHomeSingleCommand(2)
                } else if ($target.is('#home_all')) {
                    cmd = getHomeAllCommand()
                } else if ($target.is('#end_1')) {
                    cmd = getEndSingleCommand(1)
                } else if ($target.is('#end_2')) {
                    cmd = getEndSingleCommand(2)
                } else if ($target.is('#end_all')) {
                    cmd = getEndAllCommand()
                } else if ($target.is('#go_up')) {
                    cmd = getMoveSingleCommand(1, 1)
                } else if ($target.is('#go_down')) {
                    cmd = getMoveSingleCommand(1, 2)
                } else if ($target.is('#go_left')) {
                    cmd = getMoveSingleCommand(2, 2)
                } else if ($target.is('#go_right')) {
                    cmd = getMoveSingleCommand(2, 1)
                } else if ($target.is('#go_both')) {
                    cmd = getMoveBothCommand()
                } else if ($target.is('#go_raw')) {
                    cmd = getRawCommand()
                }

                sendRequest('controller/command/sync', 'POST', {command: cmd})

            } catch (err) {
                console.error(err)
            }

        } else if ($target.hasClass('gpio')) {

            e.preventDefault()

            if ($target.hasClass('disabled') || $target.prop('disabled')) {
                return
            }

            clearOutputs()

            if ($target.is('#gpio_controller_state')) {
                sendRequest('controller/gpio/state')
            } else if ($target.is('#gpio_controller_stop')) {
                sendRequest('controller/gpio/stop', 'POST')
            } else if ($target.is('#gpio_controller_reset')) {
                if (confirm('Reset controller, are you sure?')) {
                    sendRequest('controller/gpio/reset', 'POST')
                }
            } else if ($target.is('#gpio_gauger_reset')) {
                if (confirm('Reset gauger, are you sure?')) {
                    sendRequest('gauger/gpio/reset', 'POST')
                }
            }
        } else if ($target.is('#refresh_status')) {

            e.preventDefault()

            refreshStatus()

        } else if ($target.is('#gauger_connected_status')) {

            e.preventDefault()

            handleGaugerConnectButton()
        }
    })

    $('form').on('change', function(e) {
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
                const res = await fetch('gauger/' + action, {method: 'POST'})
                const {status} = await res.json()
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
            const {status} = await res.json()
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
        const opts = {method}
        if (body) {
            opts.headers = {
                'Content-Type' : 'application/json'
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

    // return 'Error' if undefined
    function ed(value) {
        return typeof value == 'undefined' ? 'Error' : value
    }

    function writeStatus(status) {
        status = status || {
            // just make defaults for arrays
            position               : [],
            orientation            : [],
            orientationCalibration : [],
            limitsEnabled          : [],
            gpsCoords              : [],
            maxSpeeds              : []
        }
        $('#position_m1').html(
            ed(status.position[0]) + (!isNaN(parseFloat(status.position[0])) ? '&deg;' : '')
        )
        $('#position_m2').html(
            ed(status.position[1]) + (!isNaN(parseFloat(status.position[1])) ? '&deg;' : '')
        )
        $('#controller_state').text('' + ed(status.controllerState))
        $('#gauger_connected_status').text(ed(status.gaugerConnectedStatus))
            .removeClass('connected disconnected')
            .addClass(ed(status.gaugerConnectedStatus).toLowerCase())
        $('#orientation_x').text('' + ed(status.orientation[0]))
        $('#orientation_y').text('' + ed(status.orientation[1]))
        $('#orientation_z').text('' + ed(status.orientation[2]))
        $('#orienation_calibration').text(status.orientationCalibration.map(v => '' + ed(v)).join('|'))
        $('#is_orientation_calibrated').text('' + ed(status.isOrientationCalibrated))
        $('#limitsEnabled_m1').text('' + ed(status.limitsEnabled[0]))
        $('#limitsEnabled_m2').text('' + ed(status.limitsEnabled[1]))
        $('#gps_lat').text('' + ed(status.gpsCoords[0]))
        $('#gps_long').text('' + ed(status.gpsCoords[1]))
        $('#mag_heading').text('' + ed(status.magHeading))
        $('#declination_angle').text('' + ed(status.declinationAngle))
        $('#maxSpeed_m1').text('' + ed(status.maxSpeeds[0]))
        $('#maxSpeed_m2').text('' + ed(status.maxSpeeds[1]))
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
        if (dir1 != 1 && dir1 != 2) {
            throw new Error('Invalid direction_1 value: ' + dir1)
        }
        if (dir2 != 1 && dir2 != 2) {
            throw new Error('Invalid direction_2 value: ' + dir2)
        }
        if (isNaN(howMuch1)) {
            throw new Error('Invalid howmuch_1 value')
        }
        if (isNaN(howMuch2)) {
            throw new Error('Invalid howmuch_2 value')
        }
        return [unit == 'steps' ? ':10' : ':11', dir1, howMuch1, dir2, howMuch2].join(' ') + ';\n'
    }

    function getRawCommand() {
        const text = $('#in_raw').val().trim()
        if (!text.length) {
            throw new Error('Empty input')
        }
        return text + '\n'
    }
})
