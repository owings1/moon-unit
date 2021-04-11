$(document).ready(function() {

    function clearOutputs() {
        $('.output').text('')
    }

    $('#clear_outputs').on('click', clearOutputs)

    var busy = false

    function sendCommand(command) {
        if (busy) {
            console.log('Busy, ignoring')
            return
        }
        busy = true
        $('.go').addClass('disabled').prop('disabled', true)
        clearOutputs()
        const req = {command}
        const opts = {
            method  : 'POST',
            body    : JSON.stringify(req),
            headers : {
                'Content-Type' : 'application/json'
            }
        }
        console.log('Sending', req)
        $('#request_output').text(JSON.stringify(req, null, 2))
        fetch('command/sync', opts).then(res => {
            busy = false
            $('.go').removeClass('disabled').prop('disabled', false)
            res.json().then(resBody => {
                console.log(resBody)
                $('#response_output').text(JSON.stringify(resBody, null, 2))
            }).catch(err => {
                console.error(err)
                $('#response_output').text(err)
            })
        }).catch(err => {
            busy = false
            $('.go').removeClass('disabled').prop('disabled', false)
            console.error(err)
            $('#response_output').text(err)
        })
    }

    function sendGpio(type) {
        clearOutputs()
        const method = type == 'state' ? 'GET' : 'POST'
        $('#request_output').text([method, 'GPIO', type].join(' '))
        fetch('gpio/' + type, {method}).then(res => {
            res.json().then(resBody => {
                console.log(resBody)
                $('#response_output').text(JSON.stringify(resBody, null, 2))
            }).catch(err => {
                console.error(err)
                $('#response_output').text(err)
            })
        }).catch(err => {
            console.error(err)
            $('#response_output').text(err)
        })
    }

    $('form').on('click', function(e) {

        var $target = $(e.target)

        if ($target.hasClass('go')) {

            e.preventDefault()

            clearOutputs()

            if (busy || $target.hasClass('disabled') || $target.prop('disabled')) {
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

                sendCommand(cmd)

            } catch (err) {
                console.error(err)
            }

        } else if ($target.hasClass('gpio')) {

            e.preventDefault()

            clearOutputs()
            if ($target.is('#gpio_state')) {
                sendGpio('state')
            } else if ($target.is('#gpio_stop')) {
                sendGpio('stop')
            } else if ($target.is('#gpio_reset')) {
                sendGpio('reset')
            }
        }
    })

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