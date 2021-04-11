$(document).ready(function() {

    function getDegreesValue() {
        return $('#in_degrees').val()
    }

    // :04 <motorId> <direction> <degrees>;
    function getMoveDegreesCommand(motorId, direction) {
        const degrees = getDegreesValue()
        if (isNaN(parseFloat(degrees))) {
            throw new Error('Invalid degrees input: ' + degrees)
        }
        return [':04', motorId, direction, degrees].join(' ') + ';\n'
    }

    var busy = false

    function sendCommand(command) {
        if (busy) {
            console.log('Busy, ignoring')
            return
        }
        busy = true
        $('.go').addClass('disabled')
        const req = {command}
        var opts = {
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
            $('.go').removeClass('disabled')
            res.json().then(resBody => {
                console.log(resBody)
                $('#response_output').text(JSON.stringify(resBody, null, 2))
            }).catch(err => {
                console.error(err)
                $('#response_output').text(err)
            })
        }).catch(err => {
            busy = false
            $('.go').removeClass('disabled')
            console.error(err)
            $('#response_output').text(err)
        })
    }

    $('form').on('click', function(e) {

        var $target = $(e.target)

        if ($target.hasClass('go')) {

            e.preventDefault()

            if (busy || $target.hasClass('disabled')) {
                return
            }
            var motorId, direction

            if ($target.is('#go_up')) {
                motorId = 1
                direction = 1
            } else if ($target.is('#go_down')) {
                motorId = 1
                direction = 2
            } else if ($target.is('#go_left')) {
                motorId = 2
                direction = 2
            } else if ($target.is('#go_right')) {
                motorId = 2
                direction = 1
            }

            try {
                var cmd = getMoveDegreesCommand(motorId, direction)
                sendCommand(cmd)
            } catch (err) {
                console.error(err)
            }
        }
        
    })
})