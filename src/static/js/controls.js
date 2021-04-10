$(document).ready(function() {

    // :04 <motorId> <direction> <degrees>;
    function getMoveDegreesCommand(motorId, direction) {
        return [':04', motorId, direction, $('#in_degrees').val()].join(' ') + ';\n'
    }

    var busy = false

    function sendCommand(command) {
        if (busy) {
            console.log('Busy, ignoring')
            return
        }
        busy = true
        var opts = {
            method: 'POST',
            body: JSON.stringify({command}),
            headers: {
                'Content-Type' : 'application/json'
            }
        }
        console.log('Sending', {command})
        fetch('command/sync', opts).then(res => {
            busy = false
            res.text().then(console.log)
        }).catch(err => {
            console.error(err)
            busy = false
        })
    }

    $('form').on('click', function(e) {

        var $target = $(e.target)

        if ($target.hasClass('go')) {

            var motorId, direction

            if ($target.is('#go_up')) {
                motorId = 1
                direction = 2
            } else if ($target.is('#go_down')) {
                motorId = 1
                direction = 1
            } else if ($target.is('#go_left')) {
                motorId = 2
                direction = 2
            } else if ($target.is('#go_right')) {
                motorId = 2
                direction = 1
            }

            var cmd = getMoveDegreesCommand(motorId, direction)
            sendCommand(cmd)
        }
        
    })
})