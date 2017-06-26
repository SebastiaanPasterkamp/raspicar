// Create the canvas
var $app = {
    canvas: null,
    ctx: null,
    requestAnimationFrame: null,
    capture: false,
    grid: false,
    record: false,

    then: Date.now(),

    isInside: function(point) {
        var area = this.area();
        return (
            point.x >= area.x && point.x <= area.x + area.width
            && point.y >= area.y && point.y <= area.y + area.height
        );
    },

    setValues: function(point) {
        var area = this.area(),
            x = Math.min(1.0, Math.max(-1.0,
                (point.x - area.x - (area.width/2)) / (area.width/2)
            )),
            y  = Math.min(1.0, Math.max(-1.0,
                -(point.y - area.y - (area.height/2)) / (area.height/2)
            ));
        if (this.relative) {
            this.x.delta = x;
            this.y.delta = y;
        }
        else {
            this.x.value = x;
            this.y.value = y;
            this.updated = true;
        }
    },

    resetValues: function() {
        this.x.value = this.x.default;
        this.y.value = this.y.default;
        if (this.relative) {
            this.x.delta = 0.0;
            this.y.delta = 0.0;
        } else {
            this.updated = true;
        }
    },

    updateValues: function() {
        if (this.relative) {
            var x = Math.min(
                    1.0,
                    Math.max(-1.0, this.x.value + this.x.delta * 0.025)
                ),
                y = Math.min(
                    1.0,
                    Math.max(-1.0, this.y.value + this.y.delta * 0.025)
                ),
                updated = false;
            updated = this.x.value != x || this.y.value != y;
            this.x.value = x;
            this.y.value = y;
            return updated;
        }
        else if (this.updated) {
            this.updated = false;
            return true;
        }
        return false;
    },

    controls: [],

    relativeCoords: function(event, rect) {
        return {
            x: event.pageX - rect.left,
            y: event.pageY - rect.top
        };
    },

    // Handle keyboard controls
    keysDown: {},

    keycontrol: function() {
        var control = $app.controls[0];
        if ($app.keysDown[38]) { // Holding up
            control.y.value = 1.0;
            control.updated = true;
        } else if ($app.keysDown[40]) { // Holding down
            control.y.value = -1.0;
            control.updated = true;
        } else {
            control.y.value = 0.0;
            control.updated = true;
        }
        if ($app.keysDown[37]) { // Holding left
            control.x.value = -1.0;
            control.updated = true;
        } else if ($app.keysDown[39]) { // Holding right
            control.x.value = 1.0;
            control.updated = true;
        } else {
            control.x.value = 0.0;
            control.updated = true;
        }
    },

    update: function (modifier) {
        $app.ctx.clearRect(0, 0, $app.canvas.width, $app.canvas.height);
        for (var i=0; i<$app.controls.length; i++) {
            var control = $app.controls[i];
            var area = control.area($app.canvas.getBoundingClientRect());

            $app.ctx.fillStyle = "rgb(250, 250, 250, 0.75)";
            $app.ctx.beginPath();
            $app.ctx.rect(
                area.x, area.y,
                area.width, area.height
            );
            $app.ctx.stroke();

            $app.ctx.fillStyle = "rgb(250, 250, 250, 0.75)";
            $app.ctx.beginPath();
            $app.ctx.arc(
                area.x + area.width / 2 + control.x.value * area.width / 2,
                area.y + area.height / 2 - control.y.value * area.height / 2,
                10,
                0, 2*Math.PI
            );
            $app.ctx.fill();

            $app.ctx.fillStyle = "rgb(250, 250, 250)";
            $app.ctx.font = "24px Helvetica";
            $app.ctx.textAlign = "left";
            $app.ctx.textBaseline = "top";
            $app.ctx.fillText(control.x.name + ": " + control.x.value.toFixed(2), 32, 32 + i*64);
            $app.ctx.fillText(control.y.name + ": " + control.y.value.toFixed(2), 32, 64 + i*64);
        }
    },

    main: function () {
        var now = Date.now();
        var delta = now - $app.then;

        $app.update(delta / 1000);

        $app.then = now;

        // Request to do this again ASAP
        $app.requestAnimationFrame.call(window, $app.main);
    }
};

$app.controls = [
    {
        id: null, // Touch.identifier currently controlling this joystick
        name: 'car',
        x: {
            value: 0.0,
            default: 0.0,
            name: 'direction'
        },
        y: {
            value: 0.0,
            default: 0.0,
            name: 'speed'
        },
        relative: false,
        update: $app.updateValues,
        area: function() { // Bounding box
            var rect = $app.canvas.getBoundingClientRect();
            return {
                x: 30,
                y: rect.height - 180,
                width: 150,
                height: 150
            };
        },
        isInside: $app.isInside,
        setValues: $app.setValues,
        resetValues: $app.resetValues
    },
    {
        id: null,
        name: 'camera',
        x: {
            value: 0.0,
            delta: 0.0,
            default: 0.0,
            name: 'pan'
        },
        y: {
            value: 0.7,
            delta: 0.0,
            default: 0.7,
            name: 'tilt'
        },
        relative: true,
        update: $app.updateValues,
        area: function() { // Bounding box
            var rect = $app.canvas.getBoundingClientRect();
            return {
                x: rect.width - 180,
                y: rect.height - 180,
                width: 150,
                height: 150
            };
        },
        isInside: $app.isInside,
        setValues: $app.setValues,
        resetValues: $app.resetValues
    }
];

$(document).ready(function(){
    $app.canvas = $("#camera")[0];
    $app.ctx = $app.canvas.getContext("2d");
    $app.canvas.width = 1280;
    $app.canvas.height = 720;

    $($app.canvas).on('resize', function() {
        $(this).height($(this).width() * 720 / 1280);
    });

    $($app.canvas).on('click', function() {
        if ($app.canvas.webkitRequestFullscreen) {
            $app.canvas.webkitRequestFullscreen(Element.ALLOW_KEYBOARD_INPUT);
        } else if ($app.canvas.mozRequestFullScreen) {
            $app.canvas.mozRequestFullScreen();
        } else if ($app.canvas.msRequestFullscreen) {
            $app.canvas.msRequestFullscreen();
        } else {
            $app.canvas.requestFullscreen(); // standard
        }
        $($app.canvas).trigger('resize');
    });

    $($app.canvas).on("touchstart", function(event) {
        event = event.originalEvent;
        var rect = $app.canvas.getBoundingClientRect();
        for (var i=0; i<event.changedTouches.length; i++) {
            var touch = event.changedTouches[i];
            var point = $app.relativeCoords(touch, rect);
            for (var j=0; j<$app.controls.length; j++) {
                var control = $app.controls[j];
                if (control.isInside(point)) {
                    control.identifier = touch.identifier;
                    control.setValues(point);
                }
            }
        }
    });

    $($app.canvas).on("touchend", function(event){
        event = event.originalEvent;
        for (var i=0; i<event.changedTouches.length; i++) {
            var touch = event.changedTouches[i];
            for (var j=0; j<$app.controls.length; j++) {
                var control = $app.controls[j];
                if (control.identifier == touch.identifier) {
                    control.identifier = null;
                    control.resetValues();
                }
            }
        }
    });

    $($app.canvas).on('touchmove', function(event) {
        event.preventDefault();
        event = event.originalEvent;
        var rect = $app.canvas.getBoundingClientRect();
        for (var i=0; i<event.changedTouches.length; i++) {
            var touch = event.changedTouches[i];
            for (var j=0; j<$app.controls.length; j++) {
                var control = $app.controls[j];
                if (control.identifier == touch.identifier) {
                    var point = $app.relativeCoords(touch, rect);
                    control.setValues(point);
                }
            }
        }
    }, false);

    $('body').on("keydown", function (e) {
        $app.keysDown[e.keyCode] = true;
        $app.keycontrol();

        if (e.keyCode == 67) {
            $app.capture = ! $app.capture;
            $.ajax({
                type: 'POST',
                url: '/control',
                data: JSON.stringify({'capture': $app.capture}),
                contentType: "application/json",
                dataType: 'json'
            });
        }

        if (e.keyCode == 71) {
            $app.grid = ! $app.grid;
            $.ajax({
                type: 'POST',
                url: '/control',
                data: JSON.stringify({'grid': $app.grid}),
                contentType: "application/json",
                dataType: 'json'
            });
        }

        if (e.keyCode == 82) {
            $app.record = ! $app.record;
            $.ajax({
                type: 'POST',
                url: '/control',
                data: JSON.stringify({'record': $app.record}),
                contentType: "application/json",
                dataType: 'json'
            });
        }

        if (e.keyCode == 70) {
            // fullscreen
            $($app.canvas).trigger('click');
        }
    });

    $('body').on("keyup", function (e) {
        $app.keysDown[e.keyCode] = false;
        $app.keycontrol();
    });

    setInterval(function() {
        var updates = {};
        var updated = false;
        for (var i=0; i<$app.controls.length; i++) {
            var control = $app.controls[i];
            if (control.update()) {
                updates[control.name] = {};
                updates[control.name][control.x.name] = control.x.value;
                updates[control.name][control.y.name] = control.y.value;
                updated = true;
            }
        }
        if (updated) {
            $.ajax({
                type: 'POST',
                url: '/control',
                data: JSON.stringify(updates),
                contentType: "application/json",
                dataType: 'json'
            });
        }
    }, 100);

    // Cross-browser support for requestAnimationFrame
    var w = window;
    $app.requestAnimationFrame = w.requestAnimationFrame
        || w.webkitRequestAnimationFrame
        || w.msRequestAnimationFrame
        || w.mozRequestAnimationFrame;

    $app.main();
});
