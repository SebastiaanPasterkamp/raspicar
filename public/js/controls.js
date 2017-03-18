// Create the canvas
var canvas = document.createElement("canvas");
var ctx = canvas.getContext("2d");
canvas.width = 640;
canvas.height = 480;
document.body.appendChild(canvas);

var camera = new Image();
camera.read = false;
camera.onload = function () {
    camera.ready = true;
};
camera.src = "/camera";


document.body.addEventListener('touchmove', function(event) {
    event.preventDefault();
}, false);

function fullscreen() {
    if (canvas.webkitRequestFullscreen) {
        canvas.webkitRequestFullscreen(Element.ALLOW_KEYBOARD_INPUT);
    } else if (canvas.mozRequestFullScreen) {
        canvas.mozRequestFullScreen();
    } else if (canvas.msRequestFullscreen) {
        canvas.msRequestFullscreen();
    } else {
        canvas.requestFullscreen(); // standard
    }
}
canvas.addEventListener("click",fullscreen);

var controls = [
    {
        id: null, // Touch.identifier currently controlling this joystick
        name: 'car',
        x: {
            value: 0.0,
            name: 'direction'
        },
        y: {
            value: 0.0,
            name: 'speed'
        },
        updated: false,
        area: function(rect) { // Bounding box
            return {
                x: 30,
                y: rect.height - 180,
                width: 150,
                height: 150
            };
        }
    },
    {
        id: null,
        name: 'camera',
        x: {
            value: 0.0,
            name: 'pan'
        },
        y: {
            value: 0.0,
            name: 'tilt'
        },
        updated: false,
        area: function(rect) { // Bounding box
            return {
                x: rect.width - 180,
                y: rect.height - 180,
                width: 150,
                height: 150
            };
        },
    }
];

var relativeCoords = function(event, rect) {
    return {
        x: event.pageX - rect.left,
        y: event.pageY - rect.top
    };
};

var isInside = function(point, area) {
    return (
        point.x >= area.x && point.x <= area.x + area.width
        && point.y >= area.y && point.y <= area.y + area.height
    );
};

var setValues = function(control, point, area) {
    control.x.value = Math.min(1.0, Math.max(-1.0,
        (point.x - area.x - (area.width/2)) / (area.width/2)
    ));
    control.y.value = Math.min(1.0, Math.max(-1.0,
        -(point.y - area.y - (area.height/2)) / (area.height/2)
    ));
    control.updated = true;
};

canvas.addEventListener("touchstart", function(event) {
    var rect = canvas.getBoundingClientRect();
    for (var i=0; i<event.changedTouches.length; i++) {
        var touch = event.changedTouches[i];
        var point = relativeCoords(touch, rect);
        for (var j=0; j<controls.length; j++) {
            var control = controls[j];
            var area = control.area(rect);
            if (isInside(point, area)) {
                control.identifier = touch.identifier;
                setValues(control, point, area);
            }
        }
    }
});

canvas.addEventListener("touchend", function(event){
    for (var i=0; i<event.changedTouches.length; i++) {
        var touch = event.changedTouches[i];
        for (var j=0; j<controls.length; j++) {
            var control = controls[j];
            if (control.identifier == touch.identifier) {
                control.identifier = null;
                control.x.value = 0.0;
                control.y.value = 0.0;
                control.updated = true;
            }
        }
    }
});

canvas.addEventListener('touchmove', function(event) {
    var rect = canvas.getBoundingClientRect();
    for (var i=0; i<event.changedTouches.length; i++) {
        var touch = event.changedTouches[i];
        for (var j=0; j<controls.length; j++) {
            var control = controls[j];
            if (control.identifier == touch.identifier) {
                setValues(
                    control,
                    relativeCoords(touch, rect),
                    control.area(rect)
                );
            }
        }
    }
}, false);

// Handle keyboard controls
var keysDown = {};

var keycontrol = function() {
    var control = controls[0];
    if (38 in keysDown) { // Holding up
        control.y.value = 1.0;
        control.updated = true;
    } else if (40 in keysDown) { // Holding down
        control.y.value = -1.0;
        control.updated = true;
    } else {
        control.y.value = 0.0;
        control.updated = true;
    }
    if (37 in keysDown) { // Holding left
        control.x.value = -1.0;
        control.updated = true;
    } else if (39 in keysDown) { // Holding right
        control.x.value = 1.0;
        control.updated = true;
    } else {
        control.x.value = 0.0;
        control.updated = true;
    }
};

addEventListener("keydown", function (e) {
    keysDown[e.keyCode] = true;
    keycontrol();
}, false);

addEventListener("keyup", function (e) {
    delete keysDown[e.keyCode];
    keycontrol();
}, false);

setInterval(function() {
    var updates = {};
    var updated = false;
    for (var i=0; i<controls.length; i++) {
        var control = controls[i];
        if (control.updated) {
            updates[control.name] = {};
            updates[control.name][control.x.name] = control.x.value;
            updates[control.name][control.y.name] = control.y.value;
            control.updated = false;
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
}, 250);

var update = function (modifier) {
    if (camera.ready) {
        ctx.drawImage(camera, 0, 0);
    }

    for (var i=0; i<controls.length; i++) {
        var control = controls[i];
        var area = control.area(canvas.getBoundingClientRect());

        ctx.fillStyle = "rgb(250, 250, 250, 0.75)";

        ctx.beginPath();
        ctx.rect(
            area.x, area.y,
            area.width, area.height
        );
        ctx.stroke();

        ctx.beginPath();
        ctx.arc(
            area.x + area.width / 2 + control.x.value * area.width / 2,
            area.y + area.height / 2 - control.y.value * area.height / 2,
            10,
            0, 2*Math.PI
        );
        ctx.fill();

        ctx.fillStyle = "rgb(250, 250, 250)";
        ctx.font = "24px Helvetica";
        ctx.textAlign = "left";
        ctx.textBaseline = "top";
        ctx.fillText(control.x.name + ": " + control.x.value.toFixed(2), 32, 32 + i*64);
        ctx.fillText(control.y.name + ": " + control.y.value.toFixed(2), 32, 64 + i*64);
    }
}

var main = function () {
    var now = Date.now();
    var delta = now - then;

    update(delta / 1000);

    then = now;

    // Request to do this again ASAP
    requestAnimationFrame(main);
};

// Cross-browser support for requestAnimationFrame
var w = window;
var requestAnimationFrame = w.requestAnimationFrame
    || w.webkitRequestAnimationFrame
    || w.msRequestAnimationFrame
    || w.mozRequestAnimationFrame;

// Let's play this game!
var then = Date.now();
main();
