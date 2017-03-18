// Create the canvas
var canvas = document.createElement("canvas");
var ctx = canvas.getContext("2d");
canvas.width = 640;
canvas.height = 480;
document.body.appendChild(canvas);

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

touch = {
    started: false,
    start_x: canvas.width/2.0,
    start_y: canvas.height/2.0,
    x: 0,
    y: 0
};

var car = {
    speed: 0.0,
    direction: 0.0,
    updated: false
};


canvas.addEventListener("click",fullscreen);

canvas.addEventListener("touchstart", function(event){
    var e = event.touches[0];
    var rect = canvas.getBoundingClientRect();
    console.log([e.clientX - rect.left, e.clientY - rect.top]);
    console.log(e);
    touch.start_x = e.pageX;
    touch.start_y = e.pageY;

    console.log(["start", touch.start_x, '=', e.pageX]);
});

canvas.addEventListener("touchend", function(event){
    car.speed = 0.0;
    car.direction = 0.0;
    car.updated = true;
});

canvas.addEventListener('touchmove', function(event) {
    var e = event.touches[0];
    touch.x = e.pageX;
    touch.y = e.pageY;

    console.log(["move", touch.x, '=', e.pageX, '=>', touch.start_y - touch.y]);

    car.speed = Math.min(1.0, Math.max(-1.0, (touch.start_y - touch.y) / 75.0));
    car.direction = Math.min(1.0, Math.max(-1.0, (touch.start_x - touch.x) / 75.0));

    car.updated = true;
}, false);

var camera = new Image();
camera.read = false;
camera.onload = function () {
    camera.ready = true;
};
camera.src = "/camera";

// Handle keyboard controls
var keysDown = {};

var keycontrol = function() {
    if (38 in keysDown) { // Holding up
        car.updated = car.updated || (car.speed != 1.0);
        car.speed = 1.0;
    } else if (40 in keysDown) { // Holding down
        car.updated = car.updated || (car.speed != -1.0);
        car.speed = -1.0;
    } else {
        car.updated = car.updated || (car.speed != 0.0);
        car.speed = 0.0;
    }
    if (37 in keysDown) { // Holding left
        car.updated = car.updated || (car.direction != -1.0);
        car.direction = -1.0;
    } else if (39 in keysDown) { // Holding right
        car.updated = car.updated || (car.direction != 1.0);
        car.direction = 1.0;
    } else {
        car.updated = car.updated || (car.direction != 0.0);
        car.direction = 0.0;
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
    if (car.updated) {
        console.log(car);
        $.ajax({
            type: 'POST',
            url: '/control',
            data: JSON.stringify(car),
            contentType: "application/json",
            dataType: 'json'
        });
        car.updated = false;
    }
}, 250);

var update = function (modifier) {
    if (camera.ready) {
        ctx.drawImage(camera, 0, 0);
    }

    var rect = canvas.getBoundingClientRect();

    ctx.beginPath();
    ctx.rect(
        30, rect.height - 180,
        150, 150
    );
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(
        105 - car.direction * 75, rect.height - 105 - car.speed * 75,
        10,
        0, 2*Math.PI
    );
    ctx.stroke();



    ctx.fillStyle = "rgb(250, 250, 250)";
    ctx.font = "24px Helvetica";
    ctx.textAlign = "left";
    ctx.textBaseline = "top";
    ctx.fillText("Speed: " + car.speed, 32, 32);
    ctx.fillText("Direction: " + car.direction, 32, 64);
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
