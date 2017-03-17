// Create the canvas
var canvas = document.createElement("canvas");
var ctx = canvas.getContext("2d");
canvas.width = 640;
canvas.height = 480;
document.body.appendChild(canvas);


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
canvas.addEventListener("click",fullscreen)

var camera = new Image();
camera.read = false;
camera.onload = function () {
    camera.ready = true;
};
camera.src = "/camera";

var car = {
    speed: 0.0,
    direction: 0.0
};

// Handle keyboard controls
var keysDown = {};

addEventListener("keydown", function (e) {
    keysDown[e.keyCode] = true;
}, false);

addEventListener("keyup", function (e) {
    delete keysDown[e.keyCode];
}, false);

var update = function (modifier) {
    var updated = false;
    if (38 in keysDown) { // Holding up
        updated = updated || (car.speed != 1.0);
        car.speed = 1.0;
    } else if (40 in keysDown) { // Holding down
        updated = updated || (car.speed != -1.0);
        car.speed = -1.0;
    } else {
        updated = updated || (car.speed != 0.0);
        car.speed = 0.0;
    }
    if (37 in keysDown) { // Holding left
        updated = updated || (car.direction != -1.0);
        car.direction = -1.0;
    } else if (39 in keysDown) { // Holding right
        updated = updated || (car.direction != 1.0);
        car.direction = 1.0;
    } else {
        updated = updated || (car.direction != 0.0);
        car.direction = 0.0;
    }

    if (updated) {
        console.log(car);
        $.post('/control', car, 'json');
    }

    ctx.fillStyle = "rgb(250, 250, 250)";
    ctx.font = "24px Helvetica";
    ctx.textAlign = "left";
    ctx.textBaseline = "top";
    ctx.fillText("Speed: " + car.speed, 32, 32);
    ctx.fillText("Direction: " + car.direction, 32, 64);
}

// Draw everything
var render = function () {
    if (camera.ready) {
        ctx.drawImage(camera, 0, 0);
    }

    ctx.fillStyle = "rgb(250, 250, 250)";
    ctx.font = "24px Helvetica";
    ctx.textAlign = "left";
    ctx.textBaseline = "top";
    ctx.fillText("Speed: " + car.speed, 32, 32);
    ctx.fillText("Direction: " + car.direction, 32, 64);
};

var main = function () {
    var now = Date.now();
    var delta = now - then;

    update(delta / 1000);
    render();

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
