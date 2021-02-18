#!/usr/bin/env python

import argparse
import os
import cv2
import time
import json

from modules.camera import VideoCamera
from modules.vehicle import Car

usePiCamera = False
try:
    import RPi.GPIO as GPIO
    from sunfounder.PCA9685 import PWM
    usePiCamera = True
except ImportError as e:
    print("Not running on a Raspberry Pi", e)
    from test.mockobjects import MockGPIO, MockPWM as PWM
    GPIO = MockGPIO()


def panorama(config, car, camera):
    panorama = config.get("panorama", {})
    steps = panorama.get("steps", 6)
    pan = panorama.get("pan", (-.2, .2))
    tilt = panorama.get("tilt", (.6, .8))
    delay = panorama.get("delay", 3.0)
    directory = panorama.get("directory", "panorama")

    os.makedirs(directory, mode=0o750, exist_ok=True)

    next_capture = time.time() + delay
    reverse = False
    for y in range(steps+1):
        for x in sorted(range(steps+1), reverse=reverse):
            try:
                filename = os.path.join(
                    directory, f'image-{x:02d}-{y:02d}.png')
                p, t = (
                    pan[0] + ((pan[1]-pan[0])*x/steps),
                    tilt[0] + ((tilt[1]-tilt[0])*y/steps),
                    )
                car.setPanTilt(p, t)
                duration = max(0, next_capture - time.time())
                print(f"{x},{y} [{p:.2f}, {t:.2f}]: Filename: {filename}")
                time.sleep(duration)
                img = camera.read()
                cv2.imwrite(filename, img)
                next_capture = time.time() + delay
            except KeyboardInterrupt:
                return

        reverse = not reverse


def loadConfig(filename):
    config = {}
    with open(filename, 'r') as fh:
        config = json.load(fh)
    return config


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Make panorama shots')
    parser.add_argument('--config', metavar='file.json', default="config.json",
                        help="Config file to use. [default: %(default)s]")

    args = parser.parse_args()

    config = loadConfig(args.config)

    GPIO.setmode(GPIO.BOARD)    # Number GPIOs by its physical location
    pwm = PWM()                 # The servo controller.
    pwm.frequency = 60

    car = Car(pwm, GPIO, config=config)
    car.start()

    camera = VideoCamera(-1, usePiCamera=usePiCamera, resolution=(1280, 720))
    camera.start()

    panorama(config, car, camera)

    car.setPanTilt(0.0, 0.7)
    car.stop()
    camera.stop()
