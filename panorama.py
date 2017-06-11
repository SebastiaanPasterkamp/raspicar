#!/bin/env python

import sys
import os
import cv2
import time
import json

sys.path.insert(0, os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..'
    )))

usePiCamera=False
try:
    import RPi.GPIO as GPIO
    from sunfounder.PCA9685 import PWM
    usePiCamera = True
except:
    from test.mockobjects import MockGPIO, MockPWM as PWM
    GPIO = MockGPIO()

from modules.vehicle import Car
from modules.camera import VideoCamera

config = {}
config_file = os.path.abspath(os.path.join(
    os.path.dirname(__file__), 'config.json'))
if os.path.exists(config_file):
    with open(config_file, 'r') as config_json:
        config = json.load(config_json)

GPIO.setmode(GPIO.BOARD)    # Number GPIOs by its physical location
pwm = PWM()                 # The servo controller.
pwm.frequency = 60

car = Car(pwm, GPIO, config=config, debug=True)
car.start()

if __name__ == "__main__":
    steps = 6
    pan = (-.2,.2)
    tilt = (.6,.8)
    delay = 3.0

    camera = VideoCamera(
        -1, usePiCamera=usePiCamera, resolution=(1280, 720)).start()

    next_capture = time.time() + delay
    reverse=False
    for y in range(steps+1):
        for x in sorted(range(steps+1), reverse=reverse):
            car.setPanTilt(
                pan[0] + ((pan[1]-pan[0])*x/steps),
                tilt[0] + ((tilt[1]-tilt[0])*y/steps)
                )
            while True:
                img = camera.read()
                if time.time() >= next_capture:
                    next_capture = time.time() + delay
                    cv2.imwrite('panorama/image-%02d-%02d.png' % (y, x), img)
                    break

        reverse = not reverse

    car.setPanTilt(0.0, 0.7)
    camera.stop()
