#!/bin/env python

import sys
import os
import cv
import time
import json

try:
    import RPi.GPIO as GPIO
    from sunfounder.PCA9685 import PWM
except:
    import os
    sys.path.insert(0, os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..'
        )))

    from test.mockobjects import MockGPIO, MockPWM as PWM
    GPIO = MockGPIO()

from modules.vehicle import Car

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

def getFrame(filename):
    capture = cv.CaptureFromCAM(-1)
    cv.SetCaptureProperty(capture,cv.CV_CAP_PROP_FRAME_WIDTH, 640)
    cv.SetCaptureProperty(capture,cv.CV_CAP_PROP_FRAME_HEIGHT, 480);
    img = cv.QueryFrame(capture)
    cv.SaveImage(filename, img)

if __name__ == "__main__":
    steps = 10
    pan = (-.2,.2)
    tilt = (.5,.7)

    car.setPanTilt(pan[0], tilt[0])
    time.sleep(1)

    reverse=False
    for y in range(steps+1):
        for x in sorted(range(steps+1), reverse=reverse):
            car.setPanTilt(
                pan[0] + ((pan[1]-pan[0])*x/steps),
                tilt[0] + ((tilt[1]-tilt[0])*y/steps)
                )
            time.sleep(0.75)
            getFrame('panorama/image-%02d-%02d.png' % (y, x))
        reverse = not reverse

    car.setPanTilt(0.0, 0.7)
