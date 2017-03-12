#!/usr/bin/env python
"""Vehicle set to provide control and motion models for various types of
vehicles"""

import os
import sys
import json

import RPi.GPIO as GPIO
from sunfounder.PCA9685 import PWM
import time    # Import necessary modules

class Car(object):
    """Car type - uses a bicycle style motion model

    - 2 motors for motion (engine direction can be flipped in config)
    - 1 servo for steering
    """

    # List of all GPIO pins
    pins = []

    # Motor PWM pins
    motor_speed_controller = [
        4,  # servo driver IC CH4
        5   # servo driver IC CH5
        ]

    # Motor GPIO pins to set HIGH when moving forward
    forward = [
        11, # pin11 - motor 0
        13  # pin13 - motor 1
        ]
    # Motor GPIO pins to set HIGH when moving backwards
    backwards = [
        12, # pin12 - motor 0
        15  # pin15 - motor 1
        ]
    speed = 0.0
    speed_max = 4000.0 # Max value for PWM

    # Steering PWM pin
    steering_controller = [
        0 # servo driver IC CH0
        ]
    rotation = 0.0
    rotation_max = 75.0
    rotation_offset = 450.0 # Default servo position for 'home' or straight


    def __init__(self, pwm, GPIO, config={}, debug=False):
        """Create car type motion controller

        pwm = Sunfounder's PWM servo controller instance

        config options:
        - flip_direction :  list of motors who's direction is flipped
        - speed_max:        maximum speed for motor PWM controllers
        - rotation_max :    maximum PWM for steering servo; maximum steering arc
        - rotation_offset : change in default PWM for steering servo
                            0.0 for 'home' or straight
        """

        self.pwm = pwm
        self.GPIO = GPIO
        self.config = config
        self.debug = debug

        for i in config.get('flip_direction', []):
            # Flip direction of motor
            self._log("Flipping direction for motor %d" % i)
            self.forward[i], self.backwards[i] = self.backwards[i], self.forward[i]
        self.speed_max = config.get('speed_max', 4000.0)

        self.rotation_max = config.get('rotation_max', 50.0)
        self.rotation_offset += config.get('rotation_offset', 0.0)


    def start(self):
        self.pins = self.forward + self.backwards
        self._log("Setting up pins for output: %r" % self.pins)
        self.GPIO.setup(self.pins, self.GPIO.OUT)   # Set all pins' mode as output

        self.setSpeed(0.0)
        self.setDirection(0.0)


    def _log(self, message):
        if self.debug:
            sys.stderr.write(message + "\n")


    def stop(self):
        self.setSpeed(0.0)
        self.setDirection(0.0)
        self._log("Releasing GPIO pins: %r" % self.pins)
        self.GPIO.cleanup(self.pins)


    def setSpeed(self, speed=0.0):
        # Set movement speed ranging from -1.0 to 1.0
        # Stop: speed = 0.0
        # Forward: 0.0 > speed >= 1.0
        # Backwards: -1.0 <= speed < 0.0

        # Speed needs to be within -1.0 and 1.0 range
        assert(-1.0 <= speed <= 1.0)

        if not speed \
                or (self.speed > 0.0 and speed < 0.0) \
                or (self.speed < 0.0 and speed > 0.0):
            # Direction changed or stop command given
            # Halt motors and reset speed
            self._log("Dir changed: %.2f -> %.2f." % (self.speed, speed))
            for speed_ctrl in self.motor_speed_controller:
                self.pwm.write(speed_ctrl, 0, 0)
            self._log("Set pins %r to %r" % (
                self.forward + self.backwards, self.GPIO.LOW
                ))
            self.GPIO.output(
                self.forward + self.backwards, self.GPIO.LOW)

        if speed:
            # Movement required. Set motor speed (always positive value)
            self._log("Set speed to %.2f = %d" % (
                speed, int(abs(speed) * self.speed_max)
                ))
            for speed_ctrl in self.motor_speed_controller:
                self.pwm.write(speed_ctrl, 0, int(abs(speed) * self.speed_max))

        if speed > 0.0:
            # Positive speed; Move forward
            self._log("Set pins %r to %r" % (
                self.forward, self.GPIO.HIGH
                ))
            self.GPIO.output(self.forward, self.GPIO.HIGH)
        elif speed < 0.0:
            # Negative speed; move backward
            self._log("Set pins %r to %r" % (
                self.backwards, self.GPIO.HIGH
                ))
            self.GPIO.output(self.backwards, self.GPIO.HIGH)

        self.speed = speed

    def setDirection(self, rotation=0.0):
        # Set rotation ranging from -1.0 to 1.0
        # Straight: rotation = 0.0
        # Right: 0.0 > rotation >= 1.0
        # Left: -1.0 <= rotation < 0.0

        # Rotation needs to be within -1.0 and 1.0 range
        assert(-1.0 <= rotation <= 1.0)

        self._log("Set rotation to %.2f = %d + %d" % (
            rotation, int(rotation * self.rotation_max), int(self.rotation_offset)))
        for direction_ctrl in self.steering_controller:
            self.pwm.write(direction_ctrl, 0, int(rotation * self.rotation_max + self.rotation_offset))

        self.rotation = rotation



if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD) # Number GPIOs by its physical location
    pwm = PWM() # The servo controller.
    pwm.frequency = 60

    config = {}
    if os.path.exists('config.json'):
        with open('config.json', 'r') as config_json:
            config = json.load(config_json)

    car = Car(pwm, GPIO, config=config, debug=True)
    car.start()

    for state in [0.5, 1.0, 0.5, 0.0, -0.5, -1.0, -0.5, 0.0]:
        car.setSpeed(state)
        car.setDirection(state)
        time.sleep(0.5)

    car.stop()

