#!/usr/bin/env python
"""Vehicle set to provide control and motion models for various types of
vehicles"""

import RPi.GPIO as GPIO
from sunfounder.PCA9685 import PWM
import time    # Import necessary modules

class Car:
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

    # Pins to set HIGH when moving forward
    forward = [
        11, # pin11 - motor 0
        12  # pin12 - motor 1
        ]
    # Pins to set HIGH when moving backwards
    backwards = [
        13, # pin13 - motor 0
        15  # pin15 - motor 1
        ]
    speed = 0.0
    #target_speed = 0.0
    #acceleration = 20.0
    #max_speed = 0.0

    def __init__(pwm, config={}):
        """Create car type motion controller

        pwm = Sunfounder's PWM servo controller instance

        config options:
        - flip_direction : list of motors who's direction is flipped

        - min_rotation : minimum rotation - defines maximum left arc
        - max_rotation : maximum rotation - defines maximum right arc
                         the average defines "straight forward"
        """

        for i in config.get('flip_direction', []):
            # Flip direction of motor
            self.forward[i], self.backwards[i] = self.backwards[i], self.forward[i]

        self.acceleration = config.get('acceleration', 20.0)
        self.max_speed = config.get('max_speed', 200.0)

        self.pins = self.motor_speed_controller \
            + self.forward \
            + self.backwards

        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)   # Set all pins' mode as output


    def setSpeed(self, speed=0.0):
        # Set movement speed ranging from -1.0 to 1.0
        # Stop: speed = 0.0
        # Forward: 0.0 > speed >= 1.0
        # Backwards: -1.0 <= speed < 0.0

        # Speed needs to be within -1.0 and 1.0 range
        assert(-1.0 <= speed <= 1.0)

        if not speed\
                or (self.speed > 0.0 and speed < 0.0) \
                or (self.speed < 0.0 and speed > 0.0):
            # Direction changed or stop command given
            # Halt motors and reset speed
            for speed_ctrl in self.motor_speed_controller:
                self.pwm.write(speed_ctrl, 0, 0.0)

            for motor in self.forward + self.backward:
                GPIO.output(motor, GPIO.LOW)

        if speed != 0.0:
            # Movement required. Set motor speed (always positive value)
            for i in self.motor_speed_controller:
                self.pwm.write(i, 0, abs(self.speed) * self.max_speed)

        if speed > 0.0:
            # Positive speed; Move forward
            for motor in self.forward:
                GPIO.output(motor, GPIO.LOW)
        elif speed < 0.0:
            # Negative speed; move backward
            for motor in self.backward:
                GPIO.output(motor, GPIO.HIGH)

        self.speed = speed


if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD) # Number GPIOs by its physical location
    pwm = PWM() # The servo controller.

    car = Car(pwm)

    for speed in [0.5, 1.0, 0.5, 0.0, -0.5, -1.0, -0.5, 0.0]:
        car.setSpeed(speed)
        time.sleep(5)


