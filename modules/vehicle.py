#!/usr/bin/env python
"""Vehicle set to provide control and motion models for various types of
vehicles"""

import sys
import json
import time
import math
from threading import Thread

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


class Car(object):
    """Car type - uses a bicycle style motion model

    - motors for motion
    - servo for steering
    - pan/tilt camera

    Once start() is called, it will estimate it's position over time
    using Dead reckoning.
    """

    def __init__(self, pwm, GPIO, config={}, debug=False):
        """Create car type motion controller

        pwm = Sunfounder's PWM servo controller instance

        Config options:
            Vehicle control:

        - flip_direction :      list of motors who's direction is flipped
        - max_speed :           max speed for motor PWM controllers
        - max_rotation :        max PWM for steering servo; maximum
                                steering arc
        - rotation_offset :     change in default PWM for steering servo
                                0.0 for 'home' or straight

            Dead reckoning parameters:
        - speed_max_rpm:        the rpm of the motor when PWM is set to
                                speed_max
        - rotation_max_angle:   the steering angle when the wheel
                                rotation is set to rotation_max
        - vehicle_length        distance between the front wheel axle
                                and rear wheel axle
        - wheel_diameter        the diameter of the wheels
        """
        self.pwm = pwm
        self.GPIO = GPIO
        self.config = config
        self.debug = debug

        motor = config.get('motor', {})
        steering = config.get('steering', {})
        camera = config.get('camera', {})
        car = config.get('car', {})

        # Motor PWM pins
        self.motor_speed_controller = motor.get('pwm_speed_ctrl', [
            4,          # servo driver IC CH4
            5           # servo driver IC CH5
            ])

        # Motor GPIO pins to set HIGH when moving forward
        self.forward = motor.get('forward', [
            11,         # pin11 - motor 0
            13          # pin13 - motor 1
            ])
        # Motor GPIO pins to set HIGH when moving backwards
        self.backwards = motor.get('backwards', [
            12,         # pin12 - motor 0
            15          # pin15 - motor 1
            ])
        # Max value for PWM
        self.speed_max = motor.get('max_speed', 4000.0)
        self.speed = 0.0

        # Steering PWM pin
        self.steering_controller = steering.get('pwm_direction_ctrl', [
            0           # servo driver IC CH0
            ])
        self.rotation_max = steering.get('max_rotation', 75.0)
        # Default servo position for straight forward
        self.rotation_default = steering.get('rotation_offset', 450.0)
        self.rotation = 0.0

        # Pan/Tilt PWM pins
        self.pan_controller = camera.get('pwm_pan_ctrl', [
            14          # servo driver IC CH14
            ])
        self.tilt_controller = camera.get('pwm_tilt_ctrl', [
            15          # servo driver IC CH15
            ])
        self.pan_max = camera.get('max_pan', 250.0)
        self.tilt_max = camera.get('max_tilt', 250.0)
        # Default servo position for straight-up
        # NOTE: Facing forward is not the base position!
        self.pan_default = camera.get('pan_default', 450.0)
        self.tilt_default = camera.get('tilt_default', 450.0)
        # Default servo position for straight forward
        self.pan_home = camera.get('pan_home', 0.0)
        self.tilt_home = camera.get('tilt_home', 0.7)
        self.pan = 0.0
        self.tilt = 0.0

        # Most common tweaks to the default Settings
        for i in motor.get('flip_direction', []):
            # Flip direction of motor
            self._log("Flipping direction for motor %d" % i)
            self.forward[i], self.backwards[i] \
                = self.backwards[i], self.forward[i]
        self.rotation_default += steering.get('bias', 0.0)
        self.pan_default += camera.get('pan_bias', 0.0)
        self.tilt_default += camera.get('tilt_bias', 0.0)

        # List of all GPIO pins in use
        self.pins = self.forward + self.backwards

        # Set car's physical properties
        self.speed_rps = car.get('max_speed_rpm', 150.0) / 60.0
        self.rotation_max_angle = (
                2.0 * math.pi * car.get('max_rotation_angle', 45.0)
                ) / 360.0
        self.vehicle_length = car.get('vehicle_length', 20)
        self.wheel_circumference = car.get('wheel_diameter', 5.0) * math.pi

    def _log(self, message):
        if self.debug:
            sys.stderr.write(message + "\n")

    def start(self):
        self._log("Setting up pins for output: %r" % self.pins)
        # Set all pins' mode as output
        self.GPIO.setup(self.pins, self.GPIO.OUT)

        self.setSpeed(0.0)
        self.setPanTilt(self.pan_home, self.tilt_home)
        self.setDirection(0.0)

        """ start the thread to update position over time """
        self.running = True
        self.path = []
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        """ Sets actual speed / steering to desired values
        slowely. After which it estimates it's current position
        using Dead Reckoning.

        Dead Reckoning formula:
        if Steering ~ 0:
            # straight line
            Position x += cos(Orientation) * Velocity * Time
            Position y += sin(Orientation) * Velocity * Time
        else:
            Turn = tan(Steering) * (Velocity * Time) / Vehicle length
            Radius = (Velocity * Time) / Turn
            Position x -= sin(Orientation) * Radius
            Position y += cos(Orientation) * Radius
            Orientation = (Orientation + Turn) % 2pi
            Position x += sin(Orientation) * Radius
            Position y -= cos(Orientation) * Radius
        """
        self.dead_reckoning = {
            'P': [0., 0.], # Position
            'O': 0.0 # Orientation
            }
        last_time = time.time()
        clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
        time.sleep(0.01)
        last_position = self.getPosition()
        while self.running:
            last_time, delta_time = time.time(), time.time() - last_time

            self.dead_reckoning = self._DeadReckoning(
                self.dead_reckoning,
                delta_time
                )

            delta = self.getPosition(last_position)
            if delta[0]**2.0 + delta[1]**2.0 > 1.0:
                last_position = self.getPosition()
                self.path.append(last_position)

            self._setSpeed(
                self.speed + clamp(self.target_speed - self.speed, -1.0 * delta_time, 1.0 * delta_time)
                )
            self._setDirection(
                self.rotation + clamp(self.target_rotation - self.rotation, -2.0 * delta_time, 2.0 * delta_time)
                )

            time.sleep(0.01)

    def _DeadReckoning(self, dr, delta_T):
        radian = math.pi * 2.0
        distance = self.speed * self.speed_rps \
            * self.wheel_circumference * delta_T
        turn = math.tan(self.rotation * self.rotation_max_angle) \
            * distance / self.vehicle_length

        update = {
            'P': dr['P'],
            'O': dr['O']
            }
        if abs(turn) <= 0.0001:
            update['P'] = [
                dr['P'][0] + math.cos(dr['O']) * distance,
                dr['P'][1] + math.sin(dr['O']) * distance
                ]
            update['O'] = (dr['O'] + turn) % radian
        else:
            radius = distance / turn
            update['P'] = [
                dr['P'][0] - math.sin(dr['O']) * radius,
                dr['P'][1] + math.cos(dr['O']) * radius
                ]
            update['O'] = (dr['O'] + turn) % radian
            update['P'] = [
                update['P'][0] + math.sin(update['O']) * radius,
                update['P'][1] - math.cos(update['O']) * radius
                ]
        return update

    def getPosition(self, old_P=None):
        """ Get the current position. If a prior position P is provided,
        return a delta position
        """
        P = self.dead_reckoning['P']
        if old_P is not None:
            return [
                P[0] - old_P[0],
                P[1] - old_P[1]
                ]
        return P

    def stop(self):
        self.running = False
        self.setSpeed(0.0)
        self.setDirection(0.0)
        self._log("Releasing GPIO pins: %r" % self.pins)
        self.GPIO.cleanup(self.pins)

    def setSpeed(self, speed=0.0):
        self.target_speed = speed

    def _setSpeed(self, speed=0.0):
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
            self.speed = 0.0

        if speed:
            # Movement required. Set motor speed (always positive value)
            self._log("Set speed to %.2f = %d" % (
                speed, int(abs(speed) * self.speed_max)
                ))
            for speed_ctrl in self.motor_speed_controller:
                self.pwm.write(speed_ctrl, 0, int(abs(speed) * self.speed_max))

        if self.speed == 0.0 and speed > 0.0:
            # Positive speed; Move forward
            self._log("Set pins %r to %r" % (
                self.forward, self.GPIO.HIGH
                ))
            self.GPIO.output(self.forward, self.GPIO.HIGH)
        elif self.speed == 0.0 and speed < 0.0:
            # Negative speed; move backward
            self._log("Set pins %r to %r" % (
                self.backwards, self.GPIO.HIGH
                ))
            self.GPIO.output(self.backwards, self.GPIO.HIGH)

        self.speed = speed

    def setDirection(self, rotation=0.0):
        self.target_rotation = rotation

    def _setDirection(self, rotation=0.0):
        # Set rotation ranging from -1.0 to 1.0
        # Straight: rotation = 0.0
        # Left : rotation between -1.0 and 0.0
        # Right: rotation between 0.0 and 1.0

        # Rotation needs to be within -1.0 and 1.0 range
        assert(-1.0 <= rotation <= 1.0)

        self._log("Set rotation to %.2f = %d - %d" % (
            rotation,
            int(self.rotation_default),
            int(rotation * self.rotation_max)
            ))
        for direction_ctrl in self.steering_controller:
            self.pwm.write(
                direction_ctrl, 0,
                int(self.rotation_default + rotation * self.rotation_max)
                )

        self.rotation = rotation

    def setPanTilt(self, pan=0.0, tilt=0.0):
        # Set pan/tilt ranging from -1.0 to 1.0
        # Straight-up: pan, tilt = 0.0, 0.0
        # Assuming no pan:
        #   Forward: tilt between 0.0 and 1.0
        #   Backwards: tilt between -1.0 and 0.0
        # Assuming positive tilt:
        #   Left : pan between -1.0 and 0.0
        #   Right: pan between 0.0 and 1.0

        # Pan/Tilt needs to be within -1.0 and 1.0 range
        assert(-1.0 <= pan <= 1.0)
        assert(-1.0 <= tilt <= 1.0)

        self._log("Set pan/tilt to %.2f,%.2f = %d + %d, %d + %d" % (
            pan, tilt,
            int(pan * self.pan_max), int(self.pan_default),
            int(tilt * self.tilt_max), int(self.tilt_default)
            ))
        for pan_ctrl in self.pan_controller:
            self.pwm.write(
                pan_ctrl, 0,
                int(self.pan_default - pan * self.pan_max)
                )
        for tilt_ctrl in self.tilt_controller:
            self.pwm.write(
                tilt_ctrl, 0,
                int(self.tilt_default - tilt * self.tilt_max)
                )

        self.pan = pan
        self.tilt = tilt


if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD)    # Number GPIOs by its physical location
    pwm = PWM()                 # The servo controller.
    pwm.frequency = 60

    import os

    config = {}
    config_file = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', 'config.json'))
    if os.path.exists(config_file):
        with open(config_file, 'r') as config_json:
            config = json.load(config_json)

    car = Car(pwm, GPIO, config=config, debug=True)
    car.start()

    car.setPanTilt(0.0, 0.0)

    for state in [0.25, 0.5, 0.25, 0.0, -0.25, -0.5, -0.25, 0.0]:
        car.setSpeed(state)
        time.sleep(0.25)

    for state in [0.5, 1.0, 0.5, 0.0, -0.5, -1.0, -0.5, 0.0]:
        car.setDirection(state)
        time.sleep(0.25)

    for state in [0.5, 1.0, 0.5, 0.0, -0.5, -1.0, -0.5, 0.0]:
        car.setPanTilt(state, 0.0)
        time.sleep(0.25)

    for state in [0.5, 1.0, 0.5, 0.0, -0.5, -1.0, -0.5, 0.0]:
        car.setPanTilt(0.0, state)
        time.sleep(0.25)

    car.stop()
