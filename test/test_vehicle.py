#!/usr/bin/env python
import unittest

import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.realpath(__file__)), '..'
    ))

from .mockobjects import MockGPIO as GPIO, MockPWM as PWM
from modules.vehicle import Car


class TestCar(unittest.TestCase):

    def setUp(self):
        self.pwm = PWM()
        self.GPIO = GPIO()

    def test_motor_config(self):
        for case in [
                {'cfg': [], 'forward': [11, 13], 'backwards': [12, 15]},
                {'cfg': [0], 'forward': [12, 13], 'backwards': [11, 15]},
                {'cfg': [1], 'forward': [11, 15], 'backwards': [12, 13]},
                {'cfg': [0, 1], 'forward': [12, 15], 'backwards': [11, 13]}
                ]:
            car = Car(self.pwm, self.GPIO, config={
                'motor': {'flip_direction': case['cfg']}
                })
            self.assertEqual(
                car.forward, case['forward'],
                'Forward %r: %r == %r' % (
                    case['cfg'], car.forward, case['forward']
                    )
                )
            self.assertEqual(
                car.backwards, case['backwards'],
                'Forward %r: %r == %r' % (
                    case['cfg'], car.backwards, case['backwards']
                    )
                )

    def test_steering_config(self):
        for case in [
                {'cfg': {'default': 100.0, 'bias': -10.0},
                 'expected': 90.0},
                {'cfg': {'default': 100.0},
                 'expected': 100.0},
                {'cfg': {'default': 100.0, 'bias': 0.0},
                 'expected': 100.0},
                {'cfg': {'default': 100.0, 'bias': 10.0},
                 'expected': 110.0}
                ]:
            car = Car(self.pwm, self.GPIO, config={
                'steering': case['cfg']
                })
            self.assertEqual(
                car.rotation_default, case['expected'],
                'Steering %r: %r == %r' % (
                    case['cfg'], car.rotation_default, case['expected']
                    )
                )

    def test_camera_config(self):
        for case in [
                {'cfg': {'pan_default': 100.0, 'pan_bias': -10.0},
                 'expected': 90.0},
                {'cfg': {'pan_default': 100.0},
                 'expected': 100.0},
                {'cfg': {'pan_default': 100.0, 'pan_bias': 0.0},
                 'expected': 100.0},
                {'cfg': {'pan_default': 100.0, 'pan_bias': 10.0},
                 'expected': 110.0}
                ]:
            car = Car(self.pwm, self.GPIO, config={
                'camera': case['cfg']
                })
            self.assertEqual(
                car.pan_default, case['expected'],
                'Camera pan %r: %r == %r' % (
                    case['cfg'], car.pan_default, case['expected']
                    )
                )

        for case in [
                {'cfg': {'tilt_default': 100.0, 'tilt_bias': -10.0},
                 'expected': 90.0},
                {'cfg': {'tilt_default': 100.0},
                 'expected': 100.0},
                {'cfg': {'tilt_default': 100.0, 'tilt_bias': 0.0},
                 'expected': 100.0},
                {'cfg': {'tilt_default': 100.0, 'tilt_bias': 10.0},
                 'expected': 110.0}
                ]:
            car = Car(self.pwm, self.GPIO, config={
                'camera': case['cfg']
                })
            self.assertEqual(
                car.tilt_default, case['expected'],
                'Camera tilt %r: %r == %r' % (
                    case['cfg'], car.tilt_default, case['expected']
                    )
                )

    def test_init(self):
        car = Car(self.pwm, self.GPIO)
        self.assertEqual(
            self.pwm.log, []
            )
        self.assertEqual(
            self.GPIO.log, []
            )

        car.start()
        configured_channels = [0, 4, 5, 14, 15]
        self.assertEqual([
            channel
            for channel in configured_channels
            if any([
                log[0] == 'write' and log[1][0] == channel
                for log in self.pwm.log
                ])
            ],
            configured_channels
            )
        gpio_setup = [
            [log[0], log[1][1]]
            for log in self.GPIO.log
            if log[0] in ['setup', 'output']
            ]
        self.assertEqual(gpio_setup, [
            ['setup', 'OUT'],
            ['output', 'LOW']
            ])

    def test_speed(self):
        car = Car(self.pwm, self.GPIO)
        car.start()

        self.GPIO.mock_reset_log()
        self.pwm.mock_reset_log()

        car.setSpeed(-1)
        self.assertEqual(
            len([
                log
                for log in self.pwm.log
                if log[0] == 'write' and log[1][2] > 0.0
                ]),
            2, "2 motors should start running")
        self.assertEqual(
            [
                [log[0], log[1][1]]
                for log in self.GPIO.log
                if log[0] in ['setup', 'output']
            ], [
                ['output', 'HIGH']
            ], "No more setup, only activate motors")

        self.GPIO.mock_reset_log()
        self.pwm.mock_reset_log()

        car.setSpeed(.5)
        self.assertEqual(
            len([
                log
                for log in self.pwm.log
                if log[0] == 'write' and log[1][2] == 0.0
                ]),
            2, "motors should stop before reversing direction")
        self.assertEqual(
            len([
                log
                for log in self.pwm.log
                if log[0] == 'write' and log[1][2] > 0.0
                ]),
            2, "2 motors should start running again")
        self.assertEqual(
            [
                [log[0], log[1][1]]
                for log in self.GPIO.log
                if log[0] in ['setup', 'output']
            ], [
                ['output', 'LOW'],
                ['output', 'HIGH']
            ], "Stop forward motion, start backwards motion")

        self.GPIO.mock_reset_log()
        self.pwm.mock_reset_log()

        car.setSpeed(1)
        self.assertEqual(
            len([
                log
                for log in self.pwm.log
                if log[0] == 'write' and log[1][2] == 0.0
                ]),
            0, "same direction; motors don't have to stop")
        self.assertEqual(
            len([
                log
                for log in self.pwm.log
                if log[0] == 'write' and log[1][2] > 0.0
                ]),
            2, "2 motors should be running on different speed")
        self.assertEqual(
            [
                [log[0], log[1][1]]
                for log in self.GPIO.log
                if log[0] in ['setup', 'output']
            ], [
            ], "No motors switched")

if __name__ == '__main__':
    unittest.main(verbosity=2)
