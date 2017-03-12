#!/usr/bin/env python
import unittest

import os
import sys

sys.path.append(os.path.join(
    os.path.dirname(os.path.realpath(__file__)), '..'
    ))

from modules.vehicle import Car

class MockPWM(object):
    def __init__(self):
        self.command_log = []

class MockGPIO(object):
    def __init__(self):
        self.command_log = []

class TestCar(unittest.TestCase):
    
    def setUp(self):
        self.pwm = MockPWM()
        self.GPIO = MockGPIO()
    
    def test_motor_config(self):
        for case in [
                {'cfg': [], 'forward': [11, 13], 'backwards': [12, 15]},
                {'cfg': [0], 'forward': [12, 13], 'backwards': [11, 15]},
                {'cfg': [1], 'forward': [11, 15], 'backwards': [12, 13]},
                {'cfg': [0,1], 'forward': [12, 15], 'backwards': [11, 13]}
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
                {'cfg': {'default': 100.0, 'bias': -10.0}, 'expected': 90.0},
                {'cfg': {'default': 100.0 }, 'expected': 100.0},
                {'cfg': {'default': 100.0, 'bias': 0.0}, 'expected': 100.0},
                {'cfg': {'default': 100.0, 'bias': 10.0}, 'expected': 110.0}
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
                {'cfg': {'pan_default': 100.0, 'pan_bias': -10.0}, 'expected': 90.0},
                {'cfg': {'pan_default': 100.0 }, 'expected': 100.0},
                {'cfg': {'pan_default': 100.0, 'pan_bias': 0.0}, 'expected': 100.0},
                {'cfg': {'pan_default': 100.0, 'pan_bias': 10.0}, 'expected': 110.0}
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
                {'cfg': {'tilt_default': 100.0, 'tilt_bias': -10.0}, 'expected': 90.0},
                {'cfg': {'tilt_default': 100.0 }, 'expected': 100.0},
                {'cfg': {'tilt_default': 100.0, 'tilt_bias': 0.0}, 'expected': 100.0},
                {'cfg': {'tilt_default': 100.0, 'tilt_bias': 10.0}, 'expected': 110.0}
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

if __name__ == '__main__':
    unittest.main(verbosity=2)
