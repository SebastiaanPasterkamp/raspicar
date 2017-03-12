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
                {'cfg': [], 'test': [[11, 13], [12, 15]]},
                {'cfg': [0], 'test': [[12, 13], [11, 15]]},
                {'cfg': [1], 'test': [[11, 15], [12, 13]]},
                {'cfg': [0,1], 'test': [[12, 15], [11, 13]]}
                ]:
            car = Car(self.pwm, self.GPIO, config={
                'flip_direction': case['cfg']
                })
            self.assertEqual(
                car.forward, case['test'][0], 'Forward: %r' % case['cfg']
                )

if __name__ == '__main__':
    unittest.main()
