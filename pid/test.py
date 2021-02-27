#!/usr/bin/env python
import unittest
from parameterized import parameterized

from .controller import PIDController


class TestPIDController(unittest.TestCase):

    @parameterized.expand([
        [(1, 2, 3), 1, 2, 3, (None, None), (None, None)],
        [(4, 5, 6, (-1, 1), (-5, 5)), 4, 5, 6, (-1, 1), (-5, 5)],
        ])
    def test_init(self, init, Kp, Ki, Kd, lim_i, lim_o):
        pid = PIDController(*init)
        self.assertEqual(pid.Kp, Kp)
        self.assertEqual(pid.Ki, Ki)
        self.assertEqual(pid.Kd, Kd)
        self.assertEqual(pid.limit_integral, lim_i)
        self.assertEqual(pid.limit_output, lim_o)

    @parameterized.expand([
        [0, (0., 0.), 0],
        [0, (None, None), 0],
        [10, (0., 0.), 0],
        [10, (None, None), 10],
        [-10, (None, None), -10],
        [5, (-10., 10.), 5],
        [-5, (-10., 10.), -5],
        [10, (-5., 5.), 5],
        [-10, (-5., 5.), -5],
        [10, (None, 5.), 5],
        [-10, (-5., None), -5],
        [10, (-5., None), 10],
        [-10, (None, 5.), -10],
        ])
    def test_clamp(self, input, limits, expected):
        pid = PIDController(0, 0, 0)
        self.assertEqual(pid._clamp(input, *limits), expected)

    @parameterized.expand([
        [1.0, (None, None), 10, 5, [5, 0]],
        [0.1, (None, None), 10, 5, [0.5, 0.45]],
        [0.1, (None, None), -10, -5, [-0.5, -0.45]],
        [0.1, (None, None), 10, 10, [0.0, 0.0]],
        [1.0, (-0.1, 0.1), 10, 5, [0.1, 0.1]],
        ])
    def test_Kp(self, Kp, limit, desired, current, expectations):
        pid = PIDController(Kp, 0, 0, limit_output=limit)
        pid.setDesired(desired)
        for expected in expectations:
            output = pid.getOutput(current, dt=0.1)
            self.assertAlmostEqual(output, expected, places=5)
            current += output

    @parameterized.expand([
        [1.0, (None, None), 10, 5, [0.5, 0.95]],
        [0.1, (None, None), 10, 5, [0.05, 0.0995]],
        [0.1, (None, None), -10, -5, [-0.05, -0.0995]],
        [0.1, (None, None), 10, 10, [0.0, 0.0]],
        [1.0, (-0.1, 0.1), 10, 5, [0.1, 0.1]],
        ])
    def test_Ki(self, Ki, limit, desired, current, expectations):
        pid = PIDController(0, Ki, 0, limit_integral=limit)
        pid.setDesired(desired)
        for expected in expectations:
            output = pid.getOutput(current, dt=0.1)
            self.assertAlmostEqual(output, expected, places=5)
            current += output

    @parameterized.expand([
        [0.1, 10, 5, 1, [0.0, -1.0, 0.0, -1.0]],
        [0.01, 10, 5, 1, [0.0, -0.1, -0.09, -0.091]],
        [0.01, -10, -5, -1, [0.0, 0.1, 0.09, 0.091]],
        [0.1, 10, 10, 0, [0.0, 0.0, 0.0]],
        ])
    def test_Kd(self, Kd, desired, current, delta, expectations):
        pid = PIDController(0, 0, Kd)
        pid.setDesired(desired)
        for expected in expectations:
            output = pid.getOutput(current, dt=0.1)
            self.assertAlmostEqual(output, expected, places=5)
            current += delta + output
