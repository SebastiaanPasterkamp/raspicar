#!/usr/bin/env python
import unittest
from parameterized import parameterized

import numpy as np
from math import pi

from .matrix import (
    eulerAnglesToRotationMatrix,
    isRotationMatrix,
    rotationMatrixToEulerAngles,
    )


class TestRotationMatrixConversion(unittest.TestCase):

    @parameterized.expand([
        [(0., 0., 0.),       [[1., 0., 0.],  [0., 1., 0.],  [0., 0., 1.]]],
        [(pi/2.0, 0., 0.),   [[1., 0., 0.], [0., 0., -1.],  [0., 1., 0.]]],
        [(pi/-2.0, 0., 0.),  [[1., 0., 0.],  [0., 0., 1.], [0., -1., 0.]]],
        [(0., pi/2.0, 0.),   [[0., 0., 1.],  [0., 1., 0.],  [-1., 0, 0.]]],
        [(0., pi/-2.0, 0.), [[0., 0., -1.],  [0., 1., 0.],  [1., 0., 0.]]],
        ])
    def test_EulerAnglesToRotationMatrix(self, theta, expected):
        result = eulerAnglesToRotationMatrix(*theta)
        self.assertIsNone(np.testing.assert_allclose(
            expected,
            result,
            rtol=1e-10,
            atol=1e-10,
            ))

    @parameterized.expand([
        [[[1., 0., 0.],  [0., 1., 0.],  [0., 0., 1.]], True],
        [[[1., 0., 0.],  [0., 0., 1.],  [0., 1., 0.]], True],
        [[[9., 0., 0.],  [0., 1., 0.],  [0., 0., 1.]], False],
        ])
    def test_IsRotationMatrix(self, matrix, expected):
        result = isRotationMatrix(np.array(matrix))
        self.assertEqual(expected, result)

    @parameterized.expand([
        [[[1., 0., 0.],  [0., 1., 0.],  [0., 0., 1.]],      (0., 0., 0.)],
        [[[1., 0., 0.], [0., 0., -1.],  [0., 1., 0.]],  (pi/2.0, 0., 0.)],
        [[[1., 0., 0.],  [0., 0., 1.], [0., -1., 0.]], (pi/-2.0, 0., 0.)],
        [[[0., 0., 1.],  [0., 1., 0.],  [-1., 0, 0.]],  (0., pi/2.0, 0.)],
        [[[0., 0., -1.], [0., 1., 0.],  [1., 0., 0.]], (0., pi/-2.0, 0.)],
        ])
    def test_RotationMatrixToEulerAngles(self, matrix, expected):
        result = rotationMatrixToEulerAngles(np.array(matrix))
        self.assertIsNone(np.testing.assert_allclose(
            expected,
            result,
            rtol=1e-10,
            atol=1e-10,
            ))
