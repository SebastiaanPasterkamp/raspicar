# From https://learnopencv.com/rotation-matrix-to-euler-angles/
import numpy as np
from math import atan2, cos, sin, sqrt


# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(x, y, z):
    R_x = np.array([
        [1,      0,       0],
        [0, cos(x), -sin(x)],
        [0, sin(x),  cos(x)],
        ])

    R_y = np.array([
        [cos(y),  0, sin(y)],
        [0,       1,      0],
        [-sin(y), 0, cos(y)],
        ])

    R_z = np.array([
        [cos(z), -sin(z), 0],
        [sin(z),  cos(z), 0],
        [0,            0, 1],
        ])

    return np.dot(R_z, np.dot(R_y, R_x))


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    i = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(i - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert(isRotationMatrix(R))

    sy = sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if singular:
        x = atan2(-R[1, 2], R[1, 1])
        y = atan2(-R[2, 0], sy)
        z = 0
    else:
        x = atan2(R[2, 1], R[2, 2])
        y = atan2(-R[2, 0], sy)
        z = atan2(R[1, 0], R[0, 0])

    return np.array([x, y, z])
