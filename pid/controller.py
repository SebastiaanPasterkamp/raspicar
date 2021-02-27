"""
PID Controller class
"""

import time


class PIDController(object):

    def __init__(self, Kp, Ki, Kd, limit_integral=(None, None),
                 limit_output=(None, None)):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.limit_integral = limit_integral
        self.limit_output = limit_output

        self._last_time = None
        self._desired = 0.0
        self._proportional = 0.0
        self._integral = 0.0
        self._derivative = 0.0
        self._current = None

    @property
    def p(self):
        return self._proportional

    @property
    def i(self):
        return self._integral

    @property
    def d(self):
        return self._derivative

    def setDesired(self, desired):
        self._desired = desired

    def getOutput(self, current, dt=None):
        current_time = time.monotonic()
        delta_time = 0.0
        if dt is not None:
            delta_time = dt
        elif self._last_time is not None:
            delta_time = current_time - self._last_time

        error = self._desired - current
        self._proportional = self.Kp * error
        self._integral = self._clamp_integral(
            self._integral + self.Ki * error * delta_time)
        self._derivative = self.Kd * (self._current - current) / delta_time \
            if self._current is not None else 0.0

        self._current = current
        self._last_time = current_time

        return self._clamp_output(
            self._proportional + self._integral + self._derivative)

    def _clamp_integral(self, integral):
        return self._clamp(integral, *self.limit_integral)

    def _clamp_output(self, output):
        return self._clamp(output, *self.limit_output)

    def _clamp(self, value, min, max):
        if min is not None and value <= min:
            return min
        if max is not None and value >= max:
            return max
        return value
