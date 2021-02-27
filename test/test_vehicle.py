#!/usr/bin/env python
import unittest
from parameterized import parameterized

from .mockobjects import MockGPIO as GPIO, MockPWM as PWM
from modules.vehicle import Car


class TestCar(unittest.TestCase):

    def setUp(self):
        self.pwm = PWM()
        self.GPIO = GPIO()

    def tearDown(self):
        self.car.stop()

    @parameterized.expand([
        [[], [11, 13], [12, 15]],
        [[0], [12, 13], [11, 15]],
        [[1], [11, 15], [12, 13]],
        [[0, 1], [12, 15], [11, 13]],
        ])
    def test_motor_config(self, cfg, forward, backwards):
        self.car = Car(
            self.pwm,
            self.GPIO,
            config={'motor': {'flip_direction': cfg}},
            )
        self.assertEqual(
            self.car.forward,
            forward,
            f"Forward {cfg}: {self.car.forward} == {forward}",
            )
        self.assertEqual(
            self.car.backwards,
            backwards,
            f"Forward {cfg}: {self.car.backwards} == {backwards}",
            )

    @parameterized.expand([
        [{'rotation_offset': 100.0, 'bias': -10.0}, 90.0],
        [{'rotation_offset': 100.0}, 100.0],
        [{'rotation_offset': 100.0, 'bias': 0.0}, 100.0],
        [{'rotation_offset': 100.0, 'bias': 10.0}, 110.],
        ])
    def test_steering_config(self, cfg, expected):
        self.car = Car(self.pwm, self.GPIO, config={'steering': cfg})
        self.assertEqual(
            self.car.rotation_default,
            expected,
            f"Steering {cfg}: {self.car.rotation_default} == {expected}",
            )

    @parameterized.expand([
        [{'pan_default': 100.0, 'pan_bias': -10.0}, 90.0],
        [{'pan_default': 100.0}, 100.0],
        [{'pan_default': 100.0, 'pan_bias': 0.0}, 100.0],
        [{'pan_default': 100.0, 'pan_bias': 10.0}, 110.0],
        ])
    def test_camera_pan_config(self, cfg, expected):
        self.car = Car(self.pwm, self.GPIO, config={'camera': cfg})
        self.assertEqual(
            self.car.pan_default,
            expected,
            f"Camera pan {cfg}: {self.car.pan_default} == {expected}",
            )

    @parameterized.expand([
        [{'tilt_default': 100.0, 'tilt_bias': -10.0}, 90.0],
        [{'tilt_default': 100.0}, 100.0],
        [{'tilt_default': 100.0, 'tilt_bias': 0.0}, 100.0],
        [{'tilt_default': 100.0, 'tilt_bias': 10.0}, 110.0],
        ])
    def test_camera_tilt_config(self, cfg, expected):
        self.car = Car(self.pwm, self.GPIO, config={'camera': cfg})
        self.assertEqual(
            self.car.tilt_default,
            expected,
            f"Camera tilt {cfg}: {self.car.tilt_default} == {expected}",
            )

    def test_init(self):
        self.car = Car(self.pwm, self.GPIO)
        self.assertEqual(self.pwm.log, [])
        self.assertEqual(self.GPIO.log, [])

        self.car.start()

        # Pan/Tilt gets intitialized; speed and steering do not
        pantilt_channels = self.car.pan_controller \
            + self.car.tilt_controller
        for channel in pantilt_channels:
            self.assertTrue(any(
                log[0] == 'write' and log[1][0] == channel
                for log in self.pwm.log
                ), f"Channel {channel} has not received it's initial command.")

        log = [
            line for line in self.GPIO.log
            if line[0] == 'setup']
        self.assertEqual(log, [
            ['setup', ([11, 13, 12, 15], 'OUT'), {}],
            ], "Wheel direction pins should be intialized.")

    def test_speed(self):
        self.car = Car(self.pwm, self.GPIO)
        self.car.start()
        # Kill the thread; put the car in manual
        self.car.running = False

        self.GPIO.mock_reset_log()
        self.pwm.mock_reset_log()

        self.car.setSpeed(-1)
        self.car._update(self.car.getPosition(), 0.1)

        pwm_log = [
            line for line in self.pwm.log
            if line[1][0] in self.car.motor_speed_controller
            ]
        self.assertEqual(
            pwm_log,
            [
                ['write', (4, 0, 400), {}],
                ['write', (5, 0, 400), {}],
                ],
            f"Motor on {self.car.motor_speed_controller} are running now")

        gpio_log = [
            line for line in self.GPIO.log
            if line[0] in ['setup', 'output']]
        self.assertEqual(
            gpio_log,
            [['output', ([12, 15], 'HIGH'), {}]],
            "Backwards drive should be enabled")

        self.GPIO.mock_reset_log()
        self.pwm.mock_reset_log()

        self.car.setSpeed(.5)
        self.car._update(self.car.getPosition(), 0.1)

        pwm_log = [
            line for line in self.pwm.log
            if line[1][0] in self.car.motor_speed_controller
            ]
        self.assertEqual(
            pwm_log,
            [
                ['write', (4, 0, 0), {}],
                ['write', (5, 0, 0), {}],
                ],
            f"Motor on {self.car.motor_speed_controller} should stop first")

        gpio_log = [
            line for line in self.GPIO.log
            if line[0] in ['setup', 'output']]
        self.assertEqual(
            gpio_log,
            [['output', ([11, 13, 12, 15], 'LOW'), {}]],
            "Any direction drive should be disabled first")

        self.GPIO.mock_reset_log()
        self.pwm.mock_reset_log()
        self.car._update(self.car.getPosition(), 0.1)

        pwm_log = [
            line for line in self.pwm.log
            if line[1][0] in self.car.motor_speed_controller
            ]
        self.assertEqual(
            pwm_log,
            [
                ['write', (4, 0, 400), {}],
                ['write', (5, 0, 400), {}],
                ],
            f"Motor on {self.car.motor_speed_controller} are running again")

        gpio_log = [
            line for line in self.GPIO.log
            if line[0] in ['setup', 'output']]
        self.assertEqual(
            gpio_log,
            [['output', ([11, 13], 'HIGH'), {}]],
            "Forward drive should be enabled")

        self.GPIO.mock_reset_log()
        self.pwm.mock_reset_log()

        self.car.setSpeed(1)
        self.car._update(self.car.getPosition(), 0.1)

        pwm_log = [
            line for line in self.pwm.log
            if line[1][0] in self.car.motor_speed_controller
            ]
        self.assertEqual(
            pwm_log,
            [
                ['write', (4, 0, 800), {}],
                ['write', (5, 0, 800), {}],
                ],
            f"Motor on {self.car.motor_speed_controller} should go faster")

        gpio_log = [
            line for line in self.GPIO.log
            if line[0] in ['setup', 'output']]
        self.assertEqual(
            gpio_log,
            [],
            "Direction should not have changed")


if __name__ == '__main__':
    unittest.main(verbosity=2)
