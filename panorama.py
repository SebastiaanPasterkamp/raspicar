#!/usr/bin/env python

from argparse import ArgumentParser
import os
import cv2
import time
import json

from modules.camera import (
    VideoCamera,
    Calibration,
    parseArguments as parseCameraArguments,
    )
from modules.vehicle import Car
from rotation.matrix import rotationMatrixToEulerAngles as matrixToEuler


usePiCamera = False
try:
    import RPi.GPIO as GPIO
    from sunfounder.PCA9685 import PWM
    usePiCamera = True
except ImportError as e:
    print("Not running on a Raspberry Pi", e)
    from test.mockobjects import MockGPIO, MockPWM as PWM
    GPIO = MockGPIO()


def panorama(config, car, camera, calib):
    panorama = config.get("panorama", {})
    steps = panorama.get("steps", 6)
    pan = panorama.get("pan", (-.2, .2))
    tilt = panorama.get("tilt", (.6, .8))
    delay = panorama.get("delay", 3.0)
    directory = panorama.get("directory", "panorama")

    os.makedirs(directory, mode=0o750, exist_ok=True)

    next_capture = time.time() + delay
    reverse = False
    for y in range(steps+1):
        for x in sorted(range(steps+1), reverse=reverse):
            try:
                filename = os.path.join(
                    directory, f'image-{x:02d}-{y:02d}.png')
                p, t = (
                    pan[0] + ((pan[1]-pan[0])*x/steps),
                    tilt[0] + ((tilt[1]-tilt[0])*y/steps),
                    )
                car.setPanTilt(p, t)
                duration = max(0, next_capture - time.time())
                print(f"{x},{y} [{p:.2f}, {t:.2f}]: Filename: {filename}")
                time.sleep(duration)
                img = camera.read()
                cv2.imwrite(filename, img)
                next_capture = time.time() + delay

                status, corners = calib.findGrid(img)
                if status:
                    try:
                        status, rvecs, tvecs, _ = \
                            calib.getPose(corners, camera)
                    except cv2.error as e:
                        print(e)
                        pass

                if status:
                    try:
                        print("PnP:")
                        print(f" - Rotation: {rvecs}")
                        print(f" - Translation: {tvecs}")
                        print(f" - Anlgles {matrixToEuler(rvecs)}")
                    except AssertionError as e:
                        print(e)
                        pass
            except KeyboardInterrupt:
                return

        reverse = not reverse


def loadConfig(filename):
    config = {}
    with open(filename, 'r') as fh:
        config = json.load(fh)
    return config


def parse_args(config, add_help=True):
    ap = ArgumentParser(add_help=add_help)
    if not add_help:
        ap.add_argument(
            "-h", "--help", default=False, action='store_true',
            help="Show this help message and exit")

    parseCameraArguments(ap, config)
    Calibration.parseArguments(ap, config)

    ap.add_argument(
        "-c", "--config", metavar="file.json", default="config.json",
        help="Config file to use. [default: %(default)s]")
    ap.add_argument(
        "-d", "--debug", default=False, action='store_true',
        help="Enable debugging by displaying the results "
        "[default: %(default)s]")

    return ap.parse_args(), ap


if __name__ == "__main__":
    config = dict()
    args, ap = parse_args(config, False)
    if os.path.exists(args.config):
        # reparse args with defaults from config
        config = loadConfig(args.config)
    args, ap = parse_args(config)

    args.grid = tuple([int(g) for g in args.grid.split(",")])
    args.size = tuple([float(s) for s in args.size.split(",")])
    args.resolution = tuple([int(s) for s in args.resolution.split(",")])

    GPIO.setmode(GPIO.BOARD)    # Number GPIOs by its physical location
    pwm = PWM()                 # The servo controller.
    pwm.frequency = 60

    car = Car(pwm, GPIO, config=config)
    car.start()

    camera = VideoCamera(
        src=args.index,
        patterns=args.patterns,
        resolution=args.resolution,
        framerate=args.framerate,
        usePiCamera=usePiCamera,
        )
    if args.npz is not None and os.path.exists(args.npz):
        camera.loadCalibration(args.npz)
    camera.start()

    calib = Calibration(args.pattern, args.grid, args.size)

    panorama(config, car, camera, calib)

    car.setPanTilt(0.0, 0.7)
    car.stop()
    camera.stop()
