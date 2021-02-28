#!/usr/bin/env python

from argparse import ArgumentParser
import cv2
import json
import os

from modules.camera import (
    VideoCamera,
    Calibration,
    parseArguments as parseCameraArguments,
    )
from rotation.matrix import rotationMatrixToEulerAngles


def main(patterns, resolution, minimum, pattern, grid, size, output, debug):
    calib = Calibration(pattern, grid, size)

    camera = VideoCamera(patterns=patterns, resolution=resolution)
    image = camera.read()
    while image is not None:
        calib.addImage(image)

        status, corners = calib.findGrid(image)
        if debug:
            # Draw and display the corners
            cv2.drawChessboardCorners(image, grid, corners, status)
            cv2.imshow('cam', image)
            cv2.waitKey(1)

        image = camera.read()

    if debug:
        cv2.destroyAllWindows()

    calib.calibrate(minimum, output)

    camera = VideoCamera(patterns=patterns, resolution=resolution)
    camera.loadCalibration(output)
    image = camera.read()
    while image is not None:
        calib.addImage(image)

        status, corners = calib.findGrid(image)
        if not status:
            image = camera.read()
            continue

        try:
            print(calib.grid, corners)
            retval, rvecs, tvecs, _ = calib.getPose(corners, camera)
        except cv2.error as e:
            print(e)
            image = camera.read()
            continue

        try:
            print("          PnP:", retval)
            print("     Rotation:", rvecs)
            print("  Translation:", tvecs)
            print("Euler Anlgles:", rotationMatrixToEulerAngles(rvecs))
        except AssertionError as e:
            print(e)
            pass

        image = camera.read()


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


def loadConfig(filename):
    config = {}
    with open(filename, 'r') as fh:
        config = json.load(fh)
    return config


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

    main(
        patterns=args.patterns,
        resolution=args.resolution,
        minimum=args.minimum,
        pattern=args.pattern,
        grid=args.grid,
        size=args.size,
        output=args.npz,
        debug=args.debug,
        )
