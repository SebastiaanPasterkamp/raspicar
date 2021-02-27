#!/usr/bin/env python

from argparse import ArgumentParser
import cv2
import json
import os

from modules.camera import VideoCamera, Calibration


def main(examples, resolution, minimum, pattern, grid, size, output, debug):
    calib = Calibration(pattern, grid, size)

    camera = VideoCamera(patterns=examples, resolution=resolution)
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


def parse_args(config, add_help=True):
    camera = config.get("camera", {})
    res = camera.get("resolution", {})
    calib = config.get("calibration", {})
    grid = calib.get("grid", {})
    size = calib.get("size", {})

    ap = ArgumentParser(add_help=add_help)
    if not add_help:
        ap.add_argument(
            "-h", "--help", default=False, action='store_true',
            help="Show this help message and exit")
    ap.add_argument(
        "-m", "--minimum", dest="minimum", type=int,
        default=calib.get("minimum", 10),
        help="Minimum number of examples to calibrate with. "
        "[default: %(default)d]")
    ap.add_argument(
        "-e", "--examples", default=[], dest="examples",
        action='append', metavar="GLOB",
        help="Use these examples (glob pattern). [default: %(default)s]")
    ap.add_argument(
        "-p", "--pattern", dest="pattern",
        default=calib.get("pattern", "asymetric"),
        choices=["chess", "circles", "asymetric"],
        help="Type of calibration pattern used. [default: %(default)s]")
    ap.add_argument(
        "-g", "--grid", dest="grid", metavar="rows,columns",
        default="%d,%d" % (grid.get("rows", 4), grid.get("columns", 11)),
        help="Grid size of the example pattern in number of corners / dots. "
        "[default: %(default)s]")
    ap.add_argument(
        "-s", "--size", dest="size", metavar="width,height",
        default="%.2f,%.2f" % (size.get("width", 27.65),
                               size.get("height", 39.51)),
        help="Real size of the example pattern in mm. [default: %(default)s]")
    ap.add_argument(
        "-o", "--output", dest="output", default=None, metavar="FILENAME.npz",
        required=True,
        help="Write calibration result to this .npz file.")
    ap.add_argument(
        "-r", "--resolution", dest="resolution", metavar="WIDTH,HEIGHT",
        default="%d,%d" % (res.get("width", 320), res.get("height", 240)),
        help="Set camera resolution. [default: %(default)s]")
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
        examples=args.examples,
        resolution=args.resolution,
        minimum=args.minimum,
        pattern=args.pattern,
        grid=args.grid,
        size=args.size,
        output=args.output,
        debug=args.debug,
        )
