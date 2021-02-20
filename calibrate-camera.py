#!/usr/bin/env python

import os
import glob
import json
from argparse import ArgumentParser

from modules.camera import Calibration


def main(examples, minimum, pattern, grid, size, output):
    file_pattern = os.path.abspath(examples)
    images = glob.glob(file_pattern)

    calib = Calibration(images, minimum, pattern, grid, size)

    if calib.collectExamples():
        calib.calibrate(output)


def parse_args(config):
    calib = config.get("calibration", {})
    grid = calib.get("grid", {})
    size = calib.get("size", {})

    ap = ArgumentParser()
    ap.add_argument(
        "-m", "--minimum", dest="minimum", type=int,
        default=calib.get("minimum", 10),
        help="Minimum number of examples to calibrate with. "
        "[default: %(default)d]")
    ap.add_argument(
        "-e", "--examples", default="capture/*.png", dest="examples",
        metavar="GLOB", help="Use these examples (glob pattern). "
        "[default: %(default)s]")
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
        default="%.2f,%2f" % (size.get("width", 27.65),
                              size.get("height", 39.51)),
        help="Real size of the example pattern in mm. [default: %(default)s]")
    ap.add_argument(
        "-o", "--output", dest="output", default=None, metavar="FILENAME.npz",
        required=True,
        help="Write calibration result to this .npz file.")
    ap.add_argument(
        "-c", "--config", metavar="file.json", default="config.json",
        help="Config file to use. [default: %(default)s]")

    return ap.parse_args(), ap


def loadConfig(filename):
    config = {}
    with open(filename, 'r') as fh:
        config = json.load(fh)
    return config


if __name__ == "__main__":
    args, ap = parse_args(dict())
    if os.path.exists(args.config):
        # reparse args with defaults from config
        config = loadConfig(args.config)
        args, ap = parse_args(config)

    args.grid = tuple([int(g) for g in args.grid.split(",")])
    args.size = tuple([float(s) for s in args.size.split(",")])

    main(
        examples=args.examples,
        minimum=args.minimum,
        pattern=args.pattern,
        grid=args.grid,
        size=args.size,
        output=args.output,
        )
