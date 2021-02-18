#!/bin/env python

import os
import glob
from argparse import ArgumentParser

from modules.camera import Calibration


# construct the argument parse and parse the arguments
ap = ArgumentParser()
ap.add_argument(
    "-m", "--minimum", dest="minimum", type=int, default=10,
    help="Minimum number of examples to calibrate with. [default: %(default)d]")
ap.add_argument(
    "-e", "--examples", default='capture/*.png', dest="examples", metavar='GLOB',
    help="Use these examples (glob pattern). [default: %(default)s]")
ap.add_argument(
    "-p", "--pattern", dest="pattern", default='asymetric',
    choices=['chess', 'circles', 'asymetric'],
    help="Type of calibration pattern used. [default: %(default)s]")
ap.add_argument(
    "-g", "--grid", dest="grid", default='4,11', metavar="rows,columns",
    help="Grid size of the example pattern in number of corners / dots. [default: %(default)s]")
ap.add_argument(
    "-s", "--size", dest="size", default='27.65,39.51', metavar="width,height",
    help="Real size of the example pattern in mm. [default: %(default)s]")
ap.add_argument(
    "-o", "--output", dest="output", default=None, metavar="FILENAME",
    help="Write calibration result to this .npz file.")
options = ap.parse_args()
options.grid = tuple([int(g) for g in options.grid.split(',')])
options.size = tuple([float(s) for s in options.size.split(',')])

file_pattern = os.path.abspath(options.examples)
images = glob.glob(file_pattern)

calib = Calibration(
    images, options.minimum,
    options.pattern, options.grid, options.size
    )

if calib.collectExamples():
    calib.calibrate(options.output)
