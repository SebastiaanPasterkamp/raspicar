#!/bin/env python

import sys
import os
import cv2
import json
import numpy as np
import glob

config = {}
config_file = os.path.abspath(os.path.join(
    os.path.dirname(__file__), 'config.json'))
if os.path.exists(config_file):
    with open(config_file, 'r') as config_json:
        config = json.load(config_json)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

file_pattern = os.path.abspath(os.path.join(
    os.path.dirname(__file__),
    '..',
    'capture',
    'capture.*.png'
    ))
images = glob.glob(file_pattern)
if images:
    print "Calibrating using '%s'" % file_pattern
else:
    print "No images found matching '%s'" % file_pattern
    sys.exit(1)

size = (27.65, 39.51)
grid = (4, 11)
# prepare object points
objp = np.array([
    [
        np.float32(y * size[1] / (grid[1]-1)),
        np.float32((x + (y%2) * .5) * size[0] / (grid[0]-0.5)),
        np.float32(0.0)
        ]
    for y in range(grid[1])
    for x in range(grid[0])
    ])

img_shape = None
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)
    img_shape = gray.shape[::-1]

    # Find the chess board corners
    status, corners = cv2.findCirclesGrid(
        gray,
        grid,
        flags=cv2.CALIB_CB_ASYMMETRIC_GRID + cv2.CALIB_CB_CLUSTERING
        )

    # If found, add object points, image points (after refining them)
    if status == True:
        objpoints.append(objp)
        imgpoints.append(corners)
    else:
        print "No pattern found in '%s'" % fname

if len(imgpoints) < 10:
    print "Not enough usable examples found."
    sys.exit(1)

print "Calibrating based on %d samples" % len(imgpoints)
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints,
    imgpoints,
    img_shape
    )

print 'mtx', mtx
print 'dist', dist

np.savez("camera_calib", mtx=mtx, dist=dist)

mean_error = 0.0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error += error

print "total error: ", mean_error/len(objpoints)
