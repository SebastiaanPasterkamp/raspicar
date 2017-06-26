#!/bin/env python
# Based on https://github.com/Nakiami/MultithreadedSimpleHTTPServer

try:
    # Python 2.x
    from SocketServer import ThreadingMixIn
    from SimpleHTTPServer import SimpleHTTPRequestHandler
    from BaseHTTPServer import HTTPServer
except ImportError:
    # Python 3.x
    from socketserver import ThreadingMixIn
    from http.server import SimpleHTTPRequestHandler, HTTPServer

class ThreadingSimpleServer(ThreadingMixIn, HTTPServer):
    pass

import sys
import os
import cv2
import numpy as np
import time
from datetime import datetime
import json

sys.path.insert(0, os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..'
    )))

usePiCamera=False
try:
    import RPi.GPIO as GPIO
    from sunfounder.PCA9685 import PWM
    usePiCamera = True
except:
    from test.mockobjects import MockGPIO, MockPWM as PWM
    GPIO = MockGPIO()

from modules.vehicle import Car
from modules.camera import VideoCamera, VisualOdometry

camera = VideoCamera(
    -1, usePiCamera=usePiCamera, resolution=(1280, 720),
    framerate=10).start()
camera_quality = 55

img = camera.read()
while img is None:
    img = camera.read()

odometry = VisualOdometry(camera)
odometry.initFeatures(100, 150)
odometry.start()

config = {}
config_file = os.path.abspath(os.path.join(
    os.path.dirname(__file__), 'config.json'))
if os.path.exists(config_file):
    with open(config_file, 'r') as config_json:
        config = json.load(config_json)

GPIO.setmode(GPIO.BOARD)    # Number GPIOs by its physical location
pwm = PWM()                 # The servo controller.
pwm.frequency = 60

car = Car(pwm, GPIO, config=config)
car.start()

running = True
capture = False
grid = False
last_snapshot = time.time()
class CarControl(SimpleHTTPRequestHandler):
    def shutdown(self):
        super(CarControl, self).shutdown()

    def do_GET(self):
        global running, capture, grid, last_snapshot

        if self.path == '/camera':
            self.send_response(200)
            self.send_header('Content-type','multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            while running:
                try:
                    img = camera.read()

                    if capture:
                        if last_snapshot <= time.time():
                            last_snapshot = time.time() + 3.0
                            ts = datetime.now()
                            filename = os.path.abspath(os.path.join(
                                os.path.dirname(__file__),
                                '..',
                                'capture',
                                ts.strftime('capture.%Y%m%d-%H%M%S.png')
                                ))
                            cv2.imwrite(filename, img)
                            print "Captured", filename

                    if grid:
                        status, corners = odometry.getGrid()

                        if status:
                            # Draw and display the corners
                            cv2.drawChessboardCorners(
                                img, odometry.grid['features'],
                                corners, status)

                    img_center = [
                        camera.resolution[0] * 0.5,
                        camera.resolution[1] * 0.5
                        ]
                    path = [
                        [
                            img_center[0] + p[0] * 0.1,
                            img_center[1] + p[1] * 0.1
                            ]
                        for p in car.path
                        ]

                    if len(path) > 1:
                        cv2.polylines(
                            img,
                            [np.array(path, np.int32)],
                            False,
                            (0,255,255)
                            )

                    path = []
                    if len(odometry.path):
                        first = odometry.path[0]
                        path = [
                            [
                                img_center[0] + p[0] * 0.1 - first[0],
                                img_center[1] + p[1] * 0.1 - first[1]
                                ]
                            for p in odometry.path
                            ]

                    if len(path) > 1:
                        cv2.polylines(
                            img,
                            [np.array(path, np.int32)],
                            False,
                            (0,0,255)
                            )

                    features = odometry.getFeatures()
                    for feature in features:
                        cv2.circle(
                            img,
                            (int(feature[0]), int(feature[1])),
                            2,
                            (0, 0, 255),
                            -1, 8, 0
                            )

                    r, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, camera_quality])
                    self.wfile.write("--jpgboundary\r\n")
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', str(len(buf)))
                    self.end_headers()
                    self.wfile.write(bytearray(buf))
                    self.wfile.write('\r\n')
                except Exception as e:
                    print "Done streaming", e
                    break
            return
        else:
            return SimpleHTTPRequestHandler.do_GET(self)

    def do_POST(self):
        global capture, grid
        if self.path == '/control':
            self.send_response(200)
            data_string = self.rfile.read(int(self.headers['Content-Length']))
            data = json.loads(data_string)

            if 'car' in data:
                car.setSpeed(data['car']['speed'])
                car.setDirection(data['car']['direction'])

            if 'camera' in data:
                car.setPanTilt(data['camera']['pan'], data['camera']['tilt'])

            if 'capture' in data:
                capture = bool(data['capture'])
                target_dir = os.path.abspath(os.path.join(
                    os.path.dirname(__file__),
                    '..',
                    'capture'
                    ))
                if not os.path.isdir(target_dir):
                    os.mkdir(target_dir)

            if 'grid' in data:
                odometry.initGrid(
                    (27.65, 39.51),
                    (4, 11),
                    'asymetric'
                    )
                grid = bool(data['grid'])

            if 'record' in data:
                record = bool(data['record'])
                if record:
                    target_dir = os.path.abspath(os.path.join(
                        os.path.dirname(__file__),
                        '..',
                        'record'
                        ))
                    if not os.path.isdir(target_dir):
                        os.mkdir(target_dir)
                    ts = datetime.now()
                    filename = os.path.abspath(os.path.join(
                        os.path.dirname(__file__),
                        '..',
                        'record',
                        ts.strftime('video-%Y%m%d-%H%M%S.avi')
                        ))
                    camera.record(filename)
                    print "recording to '%s', %rx%r @%r" % (
                        filename, camera.framerate,
                        camera.resolution[0], camera.resolution[1])
                else:
                    camera.record(None)

if __name__ == "__main__":
    port = 8000
    public_dir = os.path.abspath(os.path.join(
        os.path.dirname(__file__), 'public'
        ))

    os.chdir(public_dir)

    server = ThreadingSimpleServer(('', port), CarControl)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        running = False
        odometry.stop()
        camera.stop()
        server.shutdown()
        car.stop()
        print("Finished")
