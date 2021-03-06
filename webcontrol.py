#!/bin/env python
# Based on https://github.com/Nakiami/MultithreadedSimpleHTTPServer

try:
import sys
from modules.camera import VideoCamera, VisualOdometry
from modules.vehicle import Car
import json
from datetime import datetime
import time
import numpy as np
import cv2
import os
# Python 2.x
from socketserver import ThreadingMixIn
from http.server import SimpleHTTPRequestHandler
from http.server import HTTPServer
except ImportError:
    # Python 3.x
    from socketserver import ThreadingMixIn
    from http.server import SimpleHTTPRequestHandler, HTTPServer


class ThreadingSimpleServer(ThreadingMixIn, HTTPServer):
    pass


sys.path.insert(0, os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..'
    )))

usePiCamera = False
try:
    import RPi.GPIO as GPIO
    from sunfounder.PCA9685 import PWM
    usePiCamera = True
except:
    from test.mockobjects import MockGPIO, MockPWM as PWM
    GPIO = MockGPIO()


camera = VideoCamera(
    -1, usePiCamera=usePiCamera, resolution=(1280, 720),
    framerate=35).start()
camera_quality = 95

if os.path.exists('camera_calib.npz'):
    # Load previously saved data
    with np.load('camera_calib.npz') as calib:
        camera.mtx, camera.dist = [calib[i] for i in ('mtx', 'dist')]

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
last_snapshot = time.time()


class CarControl(SimpleHTTPRequestHandler):
    def shutdown(self):
        super(CarControl, self).shutdown()

    def do_GET(self):
        global running, capture, last_snapshot

        if self.path == '/camera':
            self.send_response(200)
            self.send_header(
                'Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
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
                            print("Captured", filename)

                    status, corners = odometry.getGrid()
                    if status:
                        # Draw and display the corners
                        cv2.drawChessboardCorners(
                            img, odometry.grid['features'],
                            corners, status)

                    center = [
                        camera.resolution[0] * 0.25,
                        camera.resolution[1] * 0.25
                        ]
                    paths = [
                        (car.path, [center[0], center[1]], (0, 0, 255)),
                        (odometry.grid_path, [
                         center[0]*3, center[1]], (0, 255, 0)),
                        (odometry.track_path, [
                         center[0], center[1]*3], (255, 0, 0))
                        ]

                    for path, img_center, color in paths:
                        line = []
                        if len(path):
                            first = path[0]
                            line = [
                                [
                                    img_center[0] + (p[0] - first[0]) * 0.1,
                                    img_center[1] + (p[1] - first[1]) * 0.1
                                    ]
                                for p in path
                                ]

                        if len(line) > 1:
                            cv2.polylines(
                                img,
                                [np.array(line, np.int32)],
                                False,
                                color
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

                    r, buf = cv2.imencode(
                        ".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, camera_quality])
                    self.wfile.write("--jpgboundary\r\n")
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', str(len(buf)))
                    self.end_headers()
                    self.wfile.write(bytearray(buf))
                    self.wfile.write('\r\n')
                except Exception as e:
                    print("Done streaming", e)
                    break
            return
        else:
            return SimpleHTTPRequestHandler.do_GET(self)

    def do_POST(self):
        global capture
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
                    print("recording to '%s', %rx%r @%r" % (
                        filename, camera.framerate,
                        camera.resolution[0], camera.resolution[1]))
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
