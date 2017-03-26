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
import cv
import time
from datetime import datetime
import json

try:
    import RPi.GPIO as GPIO
    from sunfounder.PCA9685 import PWM
except:
    import os
    sys.path.insert(0, os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..'
        )))

    from test.mockobjects import MockGPIO, MockPWM as PWM
    GPIO = MockGPIO()

from modules.vehicle import Car
from modules.camera import VisualOdometry

camera = cv.CaptureFromCAM(-1)
cameraQuality=55

odometry = VisualOdometry(
    cv.QueryFrame(camera),
    100, 150
    )

config = {}
config_file = os.path.abspath(os.path.join(
    os.path.dirname(__file__), 'config.json'))
if os.path.exists(config_file):
    with open(config_file, 'r') as config_json:
        config = json.load(config_json)

GPIO.setmode(GPIO.BOARD)    # Number GPIOs by its physical location
pwm = PWM()                 # The servo controller.
pwm.frequency = 60

car = Car(pwm, GPIO, config=config, debug=True)
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
            self.wfile.write("Content-Type: multipart/x-mixed-replace; boundary=--aaboundary")
            self.wfile.write("\r\n\r\n")

            while running:
		try:
                    img = cv.QueryFrame(camera)
                    odometry.followFeatures(img)

                    if capture:
                        if last_snapshot <= time.time():
                            last_snapshot = time.time() + 3.0
                            print last_snapshot, time.time()
                            ts = datetime.now()
                            filename = os.path.abspath(os.path.join(
                                os.path.dirname(__file__),
                                '..',
                                'capture',
                                ts.strftime('capture.%Y%m%d-%H%M%s.jpg')
                                ))
                            print "Write to:", filename
                            cv.SaveImage(filename, img)

                    for feature in odometry.features:
                        cv.Circle(
                            img,
                            (int(feature[0]), int(feature[1])),
                            2,
                            (0, 0, 255),
                            -1, 8, 0
                            )

                    cv2mat=cv.EncodeImage(".jpeg",img,(cv.CV_IMWRITE_JPEG_QUALITY,cameraQuality))
                    JpegData=cv2mat.tostring()
                    self.wfile.write("--aaboundary\r\n")
                    self.wfile.write("Content-Type: image/jpeg\r\n")
                    self.wfile.write("Content-length: "+str(len(JpegData))+"\r\n\r\n" )
                    self.wfile.write(JpegData)
                    self.wfile.write("\r\n\r\n\r\n")
                    time.sleep(0.05)
                except Exception as e:
                    print "Done streaming", e
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
        del(camera)
        server.shutdown()
        print("Finished")
