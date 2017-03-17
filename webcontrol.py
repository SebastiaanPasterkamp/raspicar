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
#import picamera

camera = None

class CarControl(SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/camera':
            self.send_response(200)
            self.send_header('Content-type','multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            stream=io.BytesIO()
            try:
                global camera
                start=time.time()
                for foo in camera.capture_continuous(stream,'jpeg'):
                    self.wfile.write("--jpgboundary")
                    self.send_header('Content-type','image/jpeg')
                    self.send_header('Content-length',len(stream.getvalue()))
                    self.end_headers()
                    self.wfile.write(stream.getvalue())
                    stream.seek(0)
                    stream.truncate()
                    time.sleep(.5)
            except KeyboardInterrupt:
                pass
        else:
            return SimpleHTTPRequestHandler.do_GET(self)


if __name__ == "__main__":

    port = 8000
    public_dir = os.path.abspath(os.path.join(
        os.path.dirname(__file__), 'public'
        ))

    os.chdir(public_dir)

    camera = picamera.PiCamera()
    camera.resolution = (640, 480)

    server = ThreadingSimpleServer(('', port), CarControl)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        camera.close()
        server.socket.close()
        print("Finished")
