"""Computer Vision class to

- support threaded camera IO
- organize visual odometry

Threaded camera IO based on:
  http://www.pyimagesearch.com/2015/12/21/increasing-webcam-fps-with-python-and-opencv/
"""

import cv2
import numpy as np
from threading import Thread
try:
    from picamera.array import PiRGBArray
    from picamera import PiCamera
except:
    # May not be running on an actual RPi
    pass
import datetime

class FPS(object):
    def __init__(self):
        """ store the start time, end time, and total number of frames
        that were examined between the start and end intervals
        """
        self._start = None
        self._end = None
        self._numFrames = 0

    def start(self):
        """ start the timer """
        self._start = datetime.datetime.now()
        return self

    def stop(self):
        """ stop the timer """
        self._end = datetime.datetime.now()

    def update(self):
        """ increment the total number of frames examined during the
        start and end intervals
        """
        self._numFrames += 1

    def elapsed(self):
        """ return the total number of seconds between the start and
        end interval
        """
        return (self._end - self._start).total_seconds()

    def fps(self):
        """ compute the (approximate) frames per second """
        return self._numFrames / self.elapsed()


class CameraStream(object):
    def __init__(self, resolution=(320, 240), framerate=32):
        """ initialize the frame and the variable used to indicate
        if the thread should be stopped """
        self.resolution = resolution
        self.framerate = framerate

        self.frame = None
        self.stopped = False
        self.video_writer = None

    def start(self):
        """ start the thread to read frames from the video stream """
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        """ Implementation specific """
        pass

    def read(self):
        """ return the frame most recently read """
        return self.frame

    def stop(self):
        """ indicate that the thread should be stopped """
        self.stopped = True

    def process(self, frame):
        if self.video_writer is not None:
            self.video_writer.write(frame)

    def record(self, path):
        if path is None:
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
            return

        fourcc = None
        if callable(cv2.cv.CV_FOURCC):
            fourcc = cv2.cv.CV_FOURCC(*'MJPG')
        else:
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.video_writer = cv2.VideoWriter(
            path, fourcc,
            self.framerate, # fps
            (
                int(self.resolution[0]),        # width
                int(self.resolution[1])         # height
            ))

class WebcamCameraStream(CameraStream):
    def __init__(self, src=0, resolution=(320, 240), framerate=32):
        """ initialize the video camera stream and read the first frame
        from the stream
        """
        super(WebcamCameraStream, self).__init__(
            resolution=resolution, framerate=framerate)

        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,
                        self.resolution[0])
        self.stream.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,
                        self.resolution[1])
        self.stream.set(cv2.cv.CV_CAP_PROP_FPS, self.framerate)

        self.resolution = (
            self.stream.get(cv2.cv.CV_CAP_PROP_FRAME_WIDTH),
            self.stream.get(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT)
            )
        self.framerate = self.stream.get(cv2.cv.CV_CAP_PROP_FPS)
        if self.framerate <= 0:
            self.framerate = framerate

        (self.grabbed, self.frame) = self.stream.read()

    def update(self):
        """ keep looping infinitely until the thread is stopped """
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return

            # otherwise, read the next frame from the stream
            (self.grabbed, self.frame) = self.stream.read()
            self.process(self.frame)

class PiCameraStream(CameraStream):
    def __init__(self, resolution=(320, 240), framerate=32):
        """ initialize the camera and stream """
        super(PiCameraStream, self).__init__(
            resolution=resolution, framerate=framerate)

        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(
            self.rawCapture, format="bgr", use_video_port=True)

    def update(self):
        """ keep looping infinitely until the thread is stopped """
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array
            self.rawCapture.truncate(0)
            self.process(self.frame)

            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return


class VideoCamera(CameraStream):
    def __init__(self, src=0, usePiCamera=False,
                 resolution=(320, 240), framerate=32):
        super(VideoCamera, self).__init__(
            resolution=resolution, framerate=framerate)

        # check to see if the picamera module should be used
        if usePiCamera:
            # initialize the picamera stream and allow the camera
            # sensor to warmup
            self.stream = PiCameraStream(
                resolution=resolution, framerate=framerate)

        # otherwise, we are using OpenCV so initialize the webcam
        # stream
        else:
            self.stream = WebcamCameraStream(
                src=src, resolution=resolution, framerate=framerate)

    def start(self):
        # start the threaded video stream
        return self.stream.start()

    def update(self):
        # grab the next frame from the stream
        self.stream.update()

    def read(self):
        # return the current frame
        return self.stream.read()

    def stop(self):
        # stop the thread and release any resources
        self.stream.stop()

class VisualOdometry(object):
    def __init__(self, start_image,
                 min_features=100, max_features=200, min_distance=32):

        self.previous = cv2.cvtColor(start_image, cv2.COLOR_BGR2GRAY)
        self.current = cv2.cvtColor(start_image, cv2.COLOR_BGR2GRAY)

        self.min_features = min_features
        self.max_features = max_features
        self.min_distance = min_distance

        self.quality = 0.05
        self.features = []

        # Make sure we have features
        self.findNewFeatures()

    def initGrid(self, size=(5.,5.), features=(5,5), shape='chess'):
        """ Initialize a chessboard or circle grid of SIZE mm with
        FEATURES features in SHAPE formation.

        - size:         tuple of (width, height) in mm
        - features:     tuple of (width, height) features
        - shape:        string with 'chess', 'circle', 'asymetric'
                        to clearify the type of grid being tracked
        """

        self.grid = {
            'size': size,
            'features': features,
            'shape': shape,
            'objp': None,
            'method': cv2.findCirclesGrid,
            'flags': cv2.CALIB_CB_CLUSTERING,
            'corners': None
            }
        if self.grid['shape'] == 'asymetric':
            self.grid['flags'] = cv2.CALIB_CB_ASYMMETRIC_GRID \
                | cv2.CALIB_CB_CLUSTERING

            self.grid['objp'] = np.array([
                [
                    np.float32(
                        y * size[1] / (features[1]-1)
                        ),
                    np.float32(
                        (x + (y%2) * .5) * size[0] / (features[0]-0.5)
                        ),
                    np.float32(0.0)
                    ]
                    for y in range(features[1])
                    for x in range(features[0])
                ])
        else:
            if self.grid['shape'] == 'chess':
                self.grid['method'] = cv2.findChessboardCorners
            self.grid['objp'] = np.array([
                [
                    np.float32(y * size[1] / (features[1]-1)),
                    np.float32(x * size[0] / (features[0]-1)),
                    np.float32(0.0)
                    ]
                    for y in range(features[1])
                    for x in range(features[0])
                ])

    def getGrid(self, new_image):
        self.current = cv2.cvtColor(new_image, cv2.COLOR_BGR2GRAY)

        status, corners = self.grid['method'](
            self.current,
            self.grid['features'],
            flags=self.grid['flags']
            )

        if status:
            self.grid['corners'] = corners
        elif self.grid['corners'] is not None:
            # calculate optical flow
            corners, status, error = cv2.calcOpticalFlowPyrLK(
                self.previous,
                self.current,
                self.grid['corners'],
                winSize=(20, 20),
                maxLevel=2,
                criteria=(
                    cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                    10, 0.03
                    )
                )
            corners = np.float32([
                c for i, c in enumerate(corners) if status[i]
                ])

            if len(corners) == len(self.grid['corners']):
                status = True
                self.grid['corners'] = corners
            else:
                status = False

        self.previous, self.current = self.current, self.previous
        return status, corners

    def followFeatures(self, new_image):
        self.current = cv2.cvtColor(new_image, cv2.COLOR_BGR2GRAY)

        # calculate optical flow
        old_features = np.float32(self.features)
        new_features, status, error = cv2.calcOpticalFlowPyrLK(
            self.previous,
            self.current,
            old_features,
            winSize=(int(self.min_distance), int(self.min_distance)),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
            )

        self.features = [
            tuple(feature.ravel())
            for i, feature in enumerate(new_features)
            if status[i]
            ]

        if len(self.features) < self.min_features:
            #Make sure we have enough features
            self.findNewFeatures()

        self.previous, self.current = self.current, self.previous

    def findNewFeatures(self):
        # Avoid sampling new features around current features
        mask = np.zeros_like(self.current)
        mask[:] = 255

        for f in self.features:
            cv2.circle(mask, f, 5, 0, -1)

        new_features = cv2.goodFeaturesToTrack(
            image=self.current,
            maxCorners=self.max_features - len(self.features),
            qualityLevel=self.quality,
            minDistance=self.min_distance,
            mask=mask
            )

        if new_features is not None:
            for x, y in np.float32(new_features).reshape(-1, 2):
                self.features.append((x, y))

if __name__ == '__main__':
    capture = cv2.VideoCapture(-1)

    ret, img = capture.read()
    odometry = VisualOdometry(
        img,
        100, 150
        )

    while True:
        ret, img = capture.read()
        odometry.followFeatures(img)

        for feature in odometry.features:
            cv2.circle(
                img,
                (int(feature[0]), int(feature[1])),
                2,
                (0, 0, 255),
                -1, 8, 0
                )

        cv2.imshow('img', img)
        c = cv2.waitKey(1)
        if (c % 256) == 27:
            # ESC pressed
            break

    capture.release()