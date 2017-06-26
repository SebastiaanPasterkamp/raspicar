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

class Calibration(object):
    def __init__(self, images, min_examples, pattern, grid, size):
        """ Calibrate the camera attributes using example images
        containing a known calibration pattern.

        - images:       list of example images potentially containing
                        a detectable pattern (not guarenteed).
        - min_examples: minimum number of examples to calibrate with.
        - pattern:      type of pattern in the image: chess, circles,
                        or asymetric.
        - grid:         number of features in the example pattern.
        - size:         real-world dimensions of the pattern.

        The geometries are expected to be (width, height).
        """

        self.images = images
        self.min_examples = min_examples
        self.pattern = pattern
        self.grid = grid
        self.size = size

        self.examples = {
            'object_points': [],
            'image_points': []
            }
        self.object_points = Calibration.getObjectPoints(
            pattern, grid, size)
        self.method = cv2.findCirclesGrid
        self.flags = cv2.CALIB_CB_CLUSTERING

        if pattern == 'asymetric':
            self.flags = cv2.CALIB_CB_ASYMMETRIC_GRID \
                | cv2.CALIB_CB_CLUSTERING
        if pattern == 'chess':
            self.method = cv2.findChessboardCorners

    @staticmethod
    def getObjectPoints(pattern, grid, size):
        if pattern == 'asymetric':
            return np.array([
                [
                    np.float32(
                        y * size[1] / (grid[1]-1)
                        ),
                    np.float32(
                        (x + (y%2) * .5) * size[0] / (grid[0]-0.5)
                        ),
                    np.float32(0.0)
                    ]
                    for y in range(grid[1])
                    for x in range(grid[0])
                ])
        return np.array([
            [
                np.float32(y * size[1] / (grid[1]-1)),
                np.float32(x * size[0] / (grid[0]-1)),
                np.float32(0.0)
                ]
                for y in range(grid[1])
                for x in range(grid[0])
            ])

    def collectExamples(self):
        img_shape = None
        example_count = 0
        for fname in self.images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray = cv2.medianBlur(gray, 5)
            img_shape = gray.shape[::-1]

            # Find the chess board corners
            status, corners = self.method(
                gray, self.grid, flags=self.flags
                )

            # If found, add object points and image points
            if status == True:
                print "Pattern found in '%s'" % fname
                self.examples['object_points'].append(
                    self.object_points)
                self.examples['image_points'].append(corners)
                example_count += 1
            else:
                print "No pattern found in '%s'" % fname

        if example_count < self.min_examples:
            print "Not enough usable examples found. Need %d, only found %d" % (
                self.min_examples, example_count
                )
            return False
        return True

    def calibrate(self, output_file=None):
        example_count = len(self.examples['image_points'])
        print "Calibrating based on %d samples" % example_count

        img = cv2.imread(self.images[0])
        img_shape = img.shape[::2]
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.examples['object_points'],
            self.examples['image_points'],
            img_shape
            )

        mean_error = 0.0
        for i in range(example_count):
            corners, _ = cv2.projectPoints(
                self.examples['object_points'][i],
                rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(
                self.examples['image_points'][i],
                corners, cv2.NORM_L2) / len(corners)
            mean_error += error

        print 'mtx', mtx
        print 'dist', dist
        print "total error: ", mean_error / example_count

        if output_file:
            np.savez(output_file, mtx=mtx, dist=dist)
        return True

class VisualOdometry(object):
    def __init__(self, camera):
        self.camera = camera

        start_image = self.camera.read()
        while start_image is None:
            start_image = self.camera.read()
        self.previous = cv2.cvtColor(start_image, cv2.COLOR_BGR2GRAY)
        self.current = cv2.cvtColor(start_image, cv2.COLOR_BGR2GRAY)

        self.path = []

        self.running = True
        self.trackGrid = False
        self.trackFeatures = False

    def start(self):
        """ start the thread to read frames from the video stream """
        if self.trackGrid or self.trackFeatures:
            Thread(target=self.update, args=()).start()
        return self

    def stop(self):
        self.running = False

    def update(self):
        while self.running:
            new_frame = self.camera.read()

            self.previous, self.current = self.current, cv2.cvtColor(
                new_frame, cv2.COLOR_BGR2GRAY)

            if self.trackFeatures:
                self.followFeatures()

            if self.trackGrid:
                self.followGrid()

    def initGrid(self, pattern='chess', grid=(5,5), size=(5.,5.)):
        """ Initialize a chessboard or circle grid of SIZE mm with
        FEATURES features in PATTERN formation.

        - pattern:      string with 'chess', 'circles', 'asymetric'
                        to clearify the type of grid being tracked
        - grid:         tuple of (width, height) features
        - size:         tuple of (width, height) in mm
        """

        self.grid = {
            'pattern': pattern,
            'grid': grid,
            'size': size,
            'objp': Calibration.getObjectPoints(pattern, grid, size),
            'method': cv2.findCirclesGrid,
            'flags': cv2.CALIB_CB_CLUSTERING,
            'status': False,
            'corners': None
            }
        if self.grid['pattern'] == 'asymetric':
            self.grid['flags'] = cv2.CALIB_CB_ASYMMETRIC_GRID \
                | cv2.CALIB_CB_CLUSTERING
        if self.grid['pattern'] == 'chess':
            self.grid['method'] = cv2.findChessboardCorners
        self.trackGrid = True

    def followGrid(self):
        status, corners = self.grid['method'](
            self.current,
            self.grid['features'],
            flags=self.grid['flags']
            )

        if status:
            self.grid['status'] = True
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
                self.grid['status'] = True
                self.grid['corners'] = corners
            else:
                self.grid['status'] = False

        if self.grid['status']:
            rvecs, tvecs, inliers = cv2.solvePnPRansac(
                self.grid['objp'],
                self.grid['corners'],
                self.camera.mtx, self.camera.dist
                )
            self.path.append([tvecs[0][0], tvecs[2][0]])

    def getGrid(self):
        if self.trackGrid:
            return self.grid['status'], self.grid['corners']
        return False, []

    def initFeatures(self, min_features=100, max_features=200,
                     min_distance=32):

        self.features = {
            'min': min_features,
            'max': max_features,
            'distance': min_distance,
            'quality': 0.05,
            'points': []
            }

        # Make sure we have features
        self.findNewFeatures()
        self.trackFeatures = True

    def followFeatures(self):
        # calculate optical flow
        old_features = np.float32(self.features['points'])
        if not len(old_features):
            #Make sure we have enough features
            self.findNewFeatures()
            return

        new_features, status, error = cv2.calcOpticalFlowPyrLK(
            self.previous,
            self.current,
            old_features,
            winSize=(
                int(self.features['distance']),
                int(self.features['distance'])
                ),
            maxLevel=2,
            criteria=(
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                10, 0.03
                )
            )

        self.features['points'] = [
            tuple(feature.ravel())
            for i, feature in enumerate(new_features)
            if status[i]
            ]

        if len(self.features['points']) < self.features['min']:
            #Make sure we have enough features
            self.findNewFeatures()

    def getFeatures(self):
        if self.trackFeatures:
            return self.features['points']
        return []

    def findNewFeatures(self):
        # Avoid sampling new features around current features
        mask = np.zeros_like(self.current)
        mask[:] = 255

        for f in self.features['points']:
            cv2.circle(mask, f, 5, 0, -1)

        new_features = cv2.goodFeaturesToTrack(
            image=self.current,
            maxCorners=\
                self.features['max'] - len(self.features['points']),
            qualityLevel=self.features['quality'],
            minDistance=self.features['distance'],
            mask=mask
            )

        if new_features is not None:
            self.features['points'].extend([
                (x,y)
                for x, y in np.float32(new_features).reshape(-1, 2)
                ])

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