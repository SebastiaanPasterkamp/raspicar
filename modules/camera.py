"""Computer Vision class to

- support threaded camera IO
- organize visual odometry

Threaded camera IO based on:
  http://www.pyimagesearch.com/2015/12/21/increasing-webcam-fps-with-python-and-opencv/
"""

import os
import glob
import cv2
import numpy as np
from threading import Thread, Lock
try:
    from picamera.array import PiRGBArray
    from picamera import PiCamera
except ImportError as e:
    print("Not running on a Raspberry Pi", e)
    pass
import time


class FPS(object):
    def __init__(self, window=1.0):
        """ store the start time, end time, and total number of frames
        that were examined between the start and end intervals. The FPS is
        calculcated using the frames during the previous time window and the
        current incomplete window. A new window is created when a new frame is
        tallied beyond the window.
        """
        self._window = window
        self._start = [None, None]
        self._end = None
        self._frames = [0, 0]

    def start(self):
        """ start the timer """
        self._start = [None, time.time()]
        return self

    def stop(self):
        """ stop the timer """
        self._end = time.time()

    def update(self):
        """ increment the total number of frames examined during the
        start and end intervals
        """
        self._frames[1] += 1
        if self._start[1] + self._window < time.time():
            self._frames = [self._frames[1], 0]
            self._start = [self._start[1], time.time()]

    def elapsed(self):
        """ return the total number of seconds between the start and
        end interval
        """
        if not self._start[0]:
            return 0
        end = self._end or time.time()
        return end - self._start[0]

    def fps(self):
        """ compute the (approximate) frames per second """
        if not self._start[0]:
            return None
        return sum(self._frames) / self.elapsed()


class CameraStream(object):
    def __init__(self, resolution=(320, 240), framerate=32):
        """ initialize the frame and the variable used to indicate
        if the thread should be stopped """
        self.resolution = resolution
        self.framerate = framerate
        self._fps = FPS()

        self.frame = None
        self.stopped = True
        self.video_writer = None
        self.readers = {}
        self.readers_lock = Lock()

    def getFPS(self):
        return self._fps.elapsed()

    def start(self):
        """ start the thread to read frames from the video stream """
        self.stopped = False
        self._fps.start()
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        """ Implementation specific """
        pass

    def read(self, id=None):
        """ return the frame most recently read """
        with self.readers_lock:
            if id in self.readers:
                self.readers[id].acquire()
        return self.frame

    def stop(self):
        """ indicate that the thread should be stopped """
        self.stopped = True
        self._fps.stop()

    def addSyncedReader(self, id):
        """ register a reader thread only interested in new frames. Their
        thread will get a unique (by id) lock which will be blocked until a
        fresh frame is available. """
        if self.stopped:
            raise RuntimeError("Cannot register synchronized readers if camera"
                               " is not running in a thread.")
        with self.readers_lock:
            if id in self.readers:
                raise ValueError(
                    f"synchronized reader {id} is already registered.")
            self.readers[id] = Lock()
            self.readers[id].acquire()

    def delSyncedReader(self, id):
        """ removes a reader thread id. """
        with self.readers_lock:
            if id not in self.readers:
                return
            # Release the lock in case something is still blocked on it
            self.readers[id].release()
            del self.readers[id]

    def loadCalibration(self, filename):
        with np.load(filename) as calib:
            self.mtx, self.dist = calib['mtx'], calib['dist']

    def process(self, frame):
        self._fps.update()

        # Inform every reader that a new frame is available
        with self.readers_lock:
            for lock in self.readers.values():
                lock.release()

        if self.video_writer is not None:
            self.video_writer.write(frame)

    def record(self, path):
        if path is None:
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
            return

        fourcc = None
        if callable(cv2.CV_FOURCC):
            fourcc = cv2.CV_FOURCC(*'MJPG')
        else:
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.video_writer = cv2.VideoWriter(
            path, fourcc,
            self.framerate,  # fps
            (
                int(self.resolution[0]),        # width
                int(self.resolution[1]),        # height
                )
            )


class WebcamCameraStream(CameraStream):
    def __init__(self, src=0, resolution=(320, 240), framerate=32):
        """ initialize the video camera stream and read the first frame
        from the stream
        """
        super(WebcamCameraStream, self).__init__(
            resolution=resolution, framerate=framerate)

        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH,
                        self.resolution[0])
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT,
                        self.resolution[1])
        self.stream.set(cv2.CAP_PROP_FPS, self.framerate)

        self.resolution = (
            self.stream.get(cv2.CAP_PROP_FRAME_WIDTH),
            self.stream.get(cv2.CAP_PROP_FRAME_HEIGHT)
            )
        self.framerate = self.stream.get(cv2.CAP_PROP_FPS)
        if self.framerate <= 0:
            self.framerate = framerate

        (self.grabbed, self.frame) = self.stream.read()

    def update(self):
        """ keep looping indefinitely until the thread is stopped """
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


class SlideShowCamera(CameraStream):
    def __init__(self, resolution=(320, 240), framerate=32):
        super(SlideShowCamera, self).__init__(
            resolution=resolution, framerate=framerate)
        self.index = 0
        self.images = []

    def loadImages(self, pattern):
        file_pattern = os.path.abspath(pattern)
        self.images.extend(sorted(glob.glob(file_pattern)))
        if self.frame is None and len(self.images):
            self.readImage(self.images[0])

    def readImage(self, path):
        frame = cv2.imread(path)
        self.frame = cv2.resize(frame, self.resolution)

    def update(self):
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return

            # otherwise, read the next frame from the stream
            self.readImage(self.images[self.index])
            self.process(self.frame)
            time.sleep(1.0 / self.framerate)

    def read(self, id=None):
        """ return the frame most recently read """
        frame = super(SlideShowCamera, self).read(id)
        self.index += 1
        if self.index >= len(self.images):
            self.stop()
            self.frame = None
        elif self.stopped:
            self.readImage(self.images[self.index])
        return frame


def VideoCamera(src=0, usePiCamera=False, patterns=[], resolution=(320, 240),
                framerate=32):
    # check to see if the picamera module should be used
    if usePiCamera:
        # initialize the picamera stream and allow the camera sensor to warmup
        return PiCameraStream(resolution=resolution, framerate=framerate)

    if len(patterns):
        camera = SlideShowCamera(resolution=resolution, framerate=framerate)
        for pattern in patterns:
            camera.loadImages(pattern)
        return camera

    # otherwise, we are using OpenCV so initialize the webcam stream
    return WebcamCameraStream(
        src=src, resolution=resolution, framerate=framerate)


class Calibration(object):
    def __init__(self, pattern, grid, size):
        """ Calibrate the camera attributes using example images
        containing a known calibration pattern.

        - pattern:      type of pattern in the image: chess, circles,
                        or asymetric.
        - grid:         number of features in the example pattern.
        - size:         real-world dimensions of the pattern.

        The geometries are expected to be (width, height).
        """
        self.pattern = pattern
        self.grid = grid
        self.size = size

        self.resolution = (0, 0)
        self.image_points = []
        self.object_points = Calibration.getObjectPoints(pattern, grid, size)
        self.method = cv2.findCirclesGrid
        self.flags = [
            cv2.CALIB_CB_SYMMETRIC_GRID,
            cv2.CALIB_CB_SYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING,
            ]
        if pattern == 'asymetric':
            self.flags = [
                cv2.CALIB_CB_ASYMMETRIC_GRID,
                cv2.CALIB_CB_ASYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING,
                ]
        elif pattern == 'chess':
            self.flags = [
                cv2.CALIB_CB_ADAPTIVE_THRESH
                | cv2.CALIB_CB_NORMALIZE_IMAGE
                | cv2.CALIB_CB_FAST_CHECK,
                ]
            self.method = cv2.findChessboardCorners

    @staticmethod
    def getObjectPoints(pattern, grid, size):
        if pattern == 'asymetric':
            return np.array([
                [
                    np.float32(y * size[1] / (grid[1]-1)),
                    np.float32((x + (y % 2) * .5) * size[0] / (grid[0]-0.5)),
                    np.float32(0.0),
                    ]
                for y in range(grid[1])
                for x in range(grid[0])
                ])
        return np.array([
            [
                np.float32(y * size[1] / (grid[1]-1)),
                np.float32(x * size[0] / (grid[0]-1)),
                np.float32(0.0),
                ]
            for y in range(grid[1])
            for x in range(grid[0])
            ])

    def addImage(self, image):
        if not self.resolution[0]:
            self.resolution = tuple(image.shape[:2])
        status, corners = self.findGrid(image)
        if status:
            self.image_points.append(corners)

    def addImages(self, images):
        for image in images:
            self.addImage(image)

    def findGrid(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.medianBlur(gray, 5)

        # Find the chess board corners
        attempt = 1
        for flags, img in [
                (flag, img)
                for flag in self.flags
                for img in (gray, blurred)
                ]:
            try:
                status, corners = self.method(img, self.grid, flags=flags)
            except cv2.error as e:
                print(e)
                continue
            if status:
                break
            attempt += 1

        # If found, add object points and image points
        return status, corners

    def getPose(self, corners, camera):
        return cv2.solvePnPRansac(
            self.object_points,
            corners,
            camera.mtx, camera.dist
            )

    def calibrate(self, min_examples=None, output_file=None):
        example_count = len(self.image_points)
        if example_count < min_examples:
            print(
                "Not enough usable examples found. Need %d, only found %d" % (
                    min_examples, example_count))
            return False
        print("Calibrating based on %d samples" % example_count)

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            [self.object_points for _ in self.image_points],
            self.image_points,
            self.resolution,
            None,
            None,
            )

        mean_error = 0.0
        for i in range(example_count):
            corners, _ = cv2.projectPoints(
                self.object_points,
                rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(self.image_points[i], corners, cv2.NORM_L2)
            mean_error += error / len(corners)

        print('mtx', mtx)
        print('dist', dist)
        print("total error: ", mean_error / example_count)

        if output_file:
            np.savez(output_file, mtx=mtx, dist=dist)
        return True


class VisualOdometry(object):
    def __init__(self, camera):
        self.camera = camera
        self.previous = None
        self.current = None

        self.grid_path = []
        self.track_path = []

        self.running = True
        self.trackGrid = False
        self.trackFeatures = False

    def start(self):
        """ start the thread to read frames from the video stream """
        self.camera.addSyncedReader("odometry")
        start_image = self.camera.read("odometry")
        self.previous = cv2.cvtColor(start_image, cv2.COLOR_BGR2GRAY)
        self.current = cv2.cvtColor(start_image, cv2.COLOR_BGR2GRAY)

        Thread(target=self.update, args=()).start()
        return self

    def stop(self):
        self.running = False

    def update(self):
        while self.running:
            self.setFrame(self.camera.read("odometry"))

    def setFrame(self, frame):
        self.previous, self.current = (
            cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY),
            self.current,
            )

        if self.trackFeatures:
            self.followFeatures()

        if self.trackGrid:
            self.followGrid()

    def initGrid(self, pattern='chess', grid=(5, 5), size=(5., 5.)):
        """ Initialize a chessboard or circle grid of SIZE mm with
        FEATURES features in PATTERN formation.

        - pattern:      string with 'chess', 'circles', 'asymetric'
                        to clearify the type of grid being tracked
        - grid:         tuple of (width, height) features
        - size:         tuple of (width, height) in mm
        """

        self.calibration = Calibration(pattern, grid, size)

    def followGrid(self):
        track_status = False

        grid_status, corners = self.calibration.findGrid(self.current)

        if not grid_status and self.grid['corners'] is not None:
            # calculate optical flow
            corners, status, error = cv2.calcOpticalFlowPyrLK(
                self.previous,
                self.current,
                self.grid['corners'],
                None,
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

            track_status = (len(corners) == len(self.grid['corners']))

        if grid_status or track_status:
            self.grid['status'], self.grid['corners'] = True, corners
            rvecs, tvecs, inliers = self.calibrate.getPose(
                corners, self.camera)

            if grid_status:
                self.grid_path.append([tvecs[2][0], tvecs[0][0]])
            self.track_path.append([tvecs[2][0], tvecs[0][0]])

    def getGrid(self):
        if self.calibration is not None:
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
            # Make sure we have enough features
            self.findNewFeatures()
            return

        new_features, status, error = cv2.calcOpticalFlowPyrLK(
            self.previous,
            self.current,
            old_features,
            None,
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
            # Make sure we have enough features
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
            maxCorners=self.features['max'] - len(self.features['points']),
            qualityLevel=self.features['quality'],
            minDistance=self.features['distance'],
            mask=mask
            )

        if new_features is not None:
            self.features['points'].extend([
                (x, y)
                for x, y in np.float32(new_features).reshape(-1, 2)
                ])


if __name__ == '__main__':
    camera = VideoCamera()
    camera.addSyncedReader("main")
    camera.start()

    odometry = VisualOdometry(camera)
    odometry.initFeatures(
        min_features=100,
        max_features=200,
        min_distance=32,
        )
    odometry.start()

    while True:
        img = camera.read("main")
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

    odometry.stop()
    camera.stop()
