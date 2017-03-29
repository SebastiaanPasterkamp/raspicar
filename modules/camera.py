"""Computer Vision class to organize visual odometry"""

import cv2
import numpy as np

class VisualOdometry(object):
    def __init__(self, start_image,
                 min_features=100, max_features=200, min_distance=32):

        self.previous = cv2.cvtColor(start_image, cv2.COLOR_BGR2GRAY)
        self.current = cv2.cvtColor(start_image, cv2.COLOR_BGR2GRAY)

        self.min_features = min_features
        self.max_features = max_features
        self.min_distance = min_distance

        self.quality = 0.3
        self.features = []

        # Make sure we have features
        self.findNewFeatures()

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