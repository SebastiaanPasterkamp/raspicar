"""Computer Vision class to organize visual odometry"""

import cv

class VisualOdometry(object):
    def __init__(self, start_image,
                 min_features=100, max_features=200, min_distance=32):

        self.size = cv.GetSize(start_image)
        self.previous = cv.CreateImage(self.size, 8, 1)
        self.current = cv.CreateImage(self.size, 8, 1)
        cv.CvtColor(start_image, self.previous, cv.CV_BGR2GRAY)
        cv.CvtColor(start_image, self.current, cv.CV_BGR2GRAY)

        self.min_features = min_features
        self.max_features = max_features
        self.min_distance = min_distance

        self.quality = 0.01
        self.features = []

        # Make sure we have features
        self.findNewFeatures()

    def followFeatures(self, new_image):
        cv.CvtColor(new_image, self.current, cv.CV_BGR2GRAY)

        next_features, next_features_status, error = cv.CalcOpticalFlowPyrLK(
            self.previous,
            self.current,
            None, None,
            self.features,
            (int(self.min_distance), int(self.min_distance)),
            0,
            (cv.CV_TERMCRIT_ITER|cv.CV_TERMCRIT_EPS, 20, 0.03),
            0
            )

        self.features = [
            next_features[i]
            for i in range(len(next_features))
            if next_features_status[i]
            ]

        if len(self.features) < self.min_features:
            #Make sure we have enough features
            self.findNewFeatures()

        self.previous, self.current = self.current, self.previous

    def findNewFeatures(self):
        # Avoid sampling new features around current features
        mask = cv.CreateImage(self.size, 8, 1 )
        cv.Set(mask, (255,255,255))

        for feature in self.features:
            cv.Circle(
                mask,
                (int(feature[0]), int(feature[1])),
                int(self.min_distance),
                (0,0,0),
                -1, 8, 0
                )

        new_features = cv.GoodFeaturesToTrack(
            image=self.current,
            eigImage=None,
            tempImage=None,
            cornerCount=self.max_features - len(self.features),
            qualityLevel=self.quality,
            minDistance=self.min_distance,
            mask=mask
            )

        self.features.extend(new_features)

if __name__ == '__main__':
    capture = cv.CaptureFromCAM(-1)

    odometry = VisualOdometry(
        cv.QueryFrame(capture),
        100, 150
        )

    cv.NamedWindow("capture", cv.CV_WINDOW_AUTOSIZE)

    while True:
        img = cv.QueryFrame(capture)
        odometry.followFeatures(img)

        for feature in odometry.features:
            cv.Circle(
                img,
                (int(feature[0]), int(feature[1])),
                2,
                (0, 0, 255),
                -1, 8, 0
                )

        cv.ShowImage('capture', img)
        c = cv.WaitKey(1)
        if (c % 256) == 27:
            # ESC pressed
            break

    del(capture)