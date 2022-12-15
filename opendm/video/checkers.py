import cv2
import numpy as np

class PercentageBlurChecker:
    def __init__(self, percentage):
        self.percentage = percentage
        self.cache = None

    def NeedPreProcess(self):
        return True

    def PreProcess(self, video_path, start_frame, end_frame, width=800, height=600):

        # Open video file
        cap = cv2.VideoCapture(video_path)
        if (cap.isOpened() == False):
            print("Error opening video stream or file")
            return

        if start_frame is not None:
            cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

        tmp = []

        frame_index = start_frame if start_frame is not None else 0
        while (cap.isOpened() and (end_frame is None or frame_index <= end_frame)):

            ret, frame = cap.read()

            if not ret:
                break

            frame_bw = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame_bw = cv2.resize(frame_bw, (width, height))
            var = cv2.Laplacian(frame_bw, cv2.CV_64F).var()

            tmp.append(var)

            frame_index += 1

        cap.release()

        # Calculate threshold
        self.threshold = np.percentile(tmp, self.percentage * 100)
        start_frame = start_frame if start_frame is not None else 0

        # Fill cache with frame blur scores and indices
        self.cache = {i + start_frame: v for i, v in enumerate(tmp)}

        return

    def IsBlur(self, image_bw, id):

        if self.cache is None:
            return 0, True

        if id not in self.cache:
            return 0, True

        return self.cache[id], self.cache[id] < self.threshold

class ThresholdBlurChecker:
    def __init__(self, threshold):
        self.threshold = threshold

    def NeedPreProcess(self):
        return False

    def PreProcess(self, video_path, start_frame, end_frame, width=800, height=600):
        return

    def IsBlur(self, image_bw, id):
        var = cv2.Laplacian(image_bw, cv2.CV_64F).var()
        return var, var < self.threshold

class SimilarityChecker:
    def __init__(self, threshold, max_features=500):
        self.threshold = threshold
        self.max_features = max_features
        self.last_image = None
        self.last_image_id = None
        self.last_image_features = None

    def IsSimilar(self, image_bw, id):

        if self.last_image is None:
            self.last_image = image_bw
            self.last_image_id = id
            self.last_image_features = cv2.goodFeaturesToTrack(image_bw, self.max_features, 0.01, 10)
            return 0, False, None

        # Detect features
        features, status, _ = cv2.calcOpticalFlowPyrLK(self.last_image, image_bw, self.last_image_features, None)

        # Filter out the "bad" features (i.e. those that are not tracked successfully)
        good_features = features[status == 1]
        good_features2 = self.last_image_features[status == 1]

        # Calculate the difference between the locations of the good features in the two frames
        distance = np.average(np.abs(good_features2 - good_features))

        res = distance < self.threshold

        if (not res):
            self.last_image = image_bw
            self.last_image_id = id
            self.last_image_features = cv2.goodFeaturesToTrack(image_bw, self.max_features, 0.01, 10)

        return distance, res, self.last_image_id
