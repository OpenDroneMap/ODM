import cv2
import numpy as np

class ThresholdBlurChecker:
    def __init__(self, threshold):
        self.threshold = threshold

    def NeedPreProcess(self):
        return False

    def PreProcess(self, video_path, start_frame, end_frame):
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


class NaiveBlackFrameChecker:
    def __init__(self, threshold):
        self.threshold = threshold

    def PreProcess(self, video_path, start_frame, end_frame, width=800, height=600):
        return

    def NeedPreProcess(self):
        return False

    def IsBlack(self, image_bw, id):
        return np.average(image_bw) < self.threshold


class BlackFrameChecker:
    def __init__(self, picture_black_ratio_th=0.98, pixel_black_th=0.30):
        self.picture_black_ratio_th = picture_black_ratio_th if picture_black_ratio_th is not None else 0.98
        self.pixel_black_th = pixel_black_th if pixel_black_th is not None else 0.30
        self.luminance_minimum_value = None
        self.luminance_range_size = None
        self.absolute_threshold = None

    def NeedPreProcess(self):
        return True

    def PreProcess(self, video_path, start_frame, end_frame):
        # Open video file
        cap = cv2.VideoCapture(video_path)

        # Set frame start and end indices
        cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
        frame_end = end_frame
        if end_frame == -1:
            frame_end = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

        # Initialize luminance range size and minimum value
        self.luminance_range_size = 0
        self.luminance_minimum_value = 255

        frame_index = start_frame if start_frame is not None else 0

        # Read and process frames from video file
        while (cap.isOpened() and (end_frame is None or frame_index <= end_frame)):

            ret, frame = cap.read()
            if not ret:
                break

            # Convert frame to grayscale
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray_frame_min = gray_frame.min()
            gray_frame_max = gray_frame.max()

            # Update luminance range size and minimum value
            self.luminance_range_size = max(self.luminance_range_size, gray_frame_max - gray_frame_min)
            self.luminance_minimum_value = min(self.luminance_minimum_value, gray_frame_min)

            frame_index += 1

        # Calculate absolute threshold for considering a pixel "black"
        self.absolute_threshold = self.luminance_minimum_value + self.pixel_black_th * self.luminance_range_size

        # Close video file
        cap.release()

    def IsBlack(self, image_bw, id):

        # Count number of pixels < self.absolute_threshold
        nb_black_pixels = np.sum(image_bw < self.absolute_threshold)

        # Calculate ratio of black pixels
        ratio_black_pixels = nb_black_pixels / (image_bw.shape[0] * image_bw.shape[1])

        # Check if ratio of black pixels is above threshold
        return ratio_black_pixels >= self.picture_black_ratio_th