from math import floor
import time
import cv2
import os

from .checkers import PercentageBlurChecker, SimilarityChecker, ThresholdBlurChecker

# Input parameters:
#  input = path to input video file
#  output = path to output directory
#  start = start frame index
#  end = end frame index
#  output_resolution = Override output resolution (ex. 640x480)
#  blur_percentage = discard the lowest X percent of frames based on blur score (allowed values from 0.0 to 1.0)
#  blur_threshold = blur measures that fall below this value will be considered 'blurry' (to be used in exclusion with -bp)
#  distance_threshold = distance measures that fall below this value will be considered 'similar'
#  frame_format = frame format (jpg, png, tiff, etc.)
#  stats_file = Save statistics to csv file
#  internal_width = We will resize the image to this width before processing
#  internal_height = We will resize the image to this height before processing

class Parameters:

    def __init__(self):

        self.input = None
        self.output = None
        self.start = 0
        self.end = None
        self.blur_percentage = None
        self.blur_threshold = None
        self.distance_threshold = None
        self.frame_format = "jpg"
        self.output_resolution = None
        self.stats_file = None

        # We will resize the image to this size before processing
        self.internal_width = 800
        self.internal_height = 600


class Video2Dataset:

    def __init__(self, parameters):

        if not self.ValidateParameters(parameters):
            raise Exception("Invalid parameters")

        self.parameters = parameters

        # We prioritize blur threshold over blur percentage.
        if parameters.blur_threshold is not None:
            self.blur_checker = ThresholdBlurChecker(parameters.blur_threshold)
        else:
            if parameters.blur_percentage is not None:
                self.blur_checker = PercentageBlurChecker(parameters.blur_percentage)
            else:
                self.blur_checker = None

        self.similarity_checker = SimilarityChecker(parameters.distance_threshold) if parameters.distance_threshold is not None else None
        self.frame_index = parameters.start
        self.orientation = None
        self.f = None

    # Validate parameters
    def ValidateParameters(self, args):

        if not os.path.exists(args.input):
            print("Input file does not exist")
            return False

        if not os.path.exists(args.output):
            os.makedirs(args.output)

        if args.start and args.start < 0:
            print("Start frame index must be greater than 0")
            return False

        if args.end:
            if args.end < 0:
                print("End frame index must be greater than 0")
                return False

            if args.end < args.start:
                print("End frame index must be greater than start frame index")
                return False

        if args.blur_percentage and (args.blur_percentage < 0 or args.blur_percentage > 1):
            print("Blur percentage must be in the range 0.0 to 1.0")
            return False

        if args.blur_threshold and args.blur_threshold < 0:
            print("Blur threshold must be greater than 0")
            return False

        if args.distance_threshold and args.distance_threshold < 0:
            print("Distance threshold must be greater than 0")
            return False

        if args.output_resolution:
            segs = args.output_resolution.split("x")
            if (len(segs) != 2):
                print("Output resolution must be in the format WxH")
                return False

            if (int(segs[0]) <= 0 or int(segs[1]) <= 0):
                print("Output resolution must be in the format WxH")
                return False

        return True

    def ProcessVideo(self):

        start = time.time()

        if (self.blur_checker is not None and self.blur_checker.NeedPreProcess()):
            print("Preprocessing video...")
            self.blur_checker.PreProcess(self.parameters.input, self.parameters.start, self.parameters.end, self.parameters.internal_width, self.parameters.internal_height)
            end = time.time()
            print("Preprocessing time: {:.2f}s".format(end - start))
            print("Calculated threshold is {}".format(self.blur_checker.threshold))

        # open video file
        cap = cv2.VideoCapture(self.parameters.input)
        if (cap.isOpened() == False):
            print("Error opening video stream or file")
            return

        frames_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

        if (self.parameters.stats_file is not None):
            self.f = open(self.parameters.stats_file, "w")
            if (self.blur_checker is not None):
                self.f.write("blur_threshold;{}\n\n".format(self.blur_checker.threshold).replace(".", ","))
            self.f.write("frame_index;blur_score;is_blurry;last_frame_index;similarity_score;is_similar;written\n")

        if (self.parameters.start is not None):
            cap.set(cv2.CAP_PROP_POS_FRAMES, self.parameters.start)
            self.frame_index = self.parameters.start
            start_frame = self.parameters.start
        else:
            start_frame = 0

        frames_to_process = self.parameters.end - start_frame + 1 if (self.parameters.end is not None) else frames_count - start_frame

        while (cap.isOpened()):
            ret, frame = cap.read()

            if not ret:
                break

            if (self.parameters.end is not None and self.frame_index > self.parameters.end):
                break

            # Calculate progress percentage
            progress = floor((self.frame_index - start_frame + 1) / frames_to_process * 100)
            print("[{:3d}%] Processing frame {}/{}: ".format(progress, self.frame_index - start_frame + 1, frames_to_process), end="")

            stats = self.ProcessFrame(frame)

            if stats is not None and self.parameters.stats_file is not None:
                self.WriteStats(stats)

        cap.release()
        if (self.f is not None):
            self.f.close()

        end = time.time()
        print("Total processing time: {:.2f}s".format(end - start))


    def ProcessFrame(self, frame):

        res = {"frame_index": self.frame_index}

        frame_bw = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_bw = cv2.resize(frame_bw, (self.parameters.internal_width, self.parameters.internal_height))

        if (self.blur_checker is not None):
            blur_score, is_blurry = self.blur_checker.IsBlur(frame_bw, self.frame_index)
            res["blur_score"] = blur_score
            res["is_blurry"] = is_blurry

            if is_blurry:
                print ("blurry, skipping")
                self.frame_index += 1
                return res

        if (self.similarity_checker is not None):
            similarity_score, is_similar, last_frame_index = self.similarity_checker.IsSimilar(frame_bw, self.frame_index)
            res["similarity_score"] = similarity_score
            res["is_similar"] = is_similar
            res["last_frame_index"] = last_frame_index

            if is_similar:
                print ("similar to {}, skipping".format(self.similarity_checker.last_image_id))
                self.frame_index += 1
                return res

        self.SaveFrame(frame)
        res["written"] = True
        self.frame_index += 1

        print ("saved")

        return res

    def SaveFrame(self, frame):

        if (self.parameters.output_resolution is not None):
            frame = cv2.resize(frame, self.parameters.output_resolution)

        cv2.imwrite(os.path.join(self.parameters.output,
            "frame_{}.{}".format(self.frame_index, self.parameters.frame_format)), frame)

    def WriteStats(self, stats):
        self.f.write("{};{};{};{};{};{};{}\n".format(stats["frame_index"],
            stats["blur_score"] if "blur_score" in stats else "",
            stats["is_blurry"] if "is_blurry" in stats else "",
            stats["last_frame_index"] if "last_frame_index" in stats else "",
            stats["similarity_score"] if "similarity_score" in stats else "",
            stats["is_similar"] if "is_similar" in stats else "",
            stats["written"] if "written" in stats else "").replace(".", ","))