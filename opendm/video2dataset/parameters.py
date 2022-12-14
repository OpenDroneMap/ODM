
import argparse
import os

class Parameters:
    def __init__(self):

        ap = argparse.ArgumentParser()
        ap.add_argument("-i", "--input", required=True, help="path to input video file")
        ap.add_argument("-o", "--output", required=True, help="path to output directory")
        ap.add_argument("-s", "--start", type=int, help="start frame index")
        ap.add_argument("-e", "--end", type=int, help="end frame index")
        ap.add_argument("-or", "--output-resolution", type=str, help="Override output resolution (ex. 640x480)")
        ap.add_argument("-bp", "--blur-percentage", type=float, help="discard the lowest X percent of frames based on blur score (allowed values from 0.0 to 1.0)")
        ap.add_argument("-bt", "--blur-threshold", type=float, help="blur measures that fall below this value will be considered 'blurry' (to be used in exclusion with -bp)")
        ap.add_argument("-dt", "--distance-threshold", type=float, help="distance measures that fall below this value will be considered 'similar'")
        ap.add_argument("-ff", "--frame-format", type=str, default="jpg", help="frame format (jpg, png, tiff, etc.)")
        ap.add_argument("-st", "--stats-file", type=str, help="Save statistics to csv file")
        args = vars(ap.parse_args())

        if (not self.ValidateParameters(args)):
            print("Invalid parameters")
            exit()

        if not os.path.exists(args["output"]):
            os.makedirs(args["output"])

        self.input = args["input"]
        self.output = args["output"]
        self.start = args["start"] if args["start"] else 0
        self.end = args["end"] if args["end"] else None
        self.blur_percentage = args["blur_percentage"] if args["blur_percentage"] else None
        self.blur_threshold = args["blur_threshold"] if args["blur_threshold"] else None
        self.distance_threshold = args["distance_threshold"] if args["distance_threshold"] else None
        self.frame_format = args["frame_format"]
        self.output_resolution = tuple(map(int, args["output_resolution"].split("x"))) if args["output_resolution"] else None
        self.stats_file = args["stats_file"] if args["stats_file"] else None

        # We will resize the image to this size before processing
        self.internal_width = 800
        self.internal_height = 600

    # Validate parameters
    def ValidateParameters(self, args):

        if not os.path.exists(args["input"]):
            print("Input file does not exist")
            return False

        if not os.path.exists(args["output"]):
            os.makedirs(args["output"])

        if args["start"] and args["start"] < 0:
            print("Start frame index must be greater than 0")
            return False

        if args["end"]:
            if args["end"] < 0:
                print("End frame index must be greater than 0")
                return False

            if args["end"] < args["start"]:
                print("End frame index must be greater than start frame index")
                return False

        if args["blur_percentage"] and (args["blur_percentage"] < 0 or args["blur_percentage"] > 1):
            print("Blur percentage must be in the range 0.0 to 1.0")
            return False

        if args["blur_threshold"] and args["blur_threshold"] < 0:
            print("Blur threshold must be greater than 0")
            return False

        if args["distance_threshold"] and args["distance_threshold"] < 0:
            print("Distance threshold must be greater than 0")
            return False

        if args["output_resolution"]:
            segs = args["output_resolution"].split("x")
            if (len(segs) != 2):
                print("Output resolution must be in the format WxH")
                return False

            if (int(segs[0]) <= 0 or int(segs[1]) <= 0):
                print("Output resolution must be in the format WxH")
                return False

        return True

