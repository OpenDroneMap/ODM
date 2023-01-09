
import argparse
import datetime
import os

class Parameters:

    input = None
    output = None
    start = None
    end = None
    output_resolution = None
    blur_percentage = None
    blur_threshold = None
    distance_threshold = None
    black_ratio_threshold = None
    pixel_black_threshold = None
    use_srt = None
    timezone = None
    frame_format = None
    stats_file = None

    def __init__(self, args):

        # "input" -> path to input video file(s), use ',' to separate multiple files")
        # "output" -> path to output directory")
        # "start" -> start frame index")
        # "end" -> end frame index")
        # "output-resolution" -> Override output resolution (ex. 640x480)")
        # "blur-percentage" -> discard the lowest X percent of frames based on blur score (allowed values from 0.0 to 1.0)")
        # "blur-threshold" -> blur measures that fall below this value will be considered 'blurry' (to be used in exclusion with --blur-percentage)")
        # "distance-threshold" -> distance measures that fall below this value will be considered 'similar'")
        # "black-ratio-threshold" -> Set the threshold for considering a frame 'black'. Express the minimum value for the ratio: nb_black_pixels / nb_pixels. Default value is 0.98")
        # "pixel-black-threshold" -> Set the threshold for considering a pixel 'black'. The threshold expresses the maximum pixel luminance value for which a pixel is considered 'black'. Good value is 0.30 (30%)")
        # "use-srt" -> Use SRT files for extracting metadata (same name as video file with .srt extension)")
        # "timezone" -> UTC timezone offset (ex. -5 for EST). Default to local timezone")
        # "frame-format" -> frame format (jpg, png, tiff, etc.)")
        # "stats-file" -> Save statistics to csv file")

        if (not self.ValidateParameters(args)):
            print("Invalid parameters")
            exit()

        if not os.path.exists(args["output"]):
            os.makedirs(args["output"])

        self.input = args["input"].split(",")
        self.output = args["output"]

        self.start = args["start"] if args["start"] else 0
        self.end = args["end"] if args["end"] else None

        self.blur_percentage = args["blur_percentage"] if args["blur_percentage"] else None
        self.blur_threshold = args["blur_threshold"] if args["blur_threshold"] else None

        self.distance_threshold = args["distance_threshold"] if args["distance_threshold"] else None

        self.black_ratio_threshold = args["black_ratio_threshold"] if args["black_ratio_threshold"] else None
        self.pixel_black_threshold = args["pixel_black_threshold"] if args["pixel_black_threshold"] else None

        self.use_srt = args["use_srt"]

        self.utc_offset = datetime.timedelta(hours=int(args["timezone"])) if args["timezone"] != "local" else datetime.datetime.now() - datetime.datetime.utcnow()

        self.frame_format = args["frame_format"]
        self.output_resolution = tuple(map(int, args["output_resolution"].split("x"))) if args["output_resolution"] else None

        self.stats_file = args["stats_file"] if args["stats_file"] else None

        # We will resize the image to this size before processing
        self.internal_width = 800
        self.internal_height = 600

    # Validate parameters
    def ValidateParameters(self, args):

        # input can be a list of files comma separated, check for each file
        files = args["input"].split(",")
        for file in files:
            if not os.path.exists(file):
                print("Input file does not exist: " + file)
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

        if args["timezone"] and args["timezone"] != "local":
            try:
                val = int(args["timezone"])
                if val < -12 or val > 14:
                    print("Timezone must be in the range -12 to 14")
                    return False
            except ValueError:
                print("Timezone must be a valid integer")
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

        if args["black_ratio_threshold"] and (args["black_ratio_threshold"] < 0 or args["black_ratio_threshold"] > 1):
            print("Black ratio threshold must be in the range 0.0 to 1.0")
            return False

        if args["pixel_black_threshold"] and (args["pixel_black_threshold"] < 0 or args["pixel_black_threshold"] > 1):
            print("Pixel black threshold must be in the range 0.0 to 1.0")
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

