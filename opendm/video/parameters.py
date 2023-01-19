
import argparse
import datetime
import os

class Parameters:

    def __init__(self, args):

        # "input" -> path to input video file(s), use ',' to separate multiple files")
        # "output" -> path to output directory")
        # "start" -> start frame index")
        # "end" -> end frame index")
        # "output-resolution" -> Override output resolution (ex. 1024)")
        # "blur-threshold" -> blur measures that fall below this value will be considered 'blurry'. Good value is 300
        # "distance-threshold" -> distance measures that fall below this value will be considered 'similar'")
        # "black-ratio-threshold" -> Set the threshold for considering a frame 'black'. Express the minimum value for the ratio: nb_black_pixels / nb_pixels. Default value is 0.98")
        # "pixel-black-threshold" -> Set the threshold for considering a pixel 'black'. The threshold expresses the maximum pixel luminance value for which a pixel is considered 'black'. Good value is 0.30 (30%)")
        # "use-srt" -> Use SRT files for extracting metadata (same name as video file with .srt extension)")
        # "limit" -> Maximum number of output frames
        # "frame-format" -> frame format (jpg, png, tiff, etc.)")
        # "stats-file" -> Save statistics to csv file")

        if not os.path.exists(args["output"]):
            os.makedirs(args["output"])

        self.input = args["input"]
        if isinstance(self.input, str):
            self.input = [self.input]

        self.output = args["output"]
        self.start = args.get("start", 0)
        self.end = args.get("end", None)
        self.limit = args.get("limit", None)
        self.blur_threshold = args.get("blur_threshold", None)
        self.distance_threshold = args.get("distance_threshold", None)
        self.black_ratio_threshold = args.get("black_ratio_threshold", None)
        self.pixel_black_threshold = args.get("pixel_black_threshold", None)
        self.use_srt = "use_srt" in args
        self.frame_format = args.get("frame_format", "jpg")
        self.max_dimension = args.get("max_dimension", None)

        self.stats_file = args.get("stats_file", None)

        # We will resize the image to this size before processing
        self.internal_resolution = 800
