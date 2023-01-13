from opendm.video.parameters import Parameters
import datetime
from fractions import Fraction
import io
from math import ceil, floor
import time
import cv2
import os
import collections
from PIL import Image
from checkers import BlackFrameChecker, PercentageBlurChecker, SimilarityChecker, ThresholdBlurChecker
from srtparser import SrtFileParser
import piexif
class Video2Dataset:

    def __init__(self, parameters : Parameters):
        self.parameters = parameters

        self.blur_checker = ThresholdBlurChecker(parameters.blur_threshold) if parameters.blur_threshold is not None else None
        self.similarity_checker = SimilarityChecker(parameters.distance_threshold) if parameters.distance_threshold is not None else None
        self.black_checker = BlackFrameChecker(parameters.black_ratio_threshold, parameters.pixel_black_threshold) if parameters.black_ratio_threshold is not None or parameters.pixel_black_threshold is not None else None

        self.frame_index = parameters.start
        self.f = None


    def ProcessVideo(self):

        start = time.time()

        if (self.parameters.stats_file is not None):
            self.f = open(self.parameters.stats_file, "w")
            self.f.write("global_idx;file_name;frame_index;blur_score;is_blurry;is_black;last_frame_index;similarity_score;is_similar;written\n")

        self.global_idx = 0

        # foreach input file
        for input_file in self.parameters.input:

            # get file name
            file_name = os.path.basename(input_file)
            print("Processing video: {}".format(input_file))

            # get video info
            video_info = get_video_info(input_file)
            print(video_info)

            if self.parameters.use_srt:

                name = os.path.splitext(input_file)[0]

                srt_files = [name + ".srt", name + ".SRT"]
                srt_parser = None

                for srt_file in srt_files:
                    if os.path.exists(srt_file):
                        print("Loading SRT file: {}".format(srt_file))
                        try:
                            srt_parser = SrtFileParser(srt_file)
                            srt_parser.parse()
                            break
                        except Exception as e:
                            print("Error parsing SRT file: {}".format(e))
                            srt_parser = None
            else:
                srt_parser = None

            if (self.blur_checker is not None and self.blur_checker.NeedPreProcess()):
                print("Preprocessing for blur checker...")
                self.blur_checker.PreProcess(input_file, self.parameters.start, self.parameters.end, self.parameters.internal_width, self.parameters.internal_height)
                end = time.time()
                print("Preprocessing time: {:.2f}s".format(end - start))
                print("Calculated threshold is {}".format(self.blur_checker.threshold))

            if (self.black_checker is not None and self.black_checker.NeedPreProcess()):
                start2 = time.time()
                print("Preprocessing for black checker...")
                self.black_checker.PreProcess(input_file, self.parameters.start, self.parameters.end, self.parameters.internal_width, self.parameters.internal_height)
                end = time.time()
                print("Preprocessing time: {:.2f}s".format(end - start2))
                print("Calculated luminance_range_size is {}".format(self.black_checker.luminance_range_size))
                print("Calculated luminance_minimum_value is {}".format(self.black_checker.luminance_minimum_value))
                print("Calculated absolute_threshold is {}".format(self.black_checker.absolute_threshold))

            # open video file
            cap = cv2.VideoCapture(input_file)
            if (not cap.isOpened()):
                print("Error opening video stream or file")
                return

            if (self.parameters.start is not None):
                cap.set(cv2.CAP_PROP_POS_FRAMES, self.parameters.start)
                self.frame_index = self.parameters.start
                start_frame = self.parameters.start
            else:
                start_frame = 0

            frames_to_process = self.parameters.end - start_frame + 1 if (self.parameters.end is not None) else video_info.total_frames - start_frame

            output_file_paths = []

            while (cap.isOpened()):
                ret, frame = cap.read()

                if not ret:
                    break

                if (self.parameters.end is not None and self.frame_index > self.parameters.end):
                    break

                # Calculate progress percentage
                progress = floor((self.frame_index - start_frame + 1) / frames_to_process * 100)
                print("[{}][{:3d}%] Processing frame {}/{}: ".format(file_name, progress, self.frame_index - start_frame + 1, frames_to_process), end="")

                stats = self.ProcessFrame(frame, video_info, srt_parser)

                if stats is not None and self.parameters.stats_file is not None:
                    self.WriteStats(input_file, stats)

                # Add element to array
                if stats is not None and "written" in stats.keys():
                    output_file_paths.append(stats["path"])

            cap.release()

        if self.f is not None:
            self.f.close()

        if self.parameters.limit is not None and self.global_idx >= self.parameters.limit:
            print("Limit of {} frames reached, trimming dataset to {} frames".format(self.parameters.limit, self.global_idx))
            limit_files(output_file_paths, self.parameters.limit)

        end = time.time()
        print("Total processing time: {:.2f}s".format(end - start))


    def ProcessFrame(self, frame, video_info, srt_parser):

        res = {"frame_index": self.frame_index, "global_idx": self.global_idx}

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

        if (self.black_checker is not None):
            is_black = self.black_checker.IsBlack(frame_bw, self.frame_index)
            res["is_black"] = is_black

            if is_black:
                print ("black, skipping")
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

        path = self.SaveFrame(frame, video_info, srt_parser)
        res["written"] = True
        res["path"] = path
        self.frame_index += 1
        self.global_idx += 1

        print ("saved")

        return res

    def SaveFrame(self, frame, video_info, srt_parser: SrtFileParser):

        if (self.parameters.output_resolution is not None):
            frame = cv2.resize(frame, self.parameters.output_resolution)

        path = os.path.join(self.parameters.output,
            "frame_{}_{}.{}".format(self.global_idx, self.frame_index, self.parameters.frame_format))

        _, buf = cv2.imencode('.' + self.parameters.frame_format, frame)

        #start_time_utc = video_info.start_time_utc if video_info.start_time_utc is not None \
        #                 else srt_parser.data[0].timestamp if srt_parser is not None \
        #                 else datetime.datetime.now()

        #elapsed_time_utc = start_time_utc + datetime.timedelta(seconds=(self.frame_index / video_info.frame_rate))
        #elapsed_time = elapsed_time_utc + srt_parser.utc_offset if srt_parser is not None else elapsed_time_utc

        delta = datetime.timedelta(seconds=(self.frame_index / video_info.frame_rate))
        # convert to datetime
        elapsed_time = datetime.datetime(1900, 1, 1) + delta

        img = Image.open(io.BytesIO(buf))

        entry = srt_parser.get_entry(elapsed_time) if srt_parser is not None else None
        elapsed_time_str = (elapsed_time + (datetime.datetime.now() - datetime.datetime(1900, 1, 1))).strftime("%Y:%m:%d %H:%M:%S.%f")

        # Exif dict contains the following keys: '0th', 'Exif', 'GPS', '1st', 'thumbnail'
        # Set the EXIF metadata
        exif_dict = {
            "0th": {
                piexif.ImageIFD.Software: "ODM",
                piexif.ImageIFD.DateTime: elapsed_time_str,
                piexif.ImageIFD.XResolution: (frame.shape[1], 1),
                piexif.ImageIFD.YResolution: (frame.shape[0], 1),
            },
            "Exif": {
                piexif.ExifIFD.DateTimeOriginal: elapsed_time_str,
                piexif.ExifIFD.DateTimeDigitized: elapsed_time_str,
                piexif.ExifIFD.PixelXDimension: (frame.shape[1], 1),
                piexif.ExifIFD.PixelYDimension: (frame.shape[0], 1),
            }}

        if entry is not None:
            segs = entry["shutter"].split("/")
            exif_dict["Exif"][piexif.ExifIFD.ExposureTime] = (int(float(segs[0])), int(float(segs[1])))
            exif_dict["Exif"][piexif.ExifIFD.FocalLength] = (entry["focal_len"], 1)
            exif_dict["Exif"][piexif.ExifIFD.FNumber] = (entry["fnum"], 1)
            exif_dict["Exif"][piexif.ExifIFD.ISOSpeedRatings] = (entry["iso"], 1)

            exif_dict["GPS"] = get_gps_location(elapsed_time, entry["latitude"], entry["longitude"], entry["altitude"])


        exif_bytes = piexif.dump(exif_dict)
        img.save(path, exif=exif_bytes)

        return path


    def WriteStats(self, input_file, stats):
        self.f.write("{};{};{};{};{};{};{};{};{};{}\n".format(
            stats["global_idx"],
            input_file,
            stats["frame_index"],
            stats["blur_score"] if "blur_score" in stats else "",
            stats["is_blurry"] if "is_blurry" in stats else "",
            stats["is_black"] if "is_black" in stats else "",
            stats["last_frame_index"] if "last_frame_index" in stats else "",
            stats["similarity_score"] if "similarity_score" in stats else "",
            stats["is_similar"] if "is_similar" in stats else "",
            stats["written"] if "written" in stats else "").replace(".", ","))


def get_video_info(input_file):

    video = cv2.VideoCapture(input_file)

    total_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    frame_rate = video.get(cv2.CAP_PROP_FPS)

    video.release()

    return collections.namedtuple("VideoInfo", ["total_frames", "frame_rate"])(total_frames, frame_rate)

def float_to_rational(f):
    f = Fraction(f).limit_denominator()
    return (f.numerator, f.denominator)

def limit_files(paths, limit):

    cnt = len(paths)
    num_to_delete = cnt - limit

    if num_to_delete <= 0:
        return

    skip = floor(num_to_delete / limit) if num_to_delete > cnt else ceil(cnt / num_to_delete)

    to_keep = []

    for i in range(len(paths)):
        if i % skip == 0:
            os.remove(paths[i])
        else:
            to_keep.append(paths[i])

    limit_files(to_keep, limit)

def to_deg(value, loc):
    """convert decimal coordinates into degrees, munutes and seconds tuple
    Keyword arguments: value is float gps-value, loc is direction list ["S", "N"] or ["W", "E"]
    return: tuple like (25, 13, 48.343 ,'N')
    """
    if value < 0:
        loc_value = loc[0]
    elif value > 0:
        loc_value = loc[1]
    else:
        loc_value = ""
    abs_value = abs(value)
    deg =  int(abs_value)
    t1 = (abs_value-deg)*60
    min = int(t1)
    sec = round((t1 - min)* 60, 5)
    return (deg, min, sec, loc_value)

def get_gps_location(elapsed_time, lat, lng, altitude):

    lat_deg = to_deg(lat, ["S", "N"])
    lng_deg = to_deg(lng, ["W", "E"])

    exiv_lat = (float_to_rational(lat_deg[0]), float_to_rational(lat_deg[1]), float_to_rational(lat_deg[2]))
    exiv_lng = (float_to_rational(lng_deg[0]), float_to_rational(lng_deg[1]), float_to_rational(lng_deg[2]))

    gps_ifd = {
        piexif.GPSIFD.GPSVersionID: (2, 0, 0, 0),
        piexif.GPSIFD.GPSDateStamp: elapsed_time.strftime('%Y:%m:%d %H:%M:%S.%f')
    }

    if altitude is not None:
        gps_ifd[piexif.GPSIFD.GPSAltitudeRef] = 0
        gps_ifd[piexif.GPSIFD.GPSAltitude] = float_to_rational(round(altitude))

    if lat is not None:
        gps_ifd[piexif.GPSIFD.GPSLatitudeRef] = lat_deg[3]
        gps_ifd[piexif.GPSIFD.GPSLatitude] = exiv_lat

    if lng is not None:
        gps_ifd[piexif.GPSIFD.GPSLongitudeRef] = lng_deg[3]
        gps_ifd[piexif.GPSIFD.GPSLongitude] = exiv_lng

    return gps_ifd