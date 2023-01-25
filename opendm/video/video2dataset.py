import datetime
from fractions import Fraction
import io
from math import ceil, floor
import time
import cv2
import os
import collections
from PIL import Image
import numpy as np
import piexif
from opendm import log
from opendm.video.srtparser import SrtFileParser
from opendm.video.parameters import Parameters
from opendm.video.checkers import BlackFrameChecker, SimilarityChecker, ThresholdBlurChecker

class Video2Dataset:

    def __init__(self, parameters : Parameters):
        self.parameters = parameters

        self.blur_checker = ThresholdBlurChecker(parameters.blur_threshold) if parameters.blur_threshold is not None else None
        self.similarity_checker = SimilarityChecker(parameters.distance_threshold) if parameters.distance_threshold is not None else None
        self.black_checker = BlackFrameChecker(parameters.black_ratio_threshold, parameters.pixel_black_threshold) if parameters.black_ratio_threshold is not None or parameters.pixel_black_threshold is not None else None

        self.frame_index = parameters.start
        self.f = None


    def ProcessVideo(self):
        self.date_now = None
        start = time.time()

        if (self.parameters.stats_file is not None):
            self.f = open(self.parameters.stats_file, "w")
            self.f.write("global_idx;file_name;frame_index;blur_score;is_blurry;is_black;last_frame_index;similarity_score;is_similar;written\n")

        self.global_idx = 0

        output_file_paths = []
        
        # foreach input file
        for input_file in self.parameters.input:
            # get file name
            file_name = os.path.basename(input_file)
            log.ODM_INFO("Processing video: {}".format(input_file))

            # get video info
            video_info = get_video_info(input_file)
            log.ODM_INFO(video_info)

            # Set pseudo start time
            if self.date_now is None:
                try:
                    self.date_now = datetime.datetime.fromtimestamp(os.path.getmtime(input_file))
                except:
                    self.date_now = datetime.datetime.now()
            else:
                self.date_now += datetime.timedelta(seconds=video_info.total_frames / video_info.frame_rate)
            
            log.ODM_INFO("Use pseudo start time: %s" % self.date_now)

            if self.parameters.use_srt:

                name = os.path.splitext(input_file)[0]

                srt_files = [name + ".srt", name + ".SRT"]
                srt_parser = None

                for srt_file in srt_files:
                    if os.path.exists(srt_file):
                        log.ODM_INFO("Loading SRT file: {}".format(srt_file))
                        try:
                            srt_parser = SrtFileParser(srt_file)
                            srt_parser.parse()
                            break
                        except Exception as e:
                            log.ODM_INFO("Error parsing SRT file: {}".format(e))
                            srt_parser = None
            else:
                srt_parser = None

            if (self.black_checker is not None and self.black_checker.NeedPreProcess()):
                start2 = time.time()
                log.ODM_INFO("Preprocessing for black frame checker... this might take a bit")
                self.black_checker.PreProcess(input_file, self.parameters.start, self.parameters.end)
                end = time.time()
                log.ODM_INFO("Preprocessing time: {:.2f}s".format(end - start2))
                log.ODM_INFO("Calculated luminance_range_size is {}".format(self.black_checker.luminance_range_size))
                log.ODM_INFO("Calculated luminance_minimum_value is {}".format(self.black_checker.luminance_minimum_value))
                log.ODM_INFO("Calculated absolute_threshold is {}".format(self.black_checker.absolute_threshold))

            # open video file
            cap = cv2.VideoCapture(input_file)
            if (not cap.isOpened()):
                log.ODM_INFO("Error opening video stream or file")
                return

            if (self.parameters.start is not None):
                cap.set(cv2.CAP_PROP_POS_FRAMES, self.parameters.start)
                self.frame_index = self.parameters.start
                start_frame = self.parameters.start
            else:
                start_frame = 0

            frames_to_process = self.parameters.end - start_frame + 1 if (self.parameters.end is not None) else video_info.total_frames - start_frame

            progress = 0
            while (cap.isOpened()):
                ret, frame = cap.read()

                if not ret:
                    break

                if (self.parameters.end is not None and self.frame_index > self.parameters.end):
                    break

                # Calculate progress percentage
                prev_progress = progress
                progress = floor((self.frame_index - start_frame + 1) / frames_to_process * 100)
                if progress != prev_progress:
                    print("[{}][{:3d}%] Processing frame {}/{}: ".format(file_name, progress, self.frame_index - start_frame + 1, frames_to_process), end="\r")

                stats = self.ProcessFrame(frame, video_info, srt_parser)

                if stats is not None and self.parameters.stats_file is not None:
                    self.WriteStats(input_file, stats)

                # Add element to array
                if stats is not None and "written" in stats.keys():
                    output_file_paths.append(stats["path"])

            cap.release()

        if self.f is not None:
            self.f.close()

        if self.parameters.limit is not None and self.parameters.limit > 0 and self.global_idx >= self.parameters.limit:
            log.ODM_INFO("Limit of {} frames reached, trimming dataset".format(self.parameters.limit))
            output_file_paths = limit_files(output_file_paths, self.parameters.limit)

        end = time.time()
        log.ODM_INFO("Total processing time: {:.2f}s".format(end - start))
        return output_file_paths


    def ProcessFrame(self, frame, video_info, srt_parser):

        res = {"frame_index": self.frame_index, "global_idx": self.global_idx}

        frame_bw = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        h, w = frame_bw.shape
        resolution = self.parameters.internal_resolution
        if resolution < w or resolution < h:
            m = max(w, h)
            factor = resolution / m
            frame_bw = cv2.resize(frame_bw, (int(ceil(w * factor)), int(ceil(h * factor))), interpolation=cv2.INTER_NEAREST)

        if (self.blur_checker is not None):
            blur_score, is_blurry = self.blur_checker.IsBlur(frame_bw, self.frame_index)
            res["blur_score"] = blur_score
            res["is_blurry"] = is_blurry

            if is_blurry:
                # print ("blurry, skipping")
                self.frame_index += 1
                return res

        if (self.black_checker is not None):
            is_black = self.black_checker.IsBlack(frame_bw, self.frame_index)
            res["is_black"] = is_black

            if is_black:
                # print ("black, skipping")
                self.frame_index += 1
                return res

        if (self.similarity_checker is not None):
            similarity_score, is_similar, last_frame_index = self.similarity_checker.IsSimilar(frame_bw, self.frame_index)
            res["similarity_score"] = similarity_score
            res["is_similar"] = is_similar
            res["last_frame_index"] = last_frame_index

            if is_similar:
                # print ("similar to {}, skipping".format(self.similarity_checker.last_image_id))
                self.frame_index += 1
                return res

        path = self.SaveFrame(frame, video_info, srt_parser)
        res["written"] = True
        res["path"] = path
        self.frame_index += 1
        self.global_idx += 1

        return res

    def SaveFrame(self, frame, video_info, srt_parser: SrtFileParser):
        max_dim = self.parameters.max_dimension
        if max_dim is not None:
            h, w, _ = frame.shape
            if max_dim < w or max_dim < h:
                m = max(w, h)
                factor = max_dim / m
                frame = cv2.resize(frame, (int(ceil(w * factor)), int(ceil(h * factor))), interpolation=cv2.INTER_AREA)

        path = os.path.join(self.parameters.output,
            "{}_{}_{}.{}".format(video_info.basename, self.global_idx, self.frame_index, self.parameters.frame_format))

        _, buf = cv2.imencode('.' + self.parameters.frame_format, frame)

        delta = datetime.timedelta(seconds=(self.frame_index / video_info.frame_rate))
        elapsed_time = datetime.datetime(1900, 1, 1) + delta

        img = Image.open(io.BytesIO(buf))
        
        entry = gps_coords = None
        if srt_parser is not None:
            entry = srt_parser.get_entry(elapsed_time)
            gps_coords = srt_parser.get_gps(elapsed_time)

        exif_time = (elapsed_time + (self.date_now - datetime.datetime(1900, 1, 1)))
        elapsed_time_str = exif_time.strftime("%Y:%m:%d %H:%M:%S")
        subsec_time_str = exif_time.strftime("%f")

        # Exif dict contains the following keys: '0th', 'Exif', 'GPS', '1st', 'thumbnail'
        # Set the EXIF metadata
        exif_dict = {
            "0th": {
                piexif.ImageIFD.Software: "ODM",
                piexif.ImageIFD.DateTime: elapsed_time_str,
                piexif.ImageIFD.XResolution: (frame.shape[1], 1),
                piexif.ImageIFD.YResolution: (frame.shape[0], 1),
                piexif.ImageIFD.Make: "DJI" if video_info.basename.lower().startswith("dji") else "Unknown",
                piexif.ImageIFD.Model: "Unknown"
            },
            "Exif": {
                piexif.ExifIFD.DateTimeOriginal: elapsed_time_str,
                piexif.ExifIFD.DateTimeDigitized: elapsed_time_str,
                piexif.ExifIFD.SubSecTime: subsec_time_str,
                piexif.ExifIFD.PixelXDimension: frame.shape[1],
                piexif.ExifIFD.PixelYDimension: frame.shape[0],
            }}

        if entry is not None:
            if entry["shutter"] is not None:
                exif_dict["Exif"][piexif.ExifIFD.ExposureTime] = (1, int(entry["shutter"]))
            if entry["focal_len"] is not None:
                exif_dict["Exif"][piexif.ExifIFD.FocalLength] = (entry["focal_len"], 100)
            if entry["fnum"] is not None:
                exif_dict["Exif"][piexif.ExifIFD.FNumber] = float_to_rational(entry["fnum"])
            if entry["iso"] is not None:
                exif_dict["Exif"][piexif.ExifIFD.ISOSpeedRatings] = entry["iso"]
        
        if gps_coords is not None:
            exif_dict["GPS"] = get_gps_location(elapsed_time, gps_coords[1], gps_coords[0], gps_coords[2])

        exif_bytes = piexif.dump(exif_dict)
        img.save(path, exif=exif_bytes, quality=95)

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
    basename = os.path.splitext(os.path.basename(input_file))[0]

    total_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    frame_rate = video.get(cv2.CAP_PROP_FPS)

    video.release()

    return collections.namedtuple("VideoInfo", ["total_frames", "frame_rate", "basename"])(total_frames, frame_rate, basename)

def float_to_rational(f):
    f = Fraction(f).limit_denominator()
    return (f.numerator, f.denominator)

def limit_files(paths, limit):
    if len(paths) <= limit:
        return paths
    
    to_keep = []
    all_idxes = np.arange(0, len(paths))
    keep_idxes = np.linspace(0, len(paths) - 1, limit, dtype=int)
    remove_idxes = set(all_idxes) - set(keep_idxes)

    p = np.array(paths)
    to_keep = list(p[keep_idxes])

    for idx in remove_idxes:
        os.remove(paths[idx])

    return to_keep

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
        piexif.GPSIFD.GPSDateStamp: elapsed_time.strftime('%Y:%m:%d')
    }

    if lat is not None and lng is not None:
        gps_ifd[piexif.GPSIFD.GPSLatitudeRef] = lat_deg[3]
        gps_ifd[piexif.GPSIFD.GPSLatitude] = exiv_lat
        gps_ifd[piexif.GPSIFD.GPSLongitudeRef] = lng_deg[3]
        gps_ifd[piexif.GPSIFD.GPSLongitude] = exiv_lng
        if altitude is not None:
            gps_ifd[piexif.GPSIFD.GPSAltitudeRef] = 0
            gps_ifd[piexif.GPSIFD.GPSAltitude] = float_to_rational(round(altitude))

    return gps_ifd