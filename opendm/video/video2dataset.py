from opendm.video.parameters import Parameters
import datetime
from fractions import Fraction
import io
from math import floor
import time
import cv2
import os
import pymediainfo
import collections
from PIL import Image
from checkers import BlackFrameChecker, PercentageBlurChecker, SimilarityChecker, ThresholdBlurChecker
from srtparser import SrtFileParser
import piexif

class Video2Dataset:

    def __init__(self, parameters : Parameters):
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
            video_info = self.GetVideoInfo(input_file)
            print(video_info)

            utc_offset = self.parameters.utc_offset if self.parameters.utc_offset is not None else video_info.utc_offset

            if self.parameters.use_srt:
                srt_file = os.path.splitext(input_file)[0] + ".srt"
                if os.path.exists(srt_file):
                    print("Loading SRT file: {}".format(srt_file))
                    srt_parser = SrtFileParser(srt_file, utc_offset)
                    srt_parser.parse()
                else:
                    srt_file = os.path.splitext(input_file)[0] + ".SRT"
                    if os.path.exists(srt_file):
                        print("Loading SRT file: {}".format(srt_file))
                        srt_parser = SrtFileParser(srt_file, utc_offset)
                        srt_parser.parse()
                    else:
                        print("SRT file not found: {}".format(srt_file))
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

            frames_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

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
                print("[{}][{:3d}%] Processing frame {}/{}: ".format(file_name, progress, self.frame_index - start_frame + 1, frames_to_process), end="")

                stats = self.ProcessFrame(frame, video_info, srt_parser)

                if stats is not None and self.parameters.stats_file is not None:
                    self.WriteStats(input_file, stats)

            cap.release()

        if self.f is not None:
            self.f.close()

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

        self.SaveFrame(frame, video_info, srt_parser)
        res["written"] = True
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

        start_time_utc = video_info.start_time_utc if video_info.start_time_utc is not None \
                         else srt_parser.data[0].timestamp if srt_parser is not None \
                         else datetime.datetime.now()

        elapsed_time_utc = start_time_utc + datetime.timedelta(seconds=(self.frame_index / video_info.frame_rate))
        elapsed_time = elapsed_time_utc + srt_parser.utc_offset

        img = Image.open(io.BytesIO(buf))

        # Exif dict contains the following keys: '0th', 'Exif', 'GPS', '1st', 'thumbnail'

        # Set the EXIF metadata
        exif_dict = {
            "0th": {
                piexif.ImageIFD.Software: "ODM",
                piexif.ImageIFD.DateTime: elapsed_time.strftime('%Y:%m:%d %H:%M:%S'),
                piexif.ImageIFD.XResolution: (frame.shape[1], 1),
                piexif.ImageIFD.YResolution: (frame.shape[0], 1),
            },
            "Exif": {
                piexif.ExifIFD.DateTimeOriginal: elapsed_time.strftime('%Y:%m:%d %H:%M:%S'),
                piexif.ExifIFD.DateTimeDigitized: elapsed_time.strftime('%Y:%m:%d %H:%M:%S'),
                piexif.ExifIFD.PixelXDimension: (frame.shape[1], 1),
                piexif.ExifIFD.PixelYDimension: (frame.shape[0], 1),
            }}

        if video_info.orientation is not None:
            exif_dict["0th"][piexif.ImageIFD.Orientation] = video_info.orientation

        if video_info.model is not None:
            exif_dict["Exif"][piexif.ImageIFD.Model] = video_info.model

        entry = srt_parser.get_entry(elapsed_time_utc) if srt_parser is not None else None

        if entry is not None:
            segs = entry["shutter"].split("/")
            exif_dict["Exif"][piexif.ExifIFD.ExposureTime] = (int(float(segs[0])), int(float(segs[1])))
            exif_dict["Exif"][piexif.ExifIFD.FocalLength] = (entry["focal_len"], 1)
            exif_dict["Exif"][piexif.ExifIFD.FNumber] = (entry["fnum"], 1)
            exif_dict["Exif"][piexif.ExifIFD.ISOSpeedRatings] = (entry["iso"], 1)

            exif_dict["GPS"] = {
                piexif.GPSIFD.GPSMapDatum: "WGS-84",
                piexif.GPSIFD.GPSDateStamp: elapsed_time.strftime('%Y:%m:%d %H:%M:%S')
            }

            if (entry["latitude"] is not None):
                exif_dict["GPS"][piexif.GPSIFD.GPSLatitude] = FloatToRational(entry["latitude"])

            if (entry["longitude"] is not None):
                exif_dict["GPS"][piexif.GPSIFD.GPSLongitude] = FloatToRational(entry["longitude"])

            if (entry["altitude"] is not None):
                exif_dict["GPS"][piexif.GPSIFD.GPSAltitude] = FloatToRational(entry["altitude"])


        exif_bytes = piexif.dump(exif_dict)
        img.save(path, exif=exif_bytes)


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


    def GetVideoInfo(self, input_file):

        video = cv2.VideoCapture(input_file)

        total_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
        frame_rate = video.get(cv2.CAP_PROP_FPS)
        start_time_utc, utc_offset, orientation, model = self.GetVideoMetadata(input_file)

        video.release()

        return collections.namedtuple("VideoInfo", ["total_frames", "frame_rate", "start_time_utc", "utc_offset", "orientation", "model"])(total_frames, frame_rate, start_time_utc, utc_offset, orientation, model)


    def GetVideoMetadata(self, input_file):

        try:

            metadata = pymediainfo.MediaInfo.parse(input_file).to_data()

            start_time_utc = None
            orientation = 1
            performer = None
            utc_offset = None

            if metadata is not None and 'tracks' in metadata:
                # Check if it is safe to access the first element of the tracks list
                if len(metadata['tracks']) > 0:

                    start_time_utc = metadata['tracks'][0].get('encoded_date') or \
                                metadata['tracks'][0].get('tagged_date')

                    start_time_utc = datetime.datetime.strptime(start_time_utc, '%Z %Y-%m-%d %H:%M:%S') if start_time_utc is not None else None

                    file_creation_date_utc = metadata['tracks'][0].get('file_creation_date')
                    file_creation_date_utc = datetime.datetime.strptime(file_creation_date_utc, '%Z %Y-%m-%d %H:%M:%S.%f') if file_creation_date_utc is not None else None

                    file_creation_date = metadata['tracks'][0].get('file_creation_date__local')
                    file_creation_date = datetime.datetime.strptime(file_creation_date, '%Y-%m-%d %H:%M:%S.%f') if file_creation_date is not None else None

                    if file_creation_date_utc is not None and file_creation_date is not None:
                        utc_offset = file_creation_date - file_creation_date_utc

                    performer = metadata['tracks'][0].get('performer')

                # Check if it is safe to access the second element of the tracks list
                if len(metadata['tracks']) > 1:

                    orientation = metadata['tracks'][1].get('rotation')

                    if orientation is not None:
                        orientation = int(float(orientation))

                        # The 8 EXIF orientation values are numbered 1 to 8.
                        # 1 = 0°: the correct orientation, no adjustment is required.
                        # 2 = 0°, mirrored: image has been flipped back-to-front.
                        # 3 = 180°: image is upside down.
                        # 4 = 180°, mirrored: image has been flipped back-to-front and is upside down.
                        # 5 = 90°: image has been flipped back-to-front and is on its side.
                        # 6 = 90°, mirrored: image is on its side.
                        # 7 = 270°: image has been flipped back-to-front and is on its far side.
                        # 8 = 270°, mirrored: image is on its far side.

                        if orientation == 0:
                            orientation = 1
                        elif orientation == 90:
                            orientation = 8
                        elif orientation == 180:
                            orientation = 3
                        elif orientation == 270:
                            orientation = 6

            return start_time_utc, utc_offset, orientation, performer

        except Exception as e:

            return start_time_utc, utc_offset, orientation, performer


def FloatToRational(f):
    f = Fraction(f).limit_denominator()
    return (f.numerator, f.denominator)