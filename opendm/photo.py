import io
import logging
import re

import exifread
import numpy as np
from six import string_types
from datetime import datetime, timedelta
import pytz

import log
import system
import xmltodict as x2d
from opendm import get_image_size


class ODM_Photo:
    """   ODMPhoto - a class for ODMPhotos
    """

    def __init__(self, path_file):
        # Standard tags (virtually all photos have these)
        self.filename = io.extract_file_from_path_file(path_file)
        self.width = None
        self.height = None
        self.camera_make = ''
        self.camera_model = ''

        # Geo tags
        self.latitude = None
        self.longitude = None
        self.altitude = None

        # Multi-band fields
        self.band_name = 'RGB'
        self.band_index = 0

        # Multi-spectral fields
        self.radiometric_calibration = None
        self.black_level = None

        # Capture info
        self.exposure_time = None
        self.iso_speed = None
        self.bits_per_sample = None
        self.vignetting_center = None
        self.vignetting_polynomial = None
        self.irradiance = None
        self.sun_sensor = None
        self.utc_time = None

        # self.center_wavelength = None
        # self.bandwidth = None

        # parse values from metadata
        self.parse_exif_values(path_file)

        # print log message
        log.ODM_DEBUG('Loaded {}'.format(self))


    def __str__(self):
        return '{} | camera: {} {} | dimensions: {} x {} | lat: {} | lon: {} | alt: {} | band: {} ({})'.format(
                            self.filename, self.camera_make, self.camera_model, self.width, self.height, 
                            self.latitude, self.longitude, self.altitude, self.band_name, self.band_index)

    def parse_exif_values(self, _path_file):
        # Disable exifread log
        logging.getLogger('exifread').setLevel(logging.CRITICAL)

        with open(_path_file, 'rb') as f:
            tags = exifread.process_file(f, details=False)
            try:
                if 'Image Make' in tags:
                    self.camera_make = tags['Image Make'].values.encode('utf8')
                if 'Image Model' in tags:
                    self.camera_model = tags['Image Model'].values.encode('utf8')
                if 'GPS GPSAltitude' in tags:
                    self.altitude = self.float_value(tags['GPS GPSAltitude'])
                    if 'GPS GPSAltitudeRef' in tags and self.int_value(tags['GPS GPSAltitudeRef']) > 0:
                        self.altitude *= -1
                if 'GPS GPSLatitude' in tags and 'GPS GPSLatitudeRef' in tags:
                    self.latitude = self.dms_to_decimal(tags['GPS GPSLatitude'], tags['GPS GPSLatitudeRef'])
                if 'GPS GPSLongitude' in tags and 'GPS GPSLongitudeRef' in tags:
                    self.longitude = self.dms_to_decimal(tags['GPS GPSLongitude'], tags['GPS GPSLongitudeRef'])
                
                if 'Image Tag 0xC61A' in tags:
                    self.black_level = self.list_values(tags['Image Tag 0xC61A'])
                elif 'BlackLevel' in tags:
                    self.black_level = self.list_values(tags['BlackLevel'])
                
                if 'EXIF ExposureTime' in tags:
                    self.exposure_time = self.float_value(tags['EXIF ExposureTime'])
                if 'EXIF ISOSpeed' in tags:
                    self.iso_speed = self.int_value(tags['EXIF ISOSpeed'])
                if 'Image BitsPerSample' in tags:
                    self.bits_per_sample = self.int_value(tags['Image BitsPerSample'])
                if 'EXIF DateTimeOriginal' in tags:
                    str_time = tags['EXIF DateTimeOriginal'].values.encode('utf8')
                    utc_time = datetime.strptime(str_time, "%Y:%m:%d %H:%M:%S")
                    subsec = 0
                    if 'EXIF SubSecTime' in tags:
                        subsec = self.int_value(tags['EXIF SubSecTime'])
                    negative = 1.0
                    if subsec < 0:
                        negative = -1.0
                        subsec *= -1.0
                    subsec = float('0.{}'.format(int(subsec)))
                    subsec *= negative
                    ms = subsec * 1e3
                    utc_time += timedelta(milliseconds = ms)
                    timezone = pytz.timezone('UTC')
                    epoch = timezone.localize(datetime.utcfromtimestamp(0))
                    self.utc_time = (timezone.localize(utc_time) - epoch).total_seconds() * 1000.0

            except IndexError as e:
                log.ODM_WARNING("Cannot read EXIF tags for %s: %s" % (_path_file, e.message))

            # Extract XMP tags
            f.seek(0)
            xmp = self.get_xmp(f)

            for tags in xmp:
                band_name = self.get_xmp_tag(tags, 'Camera:BandName')
                if band_name is not None:
                    self.band_name = band_name.replace(" ", "")

                self.set_attr_from_xmp_tag('band_index', tags, [
                    'DLS:SensorId', # Micasense RedEdge
                    '@Camera:RigCameraIndex', # Parrot Sequoia, Sentera 21244-00_3.2MP-GS-0001
                    'Camera:RigCameraIndex', # MicaSense Altum
                ])
                self.set_attr_from_xmp_tag('radiometric_calibration', tags, [
                    'MicaSense:RadiometricCalibration',
                ])

                self.set_attr_from_xmp_tag('vignetting_center', tags, [
                    'Camera:VignettingCenter',
                    'Sentera:VignettingCenter',
                ])

                self.set_attr_from_xmp_tag('vignetting_polynomial', tags, [
                    'Camera:VignettingPolynomial',
                    'Sentera:VignettingPolynomial',
                ])
                
                self.set_attr_from_xmp_tag('irradiance', tags, [
                    'Camera:Irradiance'
                ], float)

                self.set_attr_from_xmp_tag('sun_sensor', tags, [
                    'Camera:SunSensor'
                ], float)

                # self.set_attr_from_xmp_tag('center_wavelength', tags, [
                #     'Camera:CentralWavelength'
                # ], float)

                # self.set_attr_from_xmp_tag('bandwidth', tags, [
                #     'Camera:WavelengthFWHM'
                # ], float)
            
            # print(self.band_name)
            # print(self.band_index)
            # print(self.radiometric_calibration)
            # print(self.black_level)
            # print(self.exposure_time)
            # print(self.iso_speed)
            # print(self.bits_per_sample)
            # print(self.vignetting_center)
            # print(self.sun_sensor)
            # exit(1)
        self.width, self.height = get_image_size.get_image_size(_path_file)
        
        # Sanitize band name since we use it in folder paths
        self.band_name = re.sub('[^A-Za-z0-9]+', '', self.band_name)

    def set_attr_from_xmp_tag(self, attr, xmp_tags, tags, cast=None):
        v = self.get_xmp_tag(xmp_tags, tags)
        if v is not None:
            if cast is None:
                setattr(self, attr, v)
            else:
                setattr(self, attr, cast(v))
    
    def get_xmp_tag(self, xmp_tags, tags):
        if isinstance(tags, str):
            tags = [tags]
        
        for tag in tags:
            if tag in xmp_tags:
                t = xmp_tags[tag]

                if isinstance(t, string_types):
                    return str(t)
                elif isinstance(t, dict):
                    items = t.get('rdf:Seq', {}).get('rdf:li', {})
                    if items:
                        if isinstance(items, string_types):
                            return items
                        return " ".join(items)
                elif isinstance(t, int) or isinstance(t, float):
                    return t

    
    # From https://github.com/mapillary/OpenSfM/blob/master/opensfm/exif.py
    def get_xmp(self, file):
        img_str = str(file.read())
        xmp_start = img_str.find('<x:xmpmeta')
        xmp_end = img_str.find('</x:xmpmeta')

        if xmp_start < xmp_end:
            xmp_str = img_str[xmp_start:xmp_end + 12]
            xdict = x2d.parse(xmp_str)
            xdict = xdict.get('x:xmpmeta', {})
            xdict = xdict.get('rdf:RDF', {})
            xdict = xdict.get('rdf:Description', {})
            if isinstance(xdict, list):
                return xdict
            else:
                return [xdict]
        else:
            return []

    def dms_to_decimal(self, dms, sign):
        """Converts dms coords to decimal degrees"""
        degrees, minutes, seconds = self.float_values(dms)

        return (-1 if sign.values[0] in 'SWsw' else 1) * (
            degrees +
            minutes / 60 +
            seconds / 3600
        )

    def float_values(self, tag):
        return map(lambda v: float(v.num) / float(v.den), tag.values) 
    
    def float_value(self, tag):
        v = self.float_values(tag)
        if len(v) > 0:
            return v[0]

    def int_values(self, tag):
        return map(int, tag.values)

    def int_value(self, tag):
        v = self.int_values(tag)
        if len(v) > 0:
            return v[0]

    def list_values(self, tag):
        return " ".join(map(str, tag.values))

    def get_radiometric_calibration(self):
        if self.radiometric_calibration:
            parts = self.radiometric_calibration.split(" ")
            if len(parts) == 3:
                return list(map(float, parts))

        return [None, None, None]                
    
    def get_dark_level(self):
        if self.black_level:
            levels = np.array([float(v) for v in self.black_level.split(" ")])
            return levels.mean()
        return None

    def get_gain(self):
        if self.iso_speed:
            return self.iso_speed / 100.0
        return None

    def get_vignetting_center(self):
        if self.vignetting_center:
            parts = self.vignetting_center.split(" ")
            if len(parts) == 2:
                return list(map(float, parts))
        return [None, None]

    def get_vignetting_polynomial(self):
        if self.vignetting_polynomial:
            parts = self.vignetting_polynomial.split(" ")
            if len(parts) > 0:
                return list(map(float, parts))
        
        return None