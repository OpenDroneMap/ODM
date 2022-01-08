import logging
import re
import os
import math

import exifread
import numpy as np
from six import string_types
from datetime import datetime, timedelta, timezone
import pytz

from opendm import io
from opendm import log
from opendm import system
import xmltodict as x2d
from opendm import get_image_size
from xml.parsers.expat import ExpatError
from opensfm.sensors import sensor_data
from opensfm.geo import ecef_from_lla

projections = ['perspective', 'fisheye', 'brown', 'dual', 'equirectangular', 'spherical']

def find_largest_photo_dim(photos):
    max_dim = 0
    for p in photos:
        if p.width is None:
            continue
        max_dim = max(max_dim, max(p.width, p.height))
        
    return max_dim

def find_largest_photo(photos):
    max_p = None
    max_area = 0
    for p in photos:
        if p.width is None:
            continue
        area = p.width * p.height

        if area > max_area:
            max_area = area
            max_p = p

    return max_p

def get_mm_per_unit(resolution_unit):
    """Length of a resolution unit in millimeters.

    Uses the values from the EXIF specs in
    https://www.sno.phy.queensu.ca/~phil/exiftool/TagNames/EXIF.html

    Args:
        resolution_unit: the resolution unit value given in the EXIF
    """
    if resolution_unit == 2:  # inch
        return 25.4
    elif resolution_unit == 3:  # cm
        return 10
    elif resolution_unit == 4:  # mm
        return 1
    elif resolution_unit == 5:  # um
        return 0.001
    else:
        log.ODM_WARNING("Unknown EXIF resolution unit value: {}".format(resolution_unit))
        return None

class PhotoCorruptedException(Exception):
    pass

class ODM_Photo:
    """ODMPhoto - a class for ODMPhotos"""

    def __init__(self, path_file):
        self.filename = os.path.basename(path_file)
        self.mask = None
        
        # Standard tags (virtually all photos have these)
        self.width = None
        self.height = None
        self.camera_make = ''
        self.camera_model = ''
        self.orientation = 1

        # Geo tags
        self.latitude = None
        self.longitude = None
        self.altitude = None

        # Multi-band fields
        self.band_name = 'RGB'
        self.band_index = 0
        self.capture_uuid = None # DJI only

        # Multi-spectral fields
        self.fnumber = None
        self.radiometric_calibration = None
        self.black_level = None

        # Capture info
        self.exposure_time = None
        self.iso_speed = None
        self.bits_per_sample = None
        self.vignetting_center = None
        self.vignetting_polynomial = None
        self.spectral_irradiance = None
        self.horizontal_irradiance = None
        self.irradiance_scale_to_si = None
        self.utc_time = None

        # OPK angles
        self.yaw = None
        self.pitch = None
        self.roll = None
        self.omega = None
        self.phi = None
        self.kappa = None

        # DLS
        self.sun_sensor = None
        self.dls_yaw = None
        self.dls_pitch = None
        self.dls_roll = None

        # self.center_wavelength = None
        # self.bandwidth = None

        # RTK
        self.gps_xy_stddev = None # Dilution of Precision X/Y
        self.gps_z_stddev = None # Dilution of Precision Z

        # Misc SFM
        self.camera_projection = 'brown'
        self.focal_ratio = 0.85

        # parse values from metadata
        self.parse_exif_values(path_file)

        # print log message
        log.ODM_DEBUG('Loaded {}'.format(self))


    def __str__(self):
        return '{} | camera: {} {} | dimensions: {} x {} | lat: {} | lon: {} | alt: {} | band: {} ({})'.format(
                            self.filename, self.camera_make, self.camera_model, self.width, self.height, 
                            self.latitude, self.longitude, self.altitude, self.band_name, self.band_index)

    def set_mask(self, mask):
        self.mask = mask

    def update_with_geo_entry(self, geo_entry):
        self.latitude = geo_entry.y
        self.longitude = geo_entry.x
        self.altitude = geo_entry.z
        self.omega = geo_entry.omega
        self.phi = geo_entry.phi
        self.kappa = geo_entry.kappa
        self.dls_yaw = geo_entry.omega
        self.dls_pitch = geo_entry.phi
        self.dls_roll = geo_entry.kappa
        self.gps_xy_stddev = geo_entry.horizontal_accuracy
        self.gps_z_stddev = geo_entry.vertical_accuracy

    def parse_exif_values(self, _path_file):
        # Disable exifread log
        logging.getLogger('exifread').setLevel(logging.CRITICAL)

        try:
            self.width, self.height = get_image_size.get_image_size(_path_file)
        except Exception as e:
            raise PhotoCorruptedException(str(e))

        with open(_path_file, 'rb') as f:
            tags = exifread.process_file(f, details=False)
            try:
                if 'Image Make' in tags:
                    try:
                        self.camera_make = tags['Image Make'].values
                    except UnicodeDecodeError:
                        log.ODM_WARNING("EXIF Image Make might be corrupted")
                        self.camera_make = "unknown"
                if 'Image Model' in tags:
                    try:
                        self.camera_model = tags['Image Model'].values
                    except UnicodeDecodeError:
                        log.ODM_WARNING("EXIF Image Model might be corrupted")
                        self.camera_model = "unknown"
                if 'GPS GPSAltitude' in tags:
                    self.altitude = self.float_value(tags['GPS GPSAltitude'])
                    if 'GPS GPSAltitudeRef' in tags and self.int_value(tags['GPS GPSAltitudeRef']) > 0:
                        self.altitude *= -1
                if 'GPS GPSLatitude' in tags and 'GPS GPSLatitudeRef' in tags:
                    self.latitude = self.dms_to_decimal(tags['GPS GPSLatitude'], tags['GPS GPSLatitudeRef'])
                if 'GPS GPSLongitude' in tags and 'GPS GPSLongitudeRef' in tags:
                    self.longitude = self.dms_to_decimal(tags['GPS GPSLongitude'], tags['GPS GPSLongitudeRef'])
                if 'Image Orientation' in tags:
                    self.orientation = self.int_value(tags['Image Orientation'])
            except (IndexError, ValueError) as e:
                log.ODM_WARNING("Cannot read basic EXIF tags for %s: %s" % (_path_file, str(e)))

            try:
                self.focal_ratio = self.extract_focal(self.camera_make, self.camera_model, tags)
            except (IndexError, ValueError) as e:
                log.ODM_WARNING("Cannot extract focal ratio for %s: %s" % (_path_file, str(e)))

            try:
                if 'Image Tag 0xC61A' in tags:
                    self.black_level = self.list_values(tags['Image Tag 0xC61A'])
                elif 'BlackLevel' in tags:
                    self.black_level = self.list_values(tags['BlackLevel'])
                
                if 'EXIF ExposureTime' in tags:
                    self.exposure_time = self.float_value(tags['EXIF ExposureTime'])

                if 'EXIF FNumber' in tags:
                    self.fnumber = self.float_value(tags['EXIF FNumber'])
                
                if 'EXIF ISOSpeed' in tags:
                    self.iso_speed = self.int_value(tags['EXIF ISOSpeed'])
                elif 'EXIF PhotographicSensitivity' in tags:
                    self.iso_speed = self.int_value(tags['EXIF PhotographicSensitivity'])
                elif 'EXIF ISOSpeedRatings' in tags:
                    self.iso_speed = self.int_value(tags['EXIF ISOSpeedRatings'])
                    

                if 'Image BitsPerSample' in tags:
                    self.bits_per_sample = self.int_value(tags['Image BitsPerSample'])
                if 'EXIF DateTimeOriginal' in tags:
                    str_time = tags['EXIF DateTimeOriginal'].values
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
            except Exception as e:
                log.ODM_WARNING("Cannot read extended EXIF tags for %s: %s" % (_path_file, str(e)))


            # Extract XMP tags
            f.seek(0)
            xmp = self.get_xmp(f)

            for tags in xmp:
                try:
                    band_name = self.get_xmp_tag(tags, ['Camera:BandName', '@Camera:BandName'])
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
                    
                    self.set_attr_from_xmp_tag('horizontal_irradiance', tags, [
                        'Camera:HorizontalIrradiance'
                    ], float)

                    self.set_attr_from_xmp_tag('irradiance_scale_to_si', tags, [
                        'Camera:IrradianceScaleToSIUnits'
                    ], float)

                    self.set_attr_from_xmp_tag('sun_sensor', tags, [
                        'Camera:SunSensor',
                    ], float)

                    self.set_attr_from_xmp_tag('spectral_irradiance', tags, [
                        'Camera:SpectralIrradiance',
                        'Camera:Irradiance',
                    ], float)

                    self.set_attr_from_xmp_tag('capture_uuid', tags, [
                        '@drone-dji:CaptureUUID', # DJI
                        '@Camera:ImageUniqueID', # sentera 6x
                    ])

                    # DJI GPS tags
                    self.set_attr_from_xmp_tag('longitude', tags, [
                        '@drone-dji:Longitude'
                    ], float)
                    self.set_attr_from_xmp_tag('latitude', tags, [
                        '@drone-dji:Latitude'
                    ], float)
                    self.set_attr_from_xmp_tag('altitude', tags, [
                        '@drone-dji:AbsoluteAltitude'
                    ], float)

                    # Phantom 4 RTK
                    if '@drone-dji:RtkStdLon' in tags:
                        y = float(self.get_xmp_tag(tags, '@drone-dji:RtkStdLon'))
                        x = float(self.get_xmp_tag(tags, '@drone-dji:RtkStdLat'))
                        self.gps_xy_stddev = max(x, y)
                    
                        if '@drone-dji:RtkStdHgt' in tags:
                            self.gps_z_stddev = float(self.get_xmp_tag(tags, '@drone-dji:RtkStdHgt'))
                    else:
                        self.set_attr_from_xmp_tag('gps_xy_stddev', tags, [
                            '@Camera:GPSXYAccuracy',
                            'GPSXYAccuracy'
                        ], float)
                        self.set_attr_from_xmp_tag('gps_z_stddev', tags, [
                            '@Camera:GPSZAccuracy',
                            'GPSZAccuracy'
                        ], float)

                    if 'DLS:Yaw' in tags:
                        self.set_attr_from_xmp_tag('dls_yaw', tags, ['DLS:Yaw'], float)
                        self.set_attr_from_xmp_tag('dls_pitch', tags, ['DLS:Pitch'], float)
                        self.set_attr_from_xmp_tag('dls_roll', tags, ['DLS:Roll'], float)
                
                    camera_projection = self.get_xmp_tag(tags, ['@Camera:ModelType', 'Camera:ModelType'])
                    if camera_projection is not None:
                        camera_projection = camera_projection.lower()
                        if camera_projection in projections:
                            self.camera_projection = camera_projection

                    # OPK
                    self.set_attr_from_xmp_tag('yaw', tags, ['@drone-dji:GimbalYawDegree', '@Camera:Yaw', 'Camera:Yaw'], float)
                    self.set_attr_from_xmp_tag('pitch', tags, ['@drone-dji:GimbalPitchDegree', '@Camera:Pitch', 'Camera:Pitch'], float)
                    self.set_attr_from_xmp_tag('roll', tags, ['@drone-dji:GimbalRollDegree', '@Camera:Roll', 'Camera:Roll'], float)

                    # Normalize YPR conventions (assuming nadir camera)
                    # Yaw: 0 --> top of image points north
                    # Yaw: 90 --> top of image points east
                    # Yaw: 270 --> top of image points west
                    # Pitch: 0 --> nadir camera
                    # Pitch: 90 --> camera is looking forward
                    # Roll: 0 (assuming gimbal)
                    if self.has_ypr():
                        if self.camera_make.lower() == 'dji':
                            self.pitch = 90 + self.pitch

                except Exception as e:
                    log.ODM_WARNING("Cannot read XMP tags for %s: %s" % (_path_file, str(e)))

                # self.set_attr_from_xmp_tag('center_wavelength', tags, [
                #     'Camera:CentralWavelength'
                # ], float)

                # self.set_attr_from_xmp_tag('bandwidth', tags, [
                #     'Camera:WavelengthFWHM'
                # ], float)

        # Sanitize band name since we use it in folder paths
        self.band_name = re.sub('[^A-Za-z0-9]+', '', self.band_name)

        self.compute_opk()

    def extract_focal(self, make, model, tags):
        if make != "unknown":
            # remove duplicate 'make' information in 'model'
            model = model.replace(make, "")
        
        sensor_string = (make.strip() + " " + model.strip()).strip().lower()

        sensor_width = None
        if ("EXIF FocalPlaneResolutionUnit" in tags and "EXIF FocalPlaneXResolution" in tags):
            resolution_unit = self.float_value(tags["EXIF FocalPlaneResolutionUnit"])
            mm_per_unit = get_mm_per_unit(resolution_unit)
            if mm_per_unit:
                pixels_per_unit = self.float_value(tags["EXIF FocalPlaneXResolution"])
                if pixels_per_unit <= 0 and "EXIF FocalPlaneYResolution" in tags:
                    pixels_per_unit = self.float_value(tags["EXIF FocalPlaneYResolution"])
                
                if pixels_per_unit > 0 and self.width is not None:
                    units_per_pixel = 1 / pixels_per_unit
                    sensor_width = self.width * units_per_pixel * mm_per_unit

        focal_35 = None
        focal = None
        if "EXIF FocalLengthIn35mmFilm" in tags:
            focal_35 = self.float_value(tags["EXIF FocalLengthIn35mmFilm"])
        if "EXIF FocalLength" in tags:
            focal = self.float_value(tags["EXIF FocalLength"])

        if focal_35 is not None and focal_35 > 0:
            focal_ratio = focal_35 / 36.0  # 35mm film produces 36x24mm pictures.
        else:
            if not sensor_width:
                sensor_width = sensor_data().get(sensor_string, None)
            if sensor_width and focal:
                focal_ratio = focal / sensor_width
            else:
                focal_ratio = 0.85

        return focal_ratio

    def set_attr_from_xmp_tag(self, attr, xmp_tags, tags, cast=None):
        v = self.get_xmp_tag(xmp_tags, tags)
        if v is not None:
            if cast is None:
                setattr(self, attr, v)
            else:
                # Handle fractions
                if (cast == float or cast == int) and "/" in v:
                    v = self.try_parse_fraction(v)
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
        img_bytes = file.read()
        xmp_start = img_bytes.find(b'<x:xmpmeta')
        xmp_end = img_bytes.find(b'</x:xmpmeta')

        if xmp_start < xmp_end:
            xmp_str = img_bytes[xmp_start:xmp_end + 12].decode('utf8')
            try:
                xdict = x2d.parse(xmp_str)
            except ExpatError as e:
                from bs4 import BeautifulSoup
                xmp_str = str(BeautifulSoup(xmp_str, 'xml'))
                xdict = x2d.parse(xmp_str)
                log.ODM_WARNING("%s has malformed XMP XML (but we fixed it)" % self.filename)
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
        
        if degrees is not None and minutes is not None and seconds is not None:
            return (-1 if sign.values[0] in 'SWsw' else 1) * (
                degrees +
                minutes / 60 +
                seconds / 3600
            )

    def float_values(self, tag):
        if isinstance(tag.values, list):
            result = []
            for v in tag.values:
                if isinstance(v, int):
                    result.append(float(v))
                elif v.den != 0:
                    result.append(float(v.num) / float(v.den))
                else:
                    result.append(None)
            return result
        else:
            return [float(tag.values.num) / float(tag.values.den) if tag.values.den != 0 else None]
    
    def float_value(self, tag):
        v = self.float_values(tag)
        if len(v) > 0:
            return v[0]

    def int_values(self, tag):
        if isinstance(tag.values, list):
            return [int(v) for v in tag.values]
        else:
            return [int(tag.values)]

    def int_value(self, tag):
        v = self.int_values(tag)
        if len(v) > 0:
            return v[0]

    def list_values(self, tag):
        return " ".join(map(str, tag.values))

    def try_parse_fraction(self, val):
        parts = val.split("/")
        if len(parts) == 2:
            try:
                num, den = map(float, parts)
                return num / den if den != 0 else val
            except ValueError:
                pass
        return val 

    def get_radiometric_calibration(self):
        if isinstance(self.radiometric_calibration, str):
            parts = self.radiometric_calibration.split(" ")
            if len(parts) == 3:
                return list(map(float, parts))

        return [None, None, None]                
    
    def get_dark_level(self):
        if self.black_level:
            levels = np.array([float(v) for v in self.black_level.split(" ")])
            return levels.mean()

    def get_gain(self):
        #(gain = ISO/100)
        if self.iso_speed:
            return self.iso_speed / 100.0

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
                coeffs = list(map(float, parts))

                # Different camera vendors seem to use different ordering for the coefficients
                if self.camera_make != "Sentera":
                    coeffs.reverse()
                return coeffs

    def get_utc_time(self):
        if self.utc_time:
            return datetime.fromtimestamp(self.utc_time / 1000, timezone.utc)

    def get_photometric_exposure(self):
        # H ~= (exposure_time) / (f_number^2)
        if self.fnumber is not None and self.exposure_time is not None and self.exposure_time > 0 and self.fnumber > 0:
            return self.exposure_time / (self.fnumber * self.fnumber)

    def get_horizontal_irradiance(self):
        if self.horizontal_irradiance is not None:
            scale = 1.0 # Assumed
            if self.irradiance_scale_to_si is not None:
                scale = self.irradiance_scale_to_si
            
            return self.horizontal_irradiance * scale
    
    def get_sun_sensor(self):
        if self.sun_sensor is not None:
            # TODO: Presence of XMP:SunSensorExposureTime
            # and XMP:SunSensorSensitivity might
            # require additional logic. If these two tags are present, 
            # then sun_sensor is not in physical units?
            return self.sun_sensor / 65535.0 # normalize uint16 (is this correct?)
        elif self.spectral_irradiance is not None:
            scale = 1.0 # Assumed
            if self.irradiance_scale_to_si is not None:
                scale = self.irradiance_scale_to_si
            
            return self.spectral_irradiance * scale

    def get_dls_pose(self):
        if self.dls_yaw is not None:
            return [self.dls_yaw, self.dls_pitch, self.dls_roll]
        return [0.0, 0.0, 0.0]

    def get_bit_depth_max(self):
        if self.bits_per_sample:
            return float(2 ** self.bits_per_sample)

        return None

    def get_capture_id(self):
        # Use capture UUID first, capture time as fallback
        if self.capture_uuid is not None:
            return self.capture_uuid

        return self.get_utc_time()

    def get_gps_dop(self):
        val = -9999
        if self.gps_xy_stddev is not None:
            val = self.gps_xy_stddev
        if self.gps_z_stddev is not None:
            val = max(val, self.gps_z_stddev)
        if val > 0:
            return val

        return None

    def override_gps_dop(self, dop):
        self.gps_xy_stddev = self.gps_z_stddev = dop

    def override_camera_projection(self, camera_projection):
        if camera_projection in projections:
            self.camera_projection = camera_projection

    def is_thermal(self):
        #Added for support M2EA camera sensor
        if(self.camera_make == "DJI"):
            return self.camera_model == "MAVIC2-ENTERPRISE-ADVANCED" and self.width == 640 and self.height == 512
        return self.band_name.upper() in ["LWIR"] # TODO: more?

    def camera_id(self):
        return " ".join(
                [
                    "v2",
                    self.camera_make.strip(),
                    self.camera_model.strip(),
                    str(int(self.width)),
                    str(int(self.height)),
                    self.camera_projection,
                    str(float(self.focal_ratio))[:6],
                ]
            ).lower()

    def to_opensfm_exif(self):
        capture_time = 0.0
        if self.utc_time is not None:
            capture_time = self.utc_time / 1000.0
        
        gps = {}
        if self.latitude is not None and self.longitude is not None:
            gps['latitude'] = self.latitude
            gps['longitude'] = self.longitude
            if self.altitude is not None:
                gps['altitude'] = self.altitude
            else:
                gps['altitude'] = 0.0

            dop = self.get_gps_dop()
            if dop is None:
                dop = 10.0 # Default
            
            gps['dop'] = dop

        d = {
            "make": self.camera_make,
            "model": self.camera_model,
            "width": self.width,
            "height": self.height,
            "projection_type": self.camera_projection,
            "focal_ratio": self.focal_ratio,
            "orientation": self.orientation,
            "capture_time": capture_time,
            "gps": gps,
            "camera": self.camera_id()
        }

        if self.has_opk():
            d['opk'] = {
                'omega': self.omega,
                'phi': self.phi,
                'kappa': self.kappa
            }
        
        return d

    def has_ypr(self):
        return self.yaw is not None and \
            self.pitch is not None and \
            self.roll is not None
    
    def has_opk(self):
        return self.omega is not None and \
            self.phi is not None and \
            self.kappa is not None

    def has_geo(self):
        return self.latitude is not None and \
            self.longitude is not None
    
    def compute_opk(self):
        if self.has_ypr() and self.has_geo():
            y, p, r = math.radians(self.yaw), math.radians(self.pitch), math.radians(self.roll)

            # Ref: New Calibration and Computing Method for Direct 
            # Georeferencing of Image and Scanner Data Using the 
            # Position and Angular Data of an Hybrid Inertial Navigation System 
            # by Manfred Bäumker

            # YPR rotation matrix
            cnb = np.array([[ math.cos(y) * math.cos(p), math.cos(y) * math.sin(p) * math.sin(r) - math.sin(y) * math.cos(r), math.cos(y) * math.sin(p) * math.cos(r) + math.sin(y) * math.sin(r)],
                            [ math.sin(y) * math.cos(p), math.sin(y) * math.sin(p) * math.sin(r) + math.cos(y) * math.cos(r), math.sin(y) * math.sin(p) * math.cos(r) - math.cos(y) * math.sin(r)],
                            [ -math.sin(p), math.cos(p) * math.sin(r), math.cos(p) * math.cos(r)],
                           ])

            # Convert between image and body coordinates
            # Top of image pixels point to flying direction
            # and camera is looking down.
            # We might need to change this if we want different
            # camera mount orientations (e.g. backward or sideways)

            # (Swap X/Y, flip Z)
            cbb = np.array([[0, 1, 0],
                            [1, 0, 0],
                            [0, 0, -1]])
            
            delta = 1e-7
            
            alt = self.altitude if self.altitude is not None else 0.0
            p1 = np.array(ecef_from_lla(self.latitude + delta, self.longitude, alt))
            p2 = np.array(ecef_from_lla(self.latitude - delta, self.longitude, alt))
            xnp = p1 - p2
            m = np.linalg.norm(xnp)
            
            if m == 0:
                log.ODM_WARNING("Cannot compute OPK angles, divider = 0")
                return
            
            # Unit vector pointing north
            xnp /= m

            znp = np.array([0, 0, -1]).T
            ynp = np.cross(znp, xnp)

            cen = np.array([xnp, ynp, znp]).T

            # OPK rotation matrix
            ceb = cen.dot(cnb).dot(cbb)

            self.omega = math.degrees(math.atan2(-ceb[1][2], ceb[2][2]))
            self.phi = math.degrees(math.asin(ceb[0][2]))
            self.kappa = math.degrees(math.atan2(-ceb[0][1], ceb[0][0]))
