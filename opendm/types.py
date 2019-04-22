import cv2
import exifread
import re
from fractions import Fraction
from opensfm.exif import sensor_string
from opendm import get_image_size
from pyproj import Proj

import log
import io
import system
import context
import logging

class ODM_Photo:
    """   ODMPhoto - a class for ODMPhotos
    """

    def __init__(self, path_file):
        #  general purpose
        self.filename = io.extract_file_from_path_file(path_file)
        self.width = None
        self.height = None
        # other attributes
        self.camera_make = ''
        self.camera_model = ''
        self.make_model = ''
        self.latitude = None
        self.longitude = None
        self.altitude = None
        # parse values from metadata
        self.parse_exif_values(path_file)

        # print log message
        log.ODM_DEBUG('Loaded {}'.format(self))


    def __str__(self):
        return '{} | camera: {} | dimensions: {} x {} | lat: {} | lon: {} | alt: {}'.format(
                            self.filename, self.make_model, self.width, self.height, self.latitude, self.longitude, self.altitude)

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
                    self.altitude = self.float_values(tags['GPS GPSAltitude'])[0]
                    if 'GPS GPSAltitudeRef' in tags and self.int_values(tags['GPS GPSAltitudeRef'])[0] > 0:
                        self.altitude *= -1
                if 'GPS GPSLatitude' in tags and 'GPS GPSLatitudeRef' in tags:
                    self.latitude = self.dms_to_decimal(tags['GPS GPSLatitude'], tags['GPS GPSLatitudeRef'])
                if 'GPS GPSLongitude' in tags and 'GPS GPSLongitudeRef' in tags:
                    self.longitude = self.dms_to_decimal(tags['GPS GPSLongitude'], tags['GPS GPSLongitudeRef'])
            except IndexError as e:
                log.ODM_WARNING("Cannot read EXIF tags for %s: %s" % (_path_file, e.message))

        if self.camera_make and self.camera_model:
            self.make_model = sensor_string(self.camera_make, self.camera_model)

        # needed to do that since sometimes metadata contains wrong data
        try:
            self.width, self.height = get_image_size.get_image_size(_path_file)
        except get_image_size.UnknownImageFormat:
            # Fallback to slower cv2
            img = cv2.imread(_path_file)
            self.width = img.shape[1]
            self.height = img.shape[0]

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

    def int_values(self, tag):
        return map(int, tag.values)


class ODM_Reconstruction(object):
    """docstring for ODMReconstruction"""

    def __init__(self, photos, projstring = None, coords_file = None):
        self.photos = photos    # list of ODM_Photos
        self.projection = None  # Projection system the whole project will be in
        self.georef = None
        if projstring:
            self.projection = self.set_projection(projstring)
            self.georef = ODM_GeoRef(self.projection)
        else:
            self.projection = self.parse_coordinate_system(coords_file)
            if self.projection:
                self.georef = ODM_GeoRef(self.projection)

    def parse_coordinate_system(self, _file):
        """Write attributes to jobOptions from coord file"""
        # check for coordinate file existence
        if not io.file_exists(_file):
            log.ODM_WARNING('Could not find file %s' % _file)
            return

        with open(_file) as f:
            # extract reference system and utm zone from first line.
            # We will assume the following format:
            # 'WGS84 UTM 17N' or 'WGS84 UTM 17N \n'
            line = f.readline().rstrip()
            log.ODM_DEBUG('Line: %s' % line)
            ref = line.split(' ')
            # match_wgs_utm = re.search('WGS84 UTM (\d{1,2})(N|S)', line, re.I)
            try:
                if ref[0] == 'WGS84' and ref[1] == 'UTM':  # match_wgs_utm:
                    datum = ref[0]
                    utm_pole = (ref[2][len(ref[2]) - 1]).upper()
                    utm_zone = int(ref[2][:len(ref[2]) - 1])

                    proj_args = {
                        'proj': "utm", 
                        'zone': utm_zone, 
                        'datum': datum,
                        'no_defs': True
                    }
                    if utm_pole == 'S':
                        proj_args['south'] = True

                    return Proj(**proj_args)
                elif '+proj' in line:
                    return Proj(line.strip('\''))
                elif 'epsg' in line.lower():
                    return Proj(init=line)
                else:
                    log.ODM_ERROR('Could not parse coordinates. Bad CRS supplied: %s' % line)
            except RuntimeError as e:
                log.ODM_ERROR('Uh oh! There seems to be a problem with your GCP file.\n\n'
                                    'The line: %s\n\n'
                                    'Is not valid. Projections that are valid include:\n'
                                    ' - EPSG:*****\n'
                                    ' - WGS84 UTM **(N|S)\n'
                                    ' - Any valid proj4 string (for example, +proj=utm +zone=32 +north +ellps=WGS84 +datum=WGS84 +units=m +no_defs)\n\n'
                                    'Modify your GCP file and try again.' % line)
                raise RuntimeError(e)


    def set_projection(self, projstring):
        try:
            return Proj(projstring)
        except RuntimeError:
            log.ODM_EXCEPTION('Could not set projection. Please use a proj4 string')


class ODM_GeoRef(object):
    """docstring for ODMUtmZone"""

    def __init__(self, projection):
        self.projection = projection
        self.epsg = None
        self.utm_east_offset = 0
        self.utm_north_offset = 0
        self.transform = []
        self.gcps = []

    def calculate_EPSG(self, _utm_zone, _pole):
        """Calculate and return the EPSG"""
        if _pole == 'S':
            return 32700 + _utm_zone
        elif _pole == 'N':
            return 32600 + _utm_zone
        else:
            log.ODM_ERROR('Unknown pole format %s' % _pole)
            return

    def calculate_EPSG(self, proj):
        return proj

    def coord_to_fractions(self, coord, refs):
        deg_dec = abs(float(coord))
        deg = int(deg_dec)
        minute_dec = (deg_dec - deg) * 60
        minute = int(minute_dec)

        sec_dec = (minute_dec - minute) * 60
        sec_dec = round(sec_dec, 3)
        sec_denominator = 1000
        sec_numerator = int(sec_dec * sec_denominator)
        if float(coord) >= 0:
            latRef = refs[0]
        else:
            latRef = refs[1]

        output = str(deg) + '/1 ' + str(minute) + '/1 ' + str(sec_numerator) + '/' + str(sec_denominator)
        return output, latRef

    def extract_offsets(self, _file):
        if not io.file_exists(_file):
            log.ODM_ERROR('Could not find file %s' % _file)
            return

        with open(_file) as f:
            offsets = f.readlines()[1].split(' ')
            self.utm_east_offset = float(offsets[0])
            self.utm_north_offset = float(offsets[1])

    def parse_transformation_matrix(self, _file):
        if not io.file_exists(_file):
            log.ODM_ERROR('Could not find file %s' % _file)
            return

        # Create a nested list for the transformation matrix
        with open(_file) as f:
            for line in f:
                # Handle matrix formats that either
                # have leading or trailing brakets or just plain numbers.
                line = re.sub(r"[\[\],]", "", line).strip()
                self.transform += [[float(i) for i in line.split()]]

        self.utm_east_offset = self.transform[0][3]
        self.utm_north_offset = self.transform[1][3]


class ODM_Tree(object):
    def __init__(self, root_path, images_path, gcp_file = None):
        # root path to the project
        self.root_path = io.absolute_path_file(root_path)
        if not images_path:
            self.input_images = io.join_paths(self.root_path, 'images')
        else:
            self.input_images = io.absolute_path_file(images_path)

        # modules paths

        # here are defined where all modules should be located in
        # order to keep track all files al directories during the
        # whole reconstruction process.
        self.dataset_raw = io.join_paths(self.root_path, 'images')
        self.opensfm = io.join_paths(self.root_path, 'opensfm')
        self.mve = io.join_paths(self.root_path, 'mve')
        self.odm_meshing = io.join_paths(self.root_path, 'odm_meshing')
        self.odm_texturing = io.join_paths(self.root_path, 'odm_texturing')
        self.odm_25dtexturing = io.join_paths(self.root_path, 'odm_texturing_25d')
        self.odm_georeferencing = io.join_paths(self.root_path, 'odm_georeferencing')
        self.odm_25dgeoreferencing = io.join_paths(self.root_path, 'odm_25dgeoreferencing')
        self.odm_filterpoints = io.join_paths(self.root_path, 'odm_filterpoints')
        self.odm_orthophoto = io.join_paths(self.root_path, 'odm_orthophoto')

        # important files paths

        # benchmarking
        self.benchmarking = io.join_paths(self.root_path, 'benchmark.txt')
        self.dataset_list = io.join_paths(self.root_path, 'img_list.txt')

        # opensfm
        self.opensfm_tracks = io.join_paths(self.opensfm, 'tracks.csv')
        self.opensfm_bundle = io.join_paths(self.opensfm, 'bundle_r000.out')
        self.opensfm_bundle_list = io.join_paths(self.opensfm, 'list_r000.out')
        self.opensfm_image_list = io.join_paths(self.opensfm, 'image_list.txt')
        self.opensfm_reconstruction = io.join_paths(self.opensfm, 'reconstruction.json')
        self.opensfm_reconstruction_nvm = io.join_paths(self.opensfm, 'reconstruction.nvm')
        self.opensfm_model = io.join_paths(self.opensfm, 'depthmaps/merged.ply')
        self.opensfm_transformation = io.join_paths(self.opensfm, 'geocoords_transformation.txt')

        # mve
        self.mve_model = io.join_paths(self.mve, 'mve_dense_point_cloud.ply')
        self.mve_path = io.join_paths(self.opensfm, 'mve')
        self.mve_image_list = io.join_paths(self.mve_path, 'list.txt')
        self.mve_bundle = io.join_paths(self.mve_path, 'bundle/bundle.out')
        self.mve_views = io.join_paths(self.mve, 'views')

        # filter points
        self.filtered_point_cloud = io.join_paths(self.odm_filterpoints, "point_cloud.ply")

        # odm_meshing
        self.odm_mesh = io.join_paths(self.odm_meshing, 'odm_mesh.ply')
        self.odm_meshing_log = io.join_paths(self.odm_meshing, 'odm_meshing_log.txt')
        self.odm_25dmesh = io.join_paths(self.odm_meshing, 'odm_25dmesh.ply')
        self.odm_25dmeshing_log = io.join_paths(self.odm_meshing, 'odm_25dmeshing_log.txt')

        # texturing
        self.odm_texturing_undistorted_image_path = io.join_paths(
            self.odm_texturing, 'undistorted')
        self.odm_textured_model_obj = 'odm_textured_model.obj'
        self.odm_textured_model_mtl = 'odm_textured_model.mtl'
        # Log is only used by old odm_texturing
        self.odm_texuring_log = 'odm_texturing_log.txt'

        # odm_georeferencing
        self.odm_georeferencing_latlon = io.join_paths(
            self.odm_georeferencing, 'latlon.txt')
        self.odm_georeferencing_coords = io.join_paths(
            self.odm_georeferencing, 'coords.txt')
        self.odm_georeferencing_gcp = gcp_file or io.find('gcp_list.txt', self.root_path)
        self.odm_georeferencing_utm_log = io.join_paths(
            self.odm_georeferencing, 'odm_georeferencing_utm_log.txt')
        self.odm_georeferencing_log = 'odm_georeferencing_log.txt'
        self.odm_georeferencing_transform_file = 'odm_georeferencing_transform.txt'
        self.odm_georeferencing_proj = 'proj.txt'
        self.odm_georeferencing_model_txt_geo = 'odm_georeferencing_model_geo.txt'
        self.odm_georeferencing_model_obj_geo = 'odm_textured_model_geo.obj'
        self.odm_georeferencing_xyz_file = io.join_paths(
            self.odm_georeferencing, 'odm_georeferenced_model.csv')
        self.odm_georeferencing_las_json = io.join_paths(
            self.odm_georeferencing, 'las.json')
        self.odm_georeferencing_model_laz = io.join_paths(
            self.odm_georeferencing, 'odm_georeferenced_model.laz')
        self.odm_georeferencing_model_las = io.join_paths(
            self.odm_georeferencing, 'odm_georeferenced_model.las')
        self.odm_georeferencing_dem = io.join_paths(
            self.odm_georeferencing, 'odm_georeferencing_model_dem.tif')

        # odm_orthophoto
        self.odm_orthophoto_file = io.join_paths(self.odm_orthophoto, 'odm_orthophoto.png')
        self.odm_orthophoto_tif = io.join_paths(self.odm_orthophoto, 'odm_orthophoto.tif')
        self.odm_orthophoto_corners = io.join_paths(self.odm_orthophoto, 'odm_orthophoto_corners.txt')
        self.odm_orthophoto_log = io.join_paths(self.odm_orthophoto, 'odm_orthophoto_log.txt')
        self.odm_orthophoto_tif_log = io.join_paths(self.odm_orthophoto, 'gdal_translate_log.txt')
        self.odm_orthophoto_gdaladdo_log = io.join_paths(self.odm_orthophoto, 'gdaladdo_log.txt')

    def path(self, *args):
        return io.join_paths(self.root_path, *args)
