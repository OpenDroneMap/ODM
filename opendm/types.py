import cv2
import pyexiv2
import re
from fractions import Fraction
from opensfm.exif import sensor_string

import log
import io
import system
import context


class ODM_Photo:
    """   ODMPhoto - a class for ODMPhotos
    """

    def __init__(self, path_file, force_focal, force_ccd):
        #  general purpose
        self.path_file = path_file
        self.filename = io.extract_file_from_path_file(path_file)
        # useful attibutes
        self.width = None
        self.height = None
        self.ccd_width = None
        self.focal_length = None
        self.focal_length_px = None
        # other attributes
        self.camera_make = ''
        self.camera_model = ''
        self.make_model = ''
        self.latitude = None
        self.longitude = None
        self.altitude = None
        # parse values from metadata
        self.parse_pyexiv2_values(self.path_file, force_focal, force_ccd)
        # compute focal length into pixels
        self.update_focal()

        # print log message
        log.ODM_DEBUG('Loaded {} | camera: {} | dimensions: {} x {} | focal: {} | ccd: {} | lat: {} | lon: {} | alt: {}'
                      .format(self.filename, self.make_model, self.width, self.height, self.focal_length,
                              self.ccd_width, self.latitude, self.longitude, self.altitude))

    def update_focal(self):
        # compute focal length in pixels
        if self.focal_length and self.ccd_width:
            # take width or height as reference
            if self.width > self.height:
                # f(px) = w(px) * f(mm) / ccd(mm)
                self.focal_length_px = \
                    self.width * (self.focal_length / self.ccd_width)
            else:
                # f(px) = h(px) * f(mm) / ccd(mm)
                self.focal_length_px = \
                    self.height * (self.focal_length / self.ccd_width)

    def parse_pyexiv2_values(self, _path_file, _force_focal, _force_ccd):
        # read image metadata
        metadata = pyexiv2.ImageMetadata(_path_file)
        metadata.read()
        # loop over image tags
        for key in metadata:
            # try/catch tag value due to weird bug in pyexiv2 
            # ValueError: invalid literal for int() with base 10: ''
            GPS = 'Exif.GPSInfo.GPS'
            try:
                # parse tag names
                if key == 'Exif.Image.Make':
                    self.camera_make = metadata[key].value
                elif key == 'Exif.Image.Model':
                    self.camera_model = metadata[key].value
                elif key == 'Exif.Photo.FocalLength':
                    self.focal_length = float(metadata[key].value)
                elif key == GPS + 'Latitude':
                    self.latitude = self.dms_to_decimal(*metadata[key].value +
                                                        [metadata[GPS + 'LatitudeRef'].value])
                elif key == GPS + 'Longitude':
                    self.longitude = self.dms_to_decimal(*metadata[key].value +
                                                         [metadata[GPS + 'LongitudeRef'].value])
                elif key == GPS + 'Altitude':
                    self.altitude = float(metadata[key].value)
                    if metadata[GPS + 'AltitudeRef'] and int(metadata[GPS + 'AltitudeRef'].value) > 0:
                        self.altitude *= -1.
            except (pyexiv2.ExifValueError, ValueError) as e:
                pass
            except KeyError as e:
                log.ODM_DEBUG('Tag not set')
            except NotImplementedError as e:
                pass

        if self.camera_make and self.camera_model:
            self.make_model = sensor_string(self.camera_make, self.camera_model)

        # needed to do that since sometimes metadata contains wrong data
        img = cv2.imread(_path_file)
        self.width = img.shape[1]
        self.height = img.shape[0]

        # force focal and ccd_width with user parameter
        if _force_focal:
            self.focal_length = _force_focal
        if _force_ccd:
            self.ccd_width = _force_ccd

        # find ccd_width from file if needed
        if self.ccd_width is None and self.camera_model is not None:
            # load ccd_widths from file
            ccd_widths = system.get_ccd_widths()
            # search ccd by camera model
            key = [x for x in ccd_widths.keys() if self.make_model in x]
            # convert to float if found
            if key:
                self.ccd_width = float(ccd_widths[key[0]])
            else:
                log.ODM_WARNING('Could not find ccd_width in file. Use --force-ccd or edit the sensor_data.json '
                                'file to manually input ccd width')

    def dms_to_decimal(self, degrees, minutes, seconds, sign=' '):
        """Converts dms coords to decimal degrees"""
        return (-1 if sign[0] in 'SWsw' else 1) * (
            float(degrees) +
            float(minutes) / 60 +
            float(seconds) / 3600
        )


# TODO: finish this class
class ODM_Reconstruction(object):
    """docstring for ODMReconstruction"""

    def __init__(self, arg):
        super(ODMReconstruction, self).__init__()
        self.arg = arg


class ODM_GCPoint(object):
    """docstring for ODMPoint"""

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class ODM_GeoRef(object):
    """docstring for ODMUtmZone"""

    def __init__(self):
        self.datum = 'WGS84'
        self.epsg = None
        self.utm_zone = 0
        self.utm_pole = 'N'
        self.utm_east_offset = 0
        self.utm_north_offset = 0
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

    def convert_to_las(self, _file, _file_out, json_file):

        if not self.epsg:
            log.ODM_ERROR('Empty EPSG: Could not convert to LAS')
            return

        kwargs = {'bin': context.pdal_path,
                  'f_in': _file,
                  'f_out': _file_out,
                  'east': self.utm_east_offset,
                  'north': self.utm_north_offset,
                  'epsg': self.epsg,
                  'json': json_file}

        # create pipeline file transform.xml to enable transformation
        pipeline = '{{' \
                   '  "pipeline":[' \
                   '    "untransformed.ply",' \
                   '    {{' \
                   '      "type":"filters.transformation",' \
                   '      "matrix":"1 0 0 {east} 0 1 0 {north} 0 0 1 0 0 0 0 1"' \
                   '    }},' \
                   '    {{' \
                   '      "a_srs":"EPSG:{epsg}",' \
                   '      "offset_x":"{east}",' \
                   '      "offset_y":"{north}",' \
                   '      "offset_z":"0",' \
                   '      "filename":"transformed.las"' \
                   '    }}' \
                   '  ]' \
                   '}}'.format(**kwargs)

        with open(json_file, 'w') as f:
            f.write(pipeline)

        # call pdal 
        system.run('{bin}/pdal pipeline -i {json} --readers.ply.filename={f_in} '
                   '--writers.las.filename={f_out}'.format(**kwargs))

    def utm_to_latlon(self, _file, _photo, idx):

        gcp = self.gcps[idx]

        kwargs = {'epsg': self.epsg,
                  'file': _file,
                  'x': gcp.x + self.utm_east_offset,
                  'y': gcp.y + self.utm_north_offset,
                  'z': gcp.z}

        latlon = system.run_and_return('echo {x} {y} {z} '.format(**kwargs),
                                       'gdaltransform -s_srs \"EPSG:{epsg}\" '
                                       '-t_srs \"EPSG:4326\"'.format(**kwargs)).split()

        # Example: 83d18'16.285"W
        # Example: 41d2'11.789"N
        # Example: 0.998

        if len(latlon) == 3:
            lon_str, lat_str, alt_str = latlon
        elif len(latlon) == 2:
            lon_str, lat_str = latlon
            alt_str = ''
        else:
            log.ODM_ERROR('Something went wrong %s' % latlon)

        lat_frac = self.coord_to_fractions(latlon[1], ['N', 'S'])
        lon_frac = self.coord_to_fractions(latlon[0], ['E', 'W'])

        # read image metadata
        metadata = pyexiv2.ImageMetadata(_photo.path_file)
        metadata.read()

        # #set values
        #
        # # GPS latitude
        # key = 'Exif.GPSInfo.GPSLatitude'
        # value = lat_frac[0].split(' ')
        # log.ODM_DEBUG('lat_frac: %s %s %s' % (value[0], value[1], value[2]))
        # metadata[key] = pyexiv2.ExifTag(key,
        #                                 [Fraction(value[0]),
        #                                  Fraction(value[1]),
        #                                  Fraction(value[2])])
        #
        # key = 'Exif.GPSInfo.GPSLatitudeRef'
        # value = lat_frac[1]
        # metadata[key] = pyexiv2.ExifTag(key, value)
        #
        # # GPS longitude
        # key = 'Exif.GPSInfo.GPSLongitude'
        # value = lon_frac[0].split(' ')
        # metadata[key] = pyexiv2.ExifTag(key,
        #                                 [Fraction(value[0]),
        #                                  Fraction(value[1]),
        #                                  Fraction(value[2])])
        #
        # key = 'Exif.GPSInfo.GPSLongitudeRef'
        # value = lon_frac[1]
        # metadata[key] = pyexiv2.ExifTag(key, value)
        #
        # # GPS altitude
        # altitude = abs(int(float(latlon[2]) * 100))
        # key = 'Exif.GPSInfo.GPSAltitude'
        # value = Fraction(altitude, 1)
        # metadata[key] = pyexiv2.ExifTag(key, value)
        #
        # if latlon[2] >= 0:
        #     altref = '0'
        # else:
        #     altref = '1'
        # key = 'Exif.GPSInfo.GPSAltitudeRef'
        # metadata[key] = pyexiv2.ExifTag(key, altref)
        #
        # # write values
        # metadata.write()

    def parse_coordinate_system(self, _file):
        """Write attributes to jobOptions from coord file"""
        # check for coordinate file existence
        if not io.file_exists(_file):
            log.ODM_ERROR('Could not find file %s' % _file)
            return

        with open(_file) as f:
            # extract reference system and utm zone from first line.
            # We will assume the following format:
            # 'WGS84 UTM 17N' or 'WGS84 UTM 17N \n'
            line = f.readline().rstrip()
            log.ODM_DEBUG('Line: %s' % line)
            ref = line.split(' ')
            # match_wgs_utm = re.search('WGS84 UTM (\d{1,2})(N|S)', line, re.I)
            if ref[0] == 'WGS84' and ref[1] == 'UTM':  # match_wgs_utm:
                self.datum = ref[0]
                self.utm_pole = ref[2][len(ref) - 1]
                self.utm_zone = int(ref[2][:len(ref[2]) - 1])
                # extract east and west offsets from second line.
                # We will assume the following format:
                # '440143 4588391'
                # update EPSG
                self.epsg = self.calculate_EPSG(self.utm_zone, self.utm_pole)
            # If the first line looks like "EPSG:n" or "epsg:n"
            elif ref[0].split(':')[0].lower() == 'epsg':
                self.epsg = line.split(':')[1]
            else:
                log.ODM_ERROR('Could not parse coordinates. Bad CRS supplied: %s' % line)
                return

            offsets = f.readline().split(' ')
            self.utm_east_offset = int(offsets[0])
            self.utm_north_offset = int(offsets[1])

            # parse coordinates
            lines = f.readlines()
            for l in lines:
                xyz = l.split(' ')
                if len(xyz) == 3:
                    x, y, z = xyz[:3]
                elif len(xyz) == 2:
                    x, y = xyz[:2]
                    z = 0
                self.gcps.append(ODM_GCPoint(float(x), float(y), float(z)))
                # Write to json file


class ODM_Tree(object):
    def __init__(self, root_path, images_path):
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
        self.pmvs = io.join_paths(self.root_path, 'pmvs')
        self.odm_meshing = io.join_paths(self.root_path, 'odm_meshing')
        self.odm_texturing = io.join_paths(self.root_path, 'odm_texturing')
        self.odm_25dtexturing = io.join_paths(self.root_path, 'odm_texturing_25d')
        self.odm_georeferencing = io.join_paths(self.root_path, 'odm_georeferencing')
        self.odm_25dgeoreferencing = io.join_paths(self.root_path, 'odm_25dgeoreferencing')
        self.odm_orthophoto = io.join_paths(self.root_path, 'odm_orthophoto')
        self.odm_pdal = io.join_paths(self.root_path, 'pdal')

        # important files paths

        # benchmarking
        self.benchmarking = io.join_paths(self.root_path, 'benchmark.txt')

        # opensfm
        self.opensfm_tracks = io.join_paths(self.opensfm, 'tracks.csv')
        self.opensfm_bundle = io.join_paths(self.opensfm, 'bundle_r000.out')
        self.opensfm_bundle_list = io.join_paths(self.opensfm, 'list_r000.out')
        self.opensfm_image_list = io.join_paths(self.opensfm, 'image_list.txt')
        self.opensfm_reconstruction = io.join_paths(self.opensfm, 'reconstruction.json')
        self.opensfm_reconstruction_meshed = io.join_paths(self.opensfm, 'reconstruction.meshed.json')
        self.opensfm_reconstruction_nvm = io.join_paths(self.opensfm, 'reconstruction.nvm')
        self.opensfm_model = io.join_paths(self.opensfm, 'depthmaps/merged.ply')

        # pmvs
        self.pmvs_rec_path = io.join_paths(self.pmvs, 'recon0')
        self.pmvs_bundle = io.join_paths(self.pmvs_rec_path, 'bundle.rd.out')
        self.pmvs_visdat = io.join_paths(self.pmvs_rec_path, 'vis.dat')
        self.pmvs_options = io.join_paths(self.pmvs_rec_path, 'pmvs_options.txt')
        self.pmvs_model = io.join_paths(self.pmvs_rec_path, 'models/option-0000.ply')

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
        self.odm_georeferencing_gcp = io.join_paths(
            self.odm_georeferencing, 'gcp_list.txt')
        self.odm_georeferencing_utm_log = io.join_paths(
            self.odm_georeferencing, 'odm_georeferencing_utm_log.txt')
        self.odm_georeferencing_log = 'odm_georeferencing_log.txt'
        self.odm_georeferencing_transform_file = 'odm_georeferencing_transform.txt'
        self.odm_georeferencing_model_txt_geo = 'odm_georeferencing_model_geo.txt'
        self.odm_georeferencing_model_ply_geo = 'odm_georeferenced_model.ply'
        self.odm_georeferencing_model_obj_geo = 'odm_textured_model_geo.obj'
        self.odm_georeferencing_xyz_file = io.join_paths(
            self.odm_georeferencing, 'odm_georeferenced_model.csv')
        self.odm_georeferencing_las_json = io.join_paths(
            self.odm_georeferencing, 'las.json')
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
