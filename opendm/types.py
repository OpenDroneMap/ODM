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
        self.camera_make = None
        self.camera_model = None
        self.make_model = None
        # parse values from metadata
        self.parse_pyexiv2_values(self.path_file, force_focal, force_ccd)
        # compute focal length into pixels
        self.update_focal()

        # print log message
        log.ODM_DEBUG('Loaded %s | camera: %s | dimensions: %s x %s | focal: %s | ccd: %s' %
                      (self.filename, self.make_model, self.width, self.height, self.focal_length, self.ccd_width))

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
            try:
                val = metadata[key].value
                # parse tag names
                if key == 'Exif.Image.Make':
                    self.camera_make = val
                elif key == 'Exif.Image.Model':
                    self.camera_model = val
                elif key == 'Exif.Photo.FocalLength':
                    self.focal_length = float(val)
            except pyexiv2.ExifValueError, e:
                pass

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

    def convert_to_las(self, _file, pdalXML):

        if not self.epsg:
            log.ODM_ERROR('Empty EPSG: Could not convert to LAS')
            return

        kwargs = {'bin': context.pdal_path,
                  'f_in': _file,
                  'f_out': _file + '.las',
                  'east': self.utm_east_offset,
                  'north': self.utm_north_offset,
                  'epsg': self.epsg,
                  'xml': pdalXML}

        # call txt2las
        # system.run('{bin}/txt2las -i {f_in} -o {f_out} -skip 30 -parse xyzRGBssss ' \
        #           '-set_scale 0.01 0.01 0.01 -set_offset {east} {north} 0 '  \
        #           '-translate_xyz 0 -epsg {epsg}'.format(**kwargs))
        #           
        # create pipeline file transform.xml to enable transformation
        pipelineXml = '<?xml version=\"1.0\" encoding=\"utf-8\"?>'
        pipelineXml += '<Pipeline version=\"1.0\">'
        pipelineXml += '  <Writer type=\"writers.las\">'
        pipelineXml += '    <Option name=\"filename\">'
        pipelineXml += '      transformed.las'
        pipelineXml += '    </Option>'
        pipelineXml += '    <Filter type=\"filters.transformation\">'
        pipelineXml += '      <Option name=\"matrix\">'
        pipelineXml += '        1  0  0  {east}'.format(**kwargs)
        pipelineXml += '        0  1  0  {north}'.format(**kwargs)
        pipelineXml += '        0  0  1  0'
        pipelineXml += '        0  0  0  1'
        pipelineXml += '      </Option>'
        pipelineXml += '      <Reader type=\"readers.ply\">'
        pipelineXml += '        <Option name=\"filename\">'
        pipelineXml += '          untransformed.ply'
        pipelineXml += '        </Option>'
        pipelineXml += '      </Reader>'
        pipelineXml += '    </Filter>'
        pipelineXml += '  </Writer>'
        pipelineXml += '</Pipeline>'

        with open(pdalXML, 'w') as f:
            f.write(pipelineXml)

        # call pdal 
        system.run('{bin}/pdal pipeline -i {xml} --readers.ply.filename={f_in} '
                   '--writers.las.filename={f_out}'.format(**kwargs))

    def utm_to_latlon(self, _file, _photo, idx):

        gcp = self.gcps[idx]

        kwargs = {'datum': self.datum,
                  'zone': self.utm_zone,
                  'file': _file,
                  'x': gcp.x + self.utm_east_offset,
                  'y': gcp.y + self.utm_north_offset,
                  'z': gcp.z}

        latlon = system.run_and_return('echo {x} {y} | cs2cs +proj=utm '
                                       '+datum={datum} +ellps={datum} +zone={zone} +units=m +to '
                                       '+proj=latlon +ellps={datum}'.format(**kwargs)).split()

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

        tokens = re.split("[d '\"]+", lon_str)
        if len(tokens) >= 4:
            lon_deg, lon_min, lon_sec = tokens[:3]
            lon_sec_frac = Fraction(lon_sec)
            lon_sec_numerator = str(lon_sec_frac._numerator)
            lon_sec_denominator = str(lon_sec_frac._denominator)
            lon_ref = tokens[3]

            tokens = re.split("[d '\"]+", lat_str)
            if len(tokens) >= 4:
                lat_deg, lat_min, lat_sec = tokens[:3]
                lat_sec_frac = Fraction(lat_sec)
                lat_sec_numerator = str(lat_sec_frac._numerator)
                lat_sec_denominator = str(lat_sec_frac._denominator)
                lat_ref = tokens[3]

                alt_numerator = arc_denominator = 0  # BUG: arc_denominator is never used

                if alt_str:
                    alt_frac = Fraction(alt_str)
                    alt_numerator = alt_frac._numerator
                    alt_denominator = alt_frac._denominator

        # read image metadata
        metadata = pyexiv2.ImageMetadata(_photo.path_file)
        metadata.read()

        # set values

        # GPS latitude
        key = 'Exif.GPSInfo.GPSLatitude'
        value = [Fraction(int(lat_deg), 1), Fraction(int(lat_min), 1), \
                 Fraction(int(lat_sec_numerator), int(lat_sec_denominator))]
        metadata[key] = pyexiv2.ExifTag(key, value)

        key = 'Exif.GPSInfo.GPSLatitudeRef'
        value = '%s' % lat_ref
        metadata[key] = pyexiv2.ExifTag(key, value)

        # GPS longitude
        key = 'Exif.GPSInfo.GPSLongitude'
        value = [Fraction(int(lon_deg), 1), Fraction(int(lon_min), 1), \
                 Fraction(int(lon_sec_numerator), int(lon_sec_denominator))]
        metadata[key] = pyexiv2.ExifTag(key, value)

        key = 'Exif.GPSInfo.GPSLongitudeRef'
        value = '%s' % lon_ref
        metadata[key] = pyexiv2.ExifTag(key, value)

        # GPS altitude
        key = 'Exif.GPSInfo.GPSAltitude'
        value = Fraction(int(gcp.z), 1)
        metadata[key] = pyexiv2.ExifTag(key, value)

        key = 'Exif.GPSInfo.GPSAltitudeRef'
        metadata[key] = pyexiv2.ExifTag(key, '0')

        # write values
        metadata.write()

    def parse_coordinate_system(self, _file):
        """Write attributes to jobOptions from coord file"""
        # check for coordinate file existence
        if not io.file_exists(_file):
            log.ODM_ERROR('Could not find file %s' % _file)
            return

        with open(_file) as f:
            # extract reference system and utm zone from first line.
            # We will assume the following format:
            # 'WGS84 UTM 17N'
            line = f.readline().split(' ')
            self.datum = line[0]
            self.utm_pole = line[2][len(line) - 1]
            self.utm_zone = int(line[2][:len(line) - 1])
            # extract east and west offsets from second line.
            # We will assume the following format:
            # '440143 4588391'
            line = f.readline().split(' ')
            self.utm_east_offset = int(line[0])
            self.utm_north_offset = int(line[1])
            # parse coordinates
            lines = f.readlines()
            for l in lines:
                x, y, z = l.split(' ')[:3]
                self.gcps.append(ODM_GCPoint(float(x), float(y), float(z)))

        # update EPSG
        self.epsg = self.calculate_EPSG(self.utm_zone, self.utm_pole)


class ODM_Tree(object):
    def __init__(self, root_path):
        # root path to the project
        self.root_path = io.absolute_path_file(root_path)

        # modules paths

        # here are defined where all modules should be located in
        # order to keep track all files al directories during the
        # whole reconstruction process.
        self.dataset_raw = io.join_paths(self.root_path, 'images')
        self.dataset_resize = io.join_paths(self.root_path, 'images_resize')
        self.opensfm = io.join_paths(self.root_path, 'opensfm')
        self.pmvs = io.join_paths(self.root_path, 'pmvs')
        self.odm_meshing = io.join_paths(self.root_path, 'odm_meshing')
        self.odm_texturing = io.join_paths(self.root_path, 'odm_texturing')
        self.odm_georeferencing = io.join_paths(self.root_path, 'odm_georeferencing')
        self.odm_orthophoto = io.join_paths(self.root_path, 'odm_orthophoto')
        self.odm_pdal = io.join_paths(self.root_path, 'pdal')

        # important files paths

        # benchmarking
        self.benchmarking = io.join_paths(self.root_path, 'benchmark.txt')

        # opensfm
        self.opensfm_bundle = io.join_paths(self.opensfm, 'bundle_r000.out')
        self.opensfm_bundle_list = io.join_paths(self.opensfm, 'list_r000.out')
        self.opensfm_image_list = io.join_paths(self.opensfm, 'image_list.txt')
        self.opensfm_reconstruction = io.join_paths(self.opensfm, 'reconstruction.json')

        # pmvs
        self.pmvs_rec_path = io.join_paths(self.pmvs, 'recon0')
        self.pmvs_bundle = io.join_paths(self.pmvs_rec_path, 'bundle.rd.out')
        self.pmvs_visdat = io.join_paths(self.pmvs_rec_path, 'vis.dat')
        self.pmvs_options = io.join_paths(self.pmvs_rec_path, 'pmvs_options.txt')
        self.pmvs_model = io.join_paths(self.pmvs_rec_path, 'models/option-0000.ply')

        # odm_meshing
        self.odm_mesh = io.join_paths(self.odm_meshing, 'odm_mesh.ply')
        self.odm_meshing_log = io.join_paths(self.odm_meshing, 'odm_meshing_log.txt')

        # odm_texturing
        self.odm_textured_model_obj = io.join_paths(
            self.odm_texturing, 'odm_textured_model.obj')
        self.odm_textured_model_mtl = io.join_paths(
            self.odm_texturing, 'odm_textured_model.mtl')
        self.odm_texuring_log = io.join_paths(
            self.odm_texturing, 'odm_texturing_log.txt')

        # odm_georeferencing
        self.odm_georeferencing_latlon = io.join_paths(
            self.odm_georeferencing, 'latlon.txt')
        self.odm_georeferencing_coords = io.join_paths(
            self.odm_georeferencing, 'coords.txt')
        self.odm_georeferencing_gcp = io.join_paths(
            self.odm_georeferencing, 'gcp_list.txt')
        self.odm_georeferencing_utm_log = io.join_paths(
            self.odm_georeferencing, 'odm_georeferencing_utm_log.txt')
        self.odm_georeferencing_log = io.join_paths(
            self.odm_georeferencing, 'odm_georeferencing_log.txt')
        self.odm_georeferencing_model_txt_geo = io.join_paths(
            self.odm_georeferencing, 'odm_georeferencing_model_geo.txt')
        self.odm_georeferencing_model_ply_geo = io.join_paths(
            self.odm_georeferencing, 'odm_georeferenced_model.ply')
        self.odm_georeferencing_model_obj_geo = io.join_paths(
            self.odm_texturing, 'odm_textured_model_geo.obj')  # these files will be kept in odm_texturing/
        self.odm_georeferencing_model_mtl_geo = io.join_paths(
            self.odm_texturing, 'odm_textured_model_geo.mtl')  # these files will be kept in odm_texturing/
        self.odm_georeferencing_xyz_file = io.join_paths(
            self.odm_georeferencing, 'odm_georeferenced_model.csv')
        self.odm_georeferencing_pdal = io.join_paths(
            self.odm_georeferencing, 'pipeline.xml')

        # odm_orthophoto
        self.odm_orthophoto_file = io.join_paths(self.odm_orthophoto, 'odm_orthophoto.png')
        self.odm_orthophoto_tif = io.join_paths(self.odm_orthophoto, 'odm_orthophoto.tif')
        self.odm_orthophoto_corners = io.join_paths(self.odm_orthophoto, 'odm_orthphoto_corners.txt')
        self.odm_orthophoto_log = io.join_paths(self.odm_orthophoto, 'odm_orthophoto_log.txt')
        self.odm_orthophoto_tif_log = io.join_paths(self.odm_orthophoto, 'gdal_translate_log.txt')
