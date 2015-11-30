import os
import pyexiv2
import subprocess

import log
import io
import system

class ODMPhoto:
    """   ODMPhoto - a class for ODMPhotos
    """
    def __init__(self, path_file, args):
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
        # parse values from metadata
        self.parse_pyexiv2_values(self.path_file, args)
        # compute focal lenght into pixels
        self.update_focal()

        # print log message
        log.ODM_DEBUG('Loaded %s | dimensions: %s x %s | focal: %s | ccd: %s' % \
            (self.filename, self.width, self.height, self.focal_length, self.ccd_width))

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

    def parse_pyexiv2_values(self, _path_file, args):
        # read image metadata
        metadata = pyexiv2.ImageMetadata(_path_file)
        metadata.read()
        # loop over image tags
        for key in metadata:
            # catch tag value
            val = metadata[key].value
            # parse tag names
            if key == 'Exif.Image.Make':  self.camera_make = val
            elif key == 'Exif.Image.Model': self.camera_model = val
            elif key == 'Exif.Photo.PixelXDimension': self.width = val
            elif key == 'Exif.Photo.PixelYDimension': self.height = val
            elif key == 'Exif.Photo.FocalLength': self.focal_length = float(val)

        # TODO(edgar): set self.focal_length if args['force_frocal']
        #              set self.ccd_width    if args['force_ccd']

        # find ccd_width from file if needed
        if self.ccd_width is None and self.camera_model is not None:
            # load ccd_widths from file
            ccd_widths = system.get_ccd_widths()
            # search ccd by camera model
            key = [x for x in ccd_widths.keys() if self.camera_model in x]
            # convert to float if found
            if key: self.ccd_width = float(ccd_widths[key[0]])
            # else:
            # log.ODM_ERROR('Could not find ccd_width in file')


# TODO: finish this class
class ODMReconstruction(object):
    """docstring for ODMReconstruction"""
    def __init__(self, arg):
        super(ODMReconstruction, self).__init__()
        self.arg = arg
        