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
        # useful attibutes
        self.filename = None
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
        self.compute_focal_length()

    def compute_focal_length(self):
        # set log message
        msg = 'Loaded %s | dimensions: %s x %s' % \
            (self.filename, self.width, self.height)

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
            # update log message
            msg += ' | focal: %smm | ccd: %smm' % (self.focal_length, self.ccd_width)
        else:
            # update log message
            if self.focal_length:
                msg += ' | focal: %smm' % self.focal_length
            
            if self.ccd_width:
                msg += ' | ccd: %smm' % self.ccd_width
        # print log message
        log.ODM_DEBUG(msg)

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

        # extract and set filename from path file
        self.filename = io.extract_file_from_path_file(_path_file)

        # find ccd_width from file if needed
        if self.ccd_width is None and self.camera_model is not None:
            # load ccd_widths from file
            ccd_widths = system.get_ccd_widths()
            # search ccd by camera model
            key = [x for x in ccd_widths.keys() if self.camera_model in x]
            # convert to float if found
            if key: self.ccd_width = float(ccd_widths[key][0])
            # else:
            # log.ODM_ERROR('Could not find ccd_width in file')


# TODO: finish this class
class ODMReconstruction(object):
    """docstring for ODMReconstruction"""
    def __init__(self, arg):
        super(ODMReconstruction, self).__init__()
        self.arg = arg
        