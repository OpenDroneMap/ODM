import os
import cv2
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
            # try/catch tag value due to weird bug in pyexiv2 
            # ValueError: invalid literal for int() with base 10: ''
            try:
                val = metadata[key].value
                # parse tag names
                if key == 'Exif.Image.Make':  self.camera_make = val
                elif key == 'Exif.Image.Model': self.camera_model = val
                elif key == 'Exif.Photo.FocalLength': self.focal_length = float(val)
            except Exception, e:
                pass
 
        # needed to do that since sometimes metadata contains wrong data
        img = cv2.imread(_path_file)
        self.width = img.shape[1]
        self.height = img.shape[0]

        # force focal and ccd_width with user parameter
        if args['force_focal']: self.focal_length = args['force_focal']
        if args['force_ccd']: self.ccd_width = args['force_ccd']

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
class ODMTree(object):
    def __init__(self, root_path):
        ### root path to the project
        self.root_path = io.absolute_path_file(root_path)

        ### modules paths

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

        ### important files paths
        
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
        self.odm_textured_model_txt_geo = io.join_paths(
            self.odm_texturing, 'odm_textured_model_geo.txt')
        self.odm_textured_model_ply_geo = io.join_paths(
            self.odm_texturing, 'odm_textured_model_geo.ply')
        self.odm_textured_model_obj_geo = io.join_paths(
            self.odm_texturing, 'odm_textured_model_geo.obj')
        self.odm_textured_model_mtl_geo = io.join_paths(
            self.odm_texturing, 'odm_textured_model_geo.mtl')
        self.odm_texuring_log = io.join_paths(
            self.odm_texturing, 'odm_texturing_log.txt')

        # odm_georeferencing
        self.odm_georeferencing_coords = io.join_paths(
            self.odm_georeferencing, 'coords.txt')
        self.odm_georeferencing_gcp = io.join_paths(
            self.odm_georeferencing, 'gcp_list.txt')
        self.odm_georeferencing_utm_log = io.join_paths(
            self.odm_georeferencing, 'odm_georeferencing_utm_log.txt')
        self.odm_georeferencing_log = io.join_paths(
            self.odm_georeferencing, 'odm_georeferencing_log.txt')

        # odm_orthophoto
        self.odm_orthophoto_file = io.join_paths(self.odm_orthophoto, 'odm_orthophoto.png')
        self.odm_orthophoto_corners = io.join_paths(self.odm_orthophoto, 'odm_orthphoto_corners.txt')
        self.odm_orthophoto_log = io.join_paths(self.odm_orthophoto, 'odm_orthophoto_log.txt')





        