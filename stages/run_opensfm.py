import sys
import os

from opendm import log
from opendm import io
from opendm import system
from opendm import context
from opendm import gsd
from opendm import point_cloud
from opendm import types
from opendm.osfm import OSFMContext

class ODMOpenSfMStage(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']
        photos = reconstruction.photos

        if not photos:
            log.ODM_ERROR('Not enough photos in photos array to start OpenSfM')
            exit(1)

        octx = OSFMContext(tree.opensfm)
        octx.setup(args, tree.dataset_raw, photos, gcp_path=tree.odm_georeferencing_gcp, rerun=self.rerun())
        octx.extract_metadata(self.rerun())
        octx.feature_matching(self.rerun())
        octx.reconstruct(self.rerun())

        if args.fast_orthophoto:
            output_file = octx.path('reconstruction.ply')
        elif args.use_opensfm_dense:
            output_file = tree.opensfm_model
        else:
            output_file = tree.opensfm_reconstruction

        # Always export VisualSFM's reconstruction and undistort images
        # as we'll use these for texturing (after GSD estimation and resizing)
        if not args.ignore_gsd:
            image_scale = gsd.image_scale_factor(args.orthophoto_resolution, tree.opensfm_reconstruction)
        else:
            image_scale = 1.0

        if not io.file_exists(tree.opensfm_reconstruction_nvm) or self.rerun():
            octx.run('export_visualsfm --image_extension png --scale_focal %s' % image_scale)
        else:
            log.ODM_WARNING('Found a valid OpenSfM NVM reconstruction file in: %s' %
                            tree.opensfm_reconstruction_nvm)

        # These will be used for texturing
        undistorted_images_path = octx.path("undistorted")

        if not io.dir_exists(undistorted_images_path) or self.rerun():
            octx.run('undistort --image_format png --image_scale %s' % image_scale)
        else:
            log.ODM_WARNING("Found an undistorted directory in %s" % undistorted_images_path)

        # Skip dense reconstruction if necessary and export
        # sparse reconstruction instead
        if args.fast_orthophoto:
            if not io.file_exists(output_file) or self.rerun():
                octx.run('export_ply --no-cameras' % image_scale)
            else:
                log.ODM_WARNING("Found a valid PLY reconstruction in %s" % output_file)

        elif args.use_opensfm_dense:
            # Undistort images at full scale in JPG
            # (TODO: we could compare the size of the PNGs if they are < than depthmap_resolution
            # and use those instead of re-exporting full resolution JPGs)
            if not io.file_exists(output_file) or self.rerun():
                octx.run('undistort')
                octx.run('compute_depthmaps')
            else:
                log.ODM_WARNING("Found a valid dense reconstruction in %s" % output_file)

        # check if reconstruction was exported to bundler before
        octx.export_bundler(tree.opensfm_bundle_list, self.rerun())

        if reconstruction.georef and (not io.file_exists(tree.opensfm_transformation) or self.rerun()):
            octx.run('export_geocoords --transformation --proj \'%s\'' % reconstruction.georef.projection.srs)
        else:
            log.ODM_WARNING("Will skip exporting %s" % tree.opensfm_transformation)