import sys
import os

from opendm import log
from opendm import io
from opendm import system
from opendm import context
from opendm import gsd
from opendm import point_cloud
from opendm import types
from opendm import osfm

class ODMOpenSfMStage(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']
        photos = reconstruction.photos

        if not photos:
            log.ODM_ERROR('Not enough photos in photos array to start OpenSfM')
            exit(1)

        if args.fast_orthophoto:
            output_file = io.join_paths(tree.opensfm, 'reconstruction.ply')
        elif args.use_opensfm_dense:
            output_file = tree.opensfm_model
        else:
            output_file = tree.opensfm_reconstruction

        # check if reconstruction was done before
        # TODO: more granularity for each step (setup/featurematch/reconstruction/etc.)
        
        if not io.file_exists(output_file) or self.rerun():

            osfm.setup(args, tree.dataset_raw, tree.opensfm, photos, gcp_path=tree.odm_georeferencing_gcp)

            osfm.feature_matching(tree.opensfm, self.rerun())

            osfm.reconstruction(tree.opensfm, self.rerun())

            # Always export VisualSFM's reconstruction and undistort images
            # as we'll use these for texturing (after GSD estimation and resizing)
            if not args.ignore_gsd:
                image_scale = gsd.image_scale_factor(args.orthophoto_resolution, tree.opensfm_reconstruction)
            else:
                image_scale = 1.0

            if not io.file_exists(tree.opensfm_reconstruction_nvm) or self.rerun():
                osfm.run('export_visualsfm --image_extension png --scale_focal %s' % image_scale, tree.opensfm)
            else:
                log.ODM_WARNING('Found a valid OpenSfM NVM reconstruction file in: %s' %
                                tree.opensfm_reconstruction_nvm)

            # These will be used for texturing
            osfm.run('undistort --image_format png --image_scale %s' % image_scale, tree.opensfm)

            # Skip dense reconstruction if necessary and export
            # sparse reconstruction instead
            if args.fast_orthophoto:
                osfm.run('export_ply --no-cameras' % image_scale, tree.opensfm)
            elif args.use_opensfm_dense:
                # Undistort images at full scale in JPG
                # (TODO: we could compare the size of the PNGs if they are < than depthmap_resolution
                # and use those instead of re-exporting full resolution JPGs)
                osfm.run('undistort', tree.opensfm)
                osfm.run('compute_depthmaps', tree.opensfm)
        else:
            log.ODM_WARNING('Found a valid OpenSfM reconstruction file in: %s' %
                            tree.opensfm_reconstruction)

        # check if reconstruction was exported to bundler before
        osfm.export_bundler(tree.opensfm, tree.opensfm_bundle_list, self.rerun())

        if reconstruction.georef:
            osfm.run('export_geocoords --transformation --proj \'%s\'' % reconstruction.georef.projection.srs, tree.opensfm)
