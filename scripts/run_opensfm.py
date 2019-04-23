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

        # create working directories
        system.mkdir_p(tree.opensfm)

        if args.fast_orthophoto:
            output_file = io.join_paths(tree.opensfm, 'reconstruction.ply')
        elif args.use_opensfm_dense:
            output_file = tree.opensfm_model
        else:
            output_file = tree.opensfm_reconstruction

        # check if reconstruction was done before
        if not io.file_exists(output_file) or self.rerun():

            osfm.setup(args, self.params, tree.dataset_raw, tree.opensfm, photos, gcp_path=tree.odm_georeferencing_gcp)

            # run OpenSfM reconstruction
            matched_done_file = io.join_paths(tree.opensfm, 'matching_done.txt')
            if not io.file_exists(matched_done_file) or self.rerun():
                osfm.run('extract_metadata', tree.opensfm)
                osfm.run('detect_features', tree.opensfm)
                osfm.run('match_features', tree.opensfm)

                with open(matched_done_file, 'w') as fout:
                    fout.write("Matching done!\n")
            else:
                log.ODM_WARNING('Found a feature matching done progress file in: %s' %
                                matched_done_file)

            if not io.file_exists(tree.opensfm_tracks) or self.rerun():
                osfm.run('create_tracks', tree.opensfm)
            else:
                log.ODM_WARNING('Found a valid OpenSfM tracks file in: %s' %
                                tree.opensfm_tracks)

            if not io.file_exists(tree.opensfm_reconstruction) or self.rerun():
                osfm.run('reconstruct', tree.opensfm)
            else:
                log.ODM_WARNING('Found a valid OpenSfM reconstruction file in: %s' %
                                tree.opensfm_reconstruction)

            # Check that a reconstruction file has been created
            if not io.file_exists(tree.opensfm_reconstruction):
                log.ODM_ERROR("The program could not process this dataset using the current settings. "
                                "Check that the images have enough overlap, "
                                "that there are enough recognizable features "
                                "and that the images are in focus. "
                                "You could also try to increase the --min-num-features parameter."
                                "The program will now exit.")
                sys.exit(1)


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
        if not io.file_exists(tree.opensfm_bundle_list) or self.rerun():
            # convert back to bundler's format
            osfm.run('export_bundler', tree.opensfm)
        else:
            log.ODM_WARNING('Found a valid Bundler file in: %s' %
                            tree.opensfm_reconstruction)

        if reconstruction.georef:
            osfm.run('export_geocoords --transformation --proj \'%s\'' % reconstruction.georef.projection.srs, tree.opensfm)
