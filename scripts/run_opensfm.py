import ecto

from opendm import log
from opendm import io
from opendm import system
from opendm import context
from opendm import gsd

class ODMOpenSfMCell(ecto.Cell):
    def declare_params(self, params):
        params.declare("use_exif_size", "The application arguments.", False)
        params.declare("feature_process_size", "The application arguments.", 2400)
        params.declare("feature_min_frames", "The application arguments.", 4000)
        params.declare("processes", "The application arguments.", context.num_cores)
        params.declare("matching_gps_neighbors", "The application arguments.", 8)
        params.declare("matching_gps_distance", "The application arguments.", 0)
        params.declare("fixed_camera_params", "Optimize internal camera parameters", True)
        params.declare("hybrid_bundle_adjustment", "Use local + global bundle adjustment", False)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("reconstruction", "ODMReconstruction", [])
        outputs.declare("reconstruction", "list of ODMReconstructions", [])

    def process(self, inputs, outputs):

        # Benchmarking
        start_time = system.now_raw()

        log.ODM_INFO('Running ODM OpenSfM Cell')

        # get inputs
        tree = inputs.tree
        args = inputs.args
        reconstruction = inputs.reconstruction
        photos = reconstruction.photos

        if not photos:
            log.ODM_ERROR('Not enough photos in photos array to start OpenSfM')
            return ecto.QUIT

        # create working directories
        system.mkdir_p(tree.opensfm)

        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'opensfm') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'opensfm' in args.rerun_from)

        if args.fast_orthophoto:
            output_file = io.join_paths(tree.opensfm, 'reconstruction.ply')
        elif args.use_opensfm_dense:
            output_file = tree.opensfm_model
        else:
            output_file = tree.opensfm_reconstruction

        # check if reconstruction was done before
        if not io.file_exists(output_file) or rerun_cell:
            # create file list
            list_path = io.join_paths(tree.opensfm, 'image_list.txt')
            has_alt = True
            with open(list_path, 'w') as fout:
                for photo in photos:
                    if not photo.altitude:
                        has_alt = False
                    fout.write('%s\n' % io.join_paths(tree.dataset_raw, photo.filename))

            # create config file for OpenSfM
            config = [
                "use_exif_size: %s" % ('no' if not self.params.use_exif_size else 'yes'),
                "feature_process_size: %s" % self.params.feature_process_size,
                "feature_min_frames: %s" % self.params.feature_min_frames,
                "processes: %s" % self.params.processes,
                "matching_gps_neighbors: %s" % self.params.matching_gps_neighbors,
                "depthmap_method: %s" % args.opensfm_depthmap_method,
                "depthmap_resolution: %s" % args.depthmap_resolution,
                "depthmap_min_patch_sd: %s" % args.opensfm_depthmap_min_patch_sd,
                "depthmap_min_consistent_views: %s" % args.opensfm_depthmap_min_consistent_views,
                "optimize_camera_parameters: %s" % ('no' if self.params.fixed_camera_params else 'yes')
            ]

            if has_alt:
                log.ODM_DEBUG("Altitude data detected, enabling it for GPS alignment")
                config.append("use_altitude_tag: True")
                config.append("align_method: naive")

            if args.use_hybrid_bundle_adjustment:
                log.ODM_DEBUG("Enabling hybrid bundle adjustment")
                config.append("bundle_interval: 100")          # Bundle after adding 'bundle_interval' cameras
                config.append("bundle_new_points_ratio: 1.2")  # Bundle when (new points) / (bundled points) > bundle_new_points_ratio
                config.append("local_bundle_radius: 1")        # Max image graph distance for images to be included in local bundle adjustment

            if args.matcher_distance > 0:
                config.append("matching_gps_distance: %s" % self.params.matching_gps_distance)

            if tree.odm_georeferencing_gcp:
                config.append("bundle_use_gcp: yes")
                io.copy(tree.odm_georeferencing_gcp, tree.opensfm)

            # write config file
            log.ODM_DEBUG(config)
            config_filename = io.join_paths(tree.opensfm, 'config.yaml')
            with open(config_filename, 'w') as fout:
                fout.write("\n".join(config))

            # run OpenSfM reconstruction
            matched_done_file = io.join_paths(tree.opensfm, 'matching_done.txt')
            if not io.file_exists(matched_done_file) or rerun_cell:
                system.run('PYTHONPATH=%s %s/bin/opensfm extract_metadata %s' %
                           (context.pyopencv_path, context.opensfm_path, tree.opensfm))
                system.run('PYTHONPATH=%s %s/bin/opensfm detect_features %s' %
                           (context.pyopencv_path, context.opensfm_path, tree.opensfm))
                system.run('PYTHONPATH=%s %s/bin/opensfm match_features %s' %
                           (context.pyopencv_path, context.opensfm_path, tree.opensfm))
                with open(matched_done_file, 'w') as fout:
                    fout.write("Matching done!\n")
            else:
                log.ODM_WARNING('Found a feature matching done progress file in: %s' %
                                matched_done_file)

            if not io.file_exists(tree.opensfm_tracks) or rerun_cell:
                system.run('PYTHONPATH=%s %s/bin/opensfm create_tracks %s' %
                           (context.pyopencv_path, context.opensfm_path, tree.opensfm))
            else:
                log.ODM_WARNING('Found a valid OpenSfM tracks file in: %s' %
                                tree.opensfm_tracks)

            if not io.file_exists(tree.opensfm_reconstruction) or rerun_cell:
                system.run('PYTHONPATH=%s %s/bin/opensfm reconstruct %s' %
                           (context.pyopencv_path, context.opensfm_path, tree.opensfm))
            else:
                log.ODM_WARNING('Found a valid OpenSfM reconstruction file in: %s' %
                                tree.opensfm_reconstruction)

            # Always export VisualSFM's reconstruction and undistort images
            # as we'll use these for texturing (after GSD estimation and resizing)
            if not args.ignore_gsd:
                image_scale = gsd.image_scale_factor(args.orthophoto_resolution, tree.opensfm_reconstruction)
            else:
                image_scale = 1.0

            if not io.file_exists(tree.opensfm_reconstruction_nvm) or rerun_cell:
                system.run('PYTHONPATH=%s %s/bin/opensfm export_visualsfm --image_extension png --scale_focal %s %s' %
                            (context.pyopencv_path, context.opensfm_path, image_scale, tree.opensfm))
            else:
                log.ODM_WARNING('Found a valid OpenSfM NVM reconstruction file in: %s' %
                                tree.opensfm_reconstruction_nvm)

            # These will be used for texturing
            system.run('PYTHONPATH=%s %s/bin/opensfm undistort --image_format png --image_scale %s %s' %
                        (context.pyopencv_path, context.opensfm_path, image_scale, tree.opensfm))

            # Skip dense reconstruction if necessary and export
            # sparse reconstruction instead
            if args.fast_orthophoto:
                system.run('PYTHONPATH=%s %s/bin/opensfm export_ply --no-cameras %s' %
                        (context.pyopencv_path, context.opensfm_path, tree.opensfm))
            elif args.use_opensfm_dense:
                # Undistort images at full scale in JPG
                # (TODO: we could compare the size of the PNGs if they are < than depthmap_resolution
                # and use those instead of re-exporting full resolution JPGs)
                system.run('PYTHONPATH=%s %s/bin/opensfm undistort %s' %
                        (context.pyopencv_path, context.opensfm_path, tree.opensfm))
                system.run('PYTHONPATH=%s %s/bin/opensfm compute_depthmaps %s' %
                        (context.pyopencv_path, context.opensfm_path, tree.opensfm))

        else:
            log.ODM_WARNING('Found a valid OpenSfM reconstruction file in: %s' %
                            tree.opensfm_reconstruction)

        # check if reconstruction was exported to bundler before
        if not io.file_exists(tree.opensfm_bundle_list) or rerun_cell:
            # convert back to bundler's format
            system.run('PYTHONPATH=%s %s/bin/export_bundler %s' %
                       (context.pyopencv_path, context.opensfm_path, tree.opensfm))
        else:
            log.ODM_WARNING('Found a valid Bundler file in: %s' %
                            tree.opensfm_reconstruction)

        if reconstruction.georef:
            system.run('PYTHONPATH=%s %s/bin/opensfm export_geocoords %s --transformation --proj \'%s\'' %
                       (context.pyopencv_path, context.opensfm_path, tree.opensfm, reconstruction.georef.projection.srs))

        outputs.reconstruction = reconstruction

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'OpenSfM')

        log.ODM_INFO('Running ODM OpenSfM Cell - Finished')
        return ecto.OK if args.end_with != 'opensfm' else ecto.QUIT
