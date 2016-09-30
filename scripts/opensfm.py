import ecto

from opendm import log
from opendm import io
from opendm import system
from opendm import context


class ODMOpenSfMCell(ecto.Cell):
    def declare_params(self, params):
        params.declare("use_exif_size", "The application arguments.", False)
        params.declare("feature_process_size", "The application arguments.", 2400)
        params.declare("feature_min_frames", "The application arguments.", 4000)
        params.declare("processes", "The application arguments.", context.num_cores)
        params.declare("matching_gps_neighbors", "The application arguments.", 8)
        params.declare("matching_gps_distance", "The application arguments.", 0)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("photos", "list of ODMPhoto's", [])
        outputs.declare("reconstruction", "list of ODMReconstructions", [])

    def process(self, inputs, outputs):

        # Benchmarking
        start_time = system.now_raw()

        log.ODM_INFO('Running ODM OpenSfM Cell')

        # get inputs
        tree = self.inputs.tree
        args = self.inputs.args
        photos = self.inputs.photos

        if not photos:
            log.ODM_ERROR('Not enough photos in photos array to start OpenSfM')
            return ecto.QUIT

        # create working directories     
        system.mkdir_p(tree.opensfm)
        system.mkdir_p(tree.pmvs)

        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'opensfm') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'opensfm' in args.rerun_from)

        if args.use_opensfm_pointcloud:
            output_file = tree.opensfm_model
        else:
            output_file = tree.opensfm_reconstruction

        # check if reconstruction was done before
        if not io.file_exists(output_file) or rerun_cell:
            # create file list
            list_path = io.join_paths(tree.opensfm, 'image_list.txt')
            with open(list_path, 'w') as fout:
                for photo in photos:
                    fout.write('%s\n' % photo.path_file)

            # create config file for OpenSfM
            config = [
                "use_exif_size: %s" % ('no' if not self.params.use_exif_size else 'yes'),
                "feature_process_size: %s" % self.params.feature_process_size,
                "feature_min_frames: %s" % self.params.feature_min_frames,
                "processes: %s" % self.params.processes,
                "matching_gps_neighbors: %s" % self.params.matching_gps_neighbors
            ]

            if args.matcher_distance > 0:
                config.append("matching_gps_distance: %s" % self.params.matching_gps_distance)

            # write config file
            config_filename = io.join_paths(tree.opensfm, 'config.yaml')
            with open(config_filename, 'w') as fout:
                fout.write("\n".join(config))

            # run OpenSfM reconstruction
            system.run('PYTHONPATH=%s %s/bin/run_all %s' %
                       (context.pyopencv_path, context.opensfm_path, tree.opensfm))
            if args.use_opensfm_pointcloud:
                system.run('PYTHONPATH=%s %s/bin/opensfm export_visualsfm %s' %
                           (context.pyopencv_path, context.opensfm_path, tree.opensfm))
                system.run('PYTHONPATH=%s %s/bin/opensfm undistort %s' %
                           (context.pyopencv_path, context.opensfm_path, tree.opensfm))
                system.run('PYTHONPATH=%s %s/bin/opensfm compute_depthmaps %s' %
                           (context.pyopencv_path, context.opensfm_path, tree.opensfm))
        else:
            log.ODM_WARNING('Found a valid OpenSfM file in: %s' %
                            tree.opensfm_reconstruction)

        # check if reconstruction was exported to bundler before
        if not io.file_exists(tree.opensfm_bundle_list) or rerun_cell:
            # convert back to bundler's format
            system.run('PYTHONPATH=%s %s/bin/export_bundler %s' %
                       (context.pyopencv_path, context.opensfm_path, tree.opensfm))
        else:
            log.ODM_WARNING('Found a valid Bundler file in: %s' %
                            tree.opensfm_reconstruction)

        if not args.use_opensfm_pointcloud:
            # check if reconstruction was exported to pmvs before
            if not io.file_exists(tree.pmvs_visdat) or rerun_cell:
                # run PMVS converter
                system.run('PYTHONPATH=%s %s/bin/export_pmvs %s --output %s' %
                           (context.pyopencv_path, context.opensfm_path, tree.opensfm, tree.pmvs))
            else:
                log.ODM_WARNING('Found a valid CMVS file in: %s' % tree.pmvs_visdat)

            if args.time:
                system.benchmark(start_time, tree.benchmarking, 'OpenSfM')

        log.ODM_INFO('Running ODM OpenSfM Cell - Finished')
        return ecto.OK if args.end_with != 'opensfm' else ecto.QUIT
