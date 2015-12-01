import ecto

from opendm import log
from opendm import io
from opendm import system
from opendm import context

class ODMOpenSfMCell(ecto.Cell):

    def declare_params(self, params):
        params.declare("use_exif_size", "The application arguments.", False)
        params.declare("feature_process_size", "The application arguments.", False)
        params.declare("feature_min_frames", "The application arguments.", 0)
        params.declare("processes", "The application arguments.", 0)
        params.declare("matching_gps_neighbors", "The application arguments.", 0)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("photos", "Clusters output. list of ODMPhoto's", [])
        outputs.declare("reconstructions", "Clusters output. list of reconstructions", [])
        outputs.declare("reconstruction_path", "The directory to the images to load.", "")

    def process(self, inputs, outputs):

        log.ODM_INFO('Running OMD OpenSfm Cell')

        # get inputs
        args = self.inputs.args
        photos = self.inputs.photos
        project_path = io.absolute_path_file(args['project_path'])

        if not photos:
            log.ODM_ERROR('Not enough photos in photos array to start OpenSfm')
            return ecto.QUIT

        # create working directories
        opensfm_path = io.join_paths(project_path, 'opensfm')
        pmvs_path = io.join_paths(project_path, 'pmvs')        
        system.mkdir_p(opensfm_path)
        system.mkdir_p(pmvs_path)

        # check if we rerun cell or not
        rerun_cell = args['run_only'] is not None \
            and args['run_only'] == 'opensfm'

        ### check if reconstruction was done before
        reconstruction_file = io.join_paths(opensfm_path, 'reconstruction.json')

        if not io.file_exists(reconstruction_file) or rerun_cell:
            # create file list
            list_path = io.join_paths(opensfm_path, 'image_list.txt')
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

            # write config file
            config_filename = io.join_paths(project_path, 'config.yaml')
            with open(config_filename, 'w') as fout:
                fout.write("\n".join(config))

            # run OpenSfM reconstruction
            system.run('PYTHONPATH=%s %s/bin/run_all %s' % 
                (context.pyopencv_path, context.opensfm_path, opensfm_path))
        else:
            log.ODM_WARNING('Found a valid reconstruction file in: %s' % 
                (reconstruction_file))

        ### check if reconstruction was exported to bundler before
        bundler_file = io.join_paths(opensfm_path, 'bundle_r000.out')

        if not io.file_exists(bundler_file) or rerun_cell:
            # convert back to bundler's format
            system.run('PYTHONPATH=%s %s/bin/export_bundler %s' %
                (context.pyopencv_path, context.opensfm_path, opensfm_path))
        else:
            log.ODM_WARNING('Found a valid bundle file in: %s' % 
                (reconstruction_file))

        ### check if reconstruction was exported to pmvs before
        pmvs_file = io.join_paths(pmvs_path, 'recon0/pmvs_options.txt')

        if not io.file_exists(pmvs_file) or rerun_cell:
            # run PMVS converter
            system.run('PYTHONPATH=%s %s/bin/export_pmvs %s --output %s' % 
                (context.pyopencv_path, context.opensfm_path, opensfm_path, pmvs_path))
        else:
            log.ODM_WARNING('Found a valid PMVS file in: %s' % pmvs_file)

        # append biggest reconstruction path to output
        self.outputs.reconstruction_path =  io.join_paths(pmvs_path, 'recon0')

        log.ODM_INFO('Running OMD OpenSfm Cell - Finished')
        return ecto.OK if args['end_with'] != 'opensfm' else ecto.QUIT