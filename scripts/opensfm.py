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
        inputs.declare("project_path", "The directory to the images to load.", "")
        inputs.declare("photos", "Clusters output. list of ODMPhoto's", [])
        inputs.declare("reconstructions", "Clusters output. list of reconstructions", [])

    def process(self, inputs, outputs):

        log.ODM_INFO('Running OMD OpenSfm Cell')

        # get inputs
        photos = self.inputs.photos
        project_path = io.absolute_path_file(self.inputs.project_path)

        # create file list directory
        list_path = io.join_paths(project_path, 'opensfm')
        system.mkdir_p(list_path)

        # check if reconstruction was done before
        file_list_path = io.join_paths(list_path, 'image_list.txt')

        if io.file_exists(file_list_path):
            log.ODM_WARNING('Found a valid reconstruction file')
            log.ODM_INFO('Running OMD OpenSfm Cell - Finished')
            return

        # create file list
        with open(file_list_path, 'w') as fout:
            for photo in photos:
                fout.write('%s\n' % photo.path_file)

        # create config file for OpenSfM
        config = [
            "use_exif_size: no",
            "feature_process_size: %s" % self.params.feature_process_size,
            "feature_min_frames: %s" % self.params.feature_min_frames,
            "processes: %s" % self.params.processes,
            "matching_gps_neighbors: %s" % self.params.matching_gps_neighbors
        ]

        # write config file
        config_filename = io.join_paths(project_path, 'config.yaml')
        with open(config_filename, 'w') as fout:
            fout.write("\n".join(config))

        # Run OpenSfM reconstruction
        system.run('PYTHONPATH=%s %s/bin/run_all %s' % 
            (context.pyopencv_path, context.opensfm_path, list_path))

        # append reconstructions to output
        self.outputs.reconstructions = []
        
        log.ODM_INFO('Running OMD OpenSfm Cell - Finished')

#        # Convert back to bundler's format
#        run('PYTHONPATH={} "{}/bin/export_bundler" opensfm'.format(PYOPENCV_PATH, OPENSFM_PATH))
#
#        bundler_to_pmvs("opensfm/bundle_r000.out")


#########################################################################################


class ODMLoadReconstructionCell(ecto.Cell):    

    def declare_io(self, params, inputs, outputs):
        inputs.declare("project_path", "The directory to the images to load.", "")
        inputs.declare("photos", "Clusters output. list of ODMPhoto's", [])
        outputs.declare("reconstructions", "Clusters output. list of reconstructions", [])

    def process(self, inputs, outputs):        
        log.ODM_INFO('Running OMD Load Reconstruction Cell')
        log.ODM_INFO('Running OMD Load Reconstruction Cell - Finished')


#########################################################################################


class ODMConvertToBundleCell(ecto.Cell):    

    def declare_io(self, params, inputs, outputs):
        inputs.declare("project_path", "The directory to the images to load.", "")
        inputs.declare("reconstructions", "The directory to the images to load.", "")
        outputs.declare("bundler_file_path", "The directory to the images to load.", "")

    def process(self, inputs, outputs):

        log.ODM_INFO('Running OMD Convert to Bundle Cell')

        # get inputs
        reconstructions = self.inputs.reconstructions
        project_path = io.absolute_path_file(self.inputs.project_path)

        # create file list directory
        list_path = io.join_paths(project_path, 'opensfm')
        bundler_path_file =  io.join_paths(list_path, 'bundle_r000.out')
        
        # Run OpenSfM reconstruction
#        system.run('PYTHONPATH=%s %s/bin/export_bundler %s' % 
#            (context.pyopencv_path, context.opensfm_path, list_path))
        
        system.run('PYTHONPATH=%s %s/bin/export_pmvs %s' % 
            (context.pyopencv_path, context.opensfm_path, list_path))

        # appends created file to output
        self.outputs.bundler_file_path = bundler_path_file
        
        if io.file_exists(bundler_path_file):
            log.ODM_DEBUG('Bundler file created to: %s' % bundler_path_file)
        else:
            log.ODM_ERROR('Something went wrong when exporting to Bundler')
            return

        log.ODM_INFO('Running OMD Convert to Bundle Cell - Finished')

