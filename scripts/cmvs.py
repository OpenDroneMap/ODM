import ecto

from opendm import io
from opendm import log
from opendm import system

class ODMCmvsCell(ecto.Cell):    
    def declare_params(self, params):
        params.declare("cmvs_max_images", "The application arguments.", False)
        params.declare("pmvs_level", "The application arguments.", False)
        params.declare("pmvs_csize", "The application arguments.", 0)
        params.declare("pmvs_threshold", "The application arguments.", 0)
        params.declare("pmvs_wsize", "The application arguments.", 0)
        params.declare("pmvs_min_image_num", "The application arguments.", 0)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("project_path", "The directory to the images to load.", "")
        inputs.declare("bundler_file_path", "Clusters output. list of reconstructions", [])

    def process(self, inputs, outputs):
        
        log.ODM_INFO('Running OMD CMVS Cell')

        # get inputs
        bundler_file_path = self.inputs.bundler_file_path
        project_path = io.absolute_path_file(self.inputs.project_path)

        # run cmvs binary
        #bin_dir='/home/vagrant/software/OpenDroneMap-edgarriba/SuperBuild/build/cmvs/main/cmvs'
        bin_dir='/home/vagrant/software/OpenDroneMap-edgarriba/SuperBuild/build/cmvs/main/pmv2'

        cmvs_max_images = self.params.cmvs_max_images
        num_cores = 4

        system.run('%s %s %s %s' % 
            (bin_dir, bundler_file_path, cmvs_max_images, num_cores))


        log.ODM_INFO('Running OMD CMVS Cell - Finished')