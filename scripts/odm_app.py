import ecto

from dataset import ODMLoadDatasetCell
#from resize import OMDResizeCell
from cmvs import ODMCmvsCell
from opensfm import ODMOpenSfMCell, ODMLoadReconstructionCell, ODMConvertToBundleCell

class ODMApp(ecto.BlackBox):
    '''   ODMApp - a class for ODM Activities
    '''
    def __init__(self, *args, **kwargs):
        ecto.BlackBox.__init__(self, *args, **kwargs)

    @staticmethod
    def declare_direct_params(p):
        p.declare("args", "The application arguments.", {})
        p.declare("project_path", "The directory to the project.", "")

    @staticmethod
    def declare_cells(p):
        print p.args
        """
        Implement the virtual function from the base class
        Only cells from which something is forwarded have to be declared
        """
        cells = { 'project_path': ecto.Constant(value=p.project_path),
                  'args': ecto.Constant(value=p.args),
                  'load_dataset': ODMLoadDatasetCell(),
                  #'resize': OMDResizeCell()
                  'opensfm': ODMOpenSfMCell(use_exif_size=False,
                                            feature_process_size=p.args['resize_to'],
                                            feature_min_frames=p.args['min_num_features'],
                                            processes=8,
                                            matching_gps_neighbors=p.args['matcher_k']),
                  'load_reconstruction': ODMLoadReconstructionCell(),
                  'convert_reconstruction': ODMConvertToBundleCell(),
                  'cmvs': ODMCmvsCell(cmvs_max_images=p.args['cmvs_maxImages'],
                                    pmvs_level=p.args['pmvs_level'],
                                    pmvs_csize=p.args['pmvs_csize'],
                                    pmvs_threshold=p.args['pmvs_threshold'],
                                    pmvs_wsize=p.args['pmvs_wsize'],
                                    pmvs_min_image_num=p.args['pmvs_minImageNum'])

        }

        return cells

    def connections(self, _p):
        # define initial and final tasks
        # TODO: not sure how to manage that
        initial_task = _p.args['start_with']
        final_task = _p.args['end_with']
        run_only = _p.args['run_only']
        print run_only

        # define the connections like you would for the plasm
        connections = []

        # load the dataset
        connections = [ self.project_path[:] >> self.load_dataset['images_dir'],
                        self.args[:] >> self.load_dataset['args'] ]
        
        # resize images
        #connections += [ self.load_dataset['photos'] >> self.resize['photos'] ]

        # connect loaded data with opensfm
        connections += [ self.project_path[:] >> self.opensfm['project_path'],
                         self.load_dataset['photos'] >> self.opensfm['photos'] ]

        # load reconstruction
        connections += [ self.project_path[:] >> self.load_reconstruction['project_path'],
                         self.load_dataset['photos'] >> self.load_reconstruction['photos'] ]

        # convert data to Bundler format
        connections += [ self.project_path[:] >> self.convert_reconstruction['project_path'],
                         self.load_reconstruction['reconstructions'] >> self.convert_reconstruction['reconstructions'] ]

        # run cmvs
#        connections += [ self.project_path[:] >> self.cmvs['project_path'],
#                         self.convert_reconstruction['bundler_file_path'] >> self.cmvs['bundler_file_path'] ]

        return connections