import ecto

from opendm import context

from dataset import ODMLoadDatasetCell
from resize import ODMResizeCell
from opensfm import ODMOpenSfMCell
from pmvs import ODMPmvsCell
from odm_meshing import ODMeshingCell
from odm_texturing import ODMTexturingCell

class ODMApp(ecto.BlackBox):
    '''   ODMApp - a class for ODM Activities
    '''
    def __init__(self, *args, **kwargs):
        ecto.BlackBox.__init__(self, *args, **kwargs)

    @staticmethod
    def declare_direct_params(p):
        p.declare("args", "The application arguments.", {})

    @staticmethod
    def declare_cells(p):
        print p.args
        """
        Implement the virtual function from the base class
        Only cells from which something is forwarded have to be declared
        """
        cells = { 'args': ecto.Constant(value=p.args),
                  'load_dataset': ODMLoadDatasetCell(),
                  'resize': ODMResizeCell(),
                  'opensfm': ODMOpenSfMCell(use_exif_size=False,
                                            feature_process_size=p.args['resize_to'],
                                            feature_min_frames=p.args['min_num_features'],
                                            processes=context.num_cores,
                                            matching_gps_neighbors=p.args['matcher_k']),
                  'pmvs': ODMPmvsCell(),
                  'meshing': ODMeshingCell(),
                  'texturing': ODMTexturingCell()

        }

        return cells

    def connections(self, _p):
        # define initial and final tasks
        # TODO: not sure how to manage that
        initial_task = _p.args['start_with']
        final_task = _p.args['end_with']
        run_only = _p.args['run_only']

        # define the connections like you would for the plasm
        connections = []

        # load the dataset
        connections = [ self.args[:] >> self.load_dataset['args'] ]

        # resize images
        connections += [ self.args[:] >> self.resize['args'],
                         self.load_dataset['photos'] >> self.resize['photos'] ]

        # run opensfm
        connections += [ self.args[:] >> self.opensfm['args'],
                         self.load_dataset['photos'] >> self.opensfm['photos'] ]

        # run cmvs
        connections += [ self.args[:] >> self.pmvs['args'],
                         self.opensfm['reconstruction_path'] >> 
                         self.pmvs['reconstruction_path'] ]

        # create odm mesh
        connections += [ self.args[:] >> self.meshing['args'],
                         self.pmvs['model_path'] >> self.meshing['model_path'] ]

        # create odm texture
        connections += [ self.args[:] >> self.texturing['args'],
                         self.meshing['mesh_path'] >> self.texturing['model_path'] ]

        return connections