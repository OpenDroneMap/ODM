import ecto

from opendm import context
from opendm import types
from opendm import config

from dataset import ODMLoadDatasetCell
from resize import ODMResizeCell
from opensfm import ODMOpenSfMCell
from pmvs import ODMPmvsCell
from cmvs import ODMCmvsCell
from odm_meshing import ODMeshingCell
from odm_texturing import ODMTexturingCell
from odm_georeferencing import ODMGeoreferencingCell
from odm_orthophoto import ODMOrthoPhotoCell

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
        """
        Implement the virtual function from the base class
        Only cells from which something is forwarded have to be declared
        """
        cells = { 'args': ecto.Constant(value=p.args),
                  'dataset': ODMLoadDatasetCell(),
                  'resize': ODMResizeCell(),
                  'opensfm': ODMOpenSfMCell(use_exif_size=False,
                                            feature_process_size=p.args['resize_to'],
                                            feature_min_frames=p.args['min_num_features'],
                                            processes=context.num_cores,
                                            matching_gps_neighbors=p.args['matcher_k']),
                  'cmvs': ODMCmvsCell(max_images=p.args['cmvs_maxImages']),
                  'pmvs': ODMPmvsCell(level=p.args['pmvs_level'],
                                      csize=p.args['pmvs_csize'],
                                      thresh=p.args['pmvs_threshold'],
                                      wsize=p.args['pmvs_wsize'],
                                      min_imgs=p.args['pmvs_minImageNum'],
                                      cores=p.args['pmvs_num_cores']),
                  'meshing': ODMeshingCell(max_vertex=p.args['odm_meshing_maxVertexCount'],
                                           oct_tree=p.args['odm_meshing_octreeDepth'],
                                           samples=p.args['odm_meshing_samplesPerNode'],
                                           solver=p.args['odm_meshing_solverDivide']),
                  'texturing': ODMTexturingCell(resize=p.args['resize_to'],
                                                resolution=p.args['odm_texturing_textureResolution'],
                                                size=p.args['odm_texturing_textureWithSize']),
                  'georeferencing': ODMGeoreferencingCell(img_size=p.args['resize_to'],
                                                          gcp_file=p.args['odm_georeferencing_gcpFile'],
                                                          use_gcp=p.args['odm_georeferencing_useGcp']),
                  'orthophoto': ODMOrthoPhotoCell(resolution=p.args['odm_orthophoto_resolution'])

        }

        return cells    

    def configure(self, p, _i, _o):
        tree = types.ODMTree(p.args['project_path'])
        self.tree = ecto.Constant(value=tree)


    def connections(self, _p):
        # define initial task
        initial_task = _p.args['start_with']
        initial_task_id = config.processopts.index(initial_task)

        print _p.args['rerun']

        ## define the connections like you would for the plasm
        connections = []

        ## load the dataset
        connections = [ self.tree[:] >> self.dataset['tree'],
                        self.args[:] >> self.dataset['args'] ]

        ## check we want to start with resize task
        if initial_task_id <= config.processopts.index('resize'):

            # resize images
            connections += [ self.tree[:] >> self.resize['tree'],
                             self.args[:] >> self.resize['args'],
                             self.dataset['photos'] >> self.resize['photos'] ]
        
        # forward opensfm variables
        connections += [ self.tree[:] >> self.opensfm['tree'],
                         self.args[:] >> self.opensfm['args'] ]

        ## check we want to start with opensfm task
        if initial_task_id <= config.processopts.index('opensfm'):
            # run opensfm with images from load dataset
            connections += [ self.dataset['photos'] >> self.opensfm['photos'] ]
        else:
            # run opensfm with images from resize
            connections += [ self.resize['photos'] >> self.opensfm['photos'] ]

        ## check we want to start with cmvs task
        if initial_task_id <= config.processopts.index('cmvs'):

            # run cmvs
            connections += [ self.tree[:] >> self.cmvs['tree'],
                             self.args[:] >> self.cmvs['args'],
                             self.opensfm['reconstruction'] >> self.cmvs['reconstruction'] ]
        
        ## check we want to start with cmvs task
        if initial_task_id <= config.processopts.index('pmvs'):

            # run pmvs
            connections += [ self.tree[:] >> self.pmvs['tree'],
                             self.args[:] >> self.pmvs['args'],
                             self.cmvs['reconstruction'] >> self.pmvs['reconstruction'] ]

        ## check we want to start with odm_meshing task
        if initial_task_id <= config.processopts.index('odm_meshing'):

            # create odm mesh
            connections += [ self.tree[:] >> self.meshing['tree'],
                             self.args[:] >> self.meshing['args'],
                             self.pmvs['reconstruction'] >> self.meshing['reconstruction'] ]

        ## check we want to start with odm_texturing task
        if initial_task_id <= config.processopts.index('odm_texturing'):

            # create odm texture
            connections += [ self.tree[:] >> self.texturing['tree'],
                             self.args[:] >> self.texturing['args'],
                             self.meshing['reconstruction'] >> self.texturing['reconstruction'] ]
        
        ## check we want to start with odm_georeferencing task
        if initial_task_id <= config.processopts.index('odm_georeferencing'):

            # create odm georeference
            connections += [ self.tree[:] >> self.georeferencing['tree'],
                             self.args[:] >> self.georeferencing['args'],
                             self.dataset['photos'] >> self.georeferencing['photos'],
                             self.texturing['reconstruction'] >> self.georeferencing['reconstruction'] ]

        ## check we want to start with odm_orthophoto task
        if initial_task_id <= config.processopts.index('odm_orthophoto'):

            ## create odm orthophoto
            connections += [ self.tree[:] >> self.orthophoto['tree'],
                             self.args[:] >> self.orthophoto['args'],
                             self.georeferencing['reconstruction'] >> self.orthophoto['reconstruction'] ]

        return connections