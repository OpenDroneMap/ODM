import ecto
import os

from opendm import context
from opendm import types
from opendm import io
from opendm import system

from dataset import ODMLoadDatasetCell
from resize import ODMResizeCell
from opensfm import ODMOpenSfMCell
from odm_slam import ODMSlamCell
from pmvs import ODMPmvsCell
from cmvs import ODMCmvsCell
from odm_meshing import ODMeshingCell
#from odm_texturing import ODMTexturingCell
from mvstex import ODMMvsTexCell
from odm_georeferencing import ODMGeoreferencingCell
from odm_orthophoto import ODMOrthoPhotoCell


class ODMApp(ecto.BlackBox):
    """ODMApp - a class for ODM Activities
    """

    def __init__(self, *args, **kwargs):
        ecto.BlackBox.__init__(self, *args, **kwargs)
        self.tree = None

    @staticmethod
    def declare_direct_params(p):
        p.declare("args", "The application arguments.", {})

    @staticmethod
    def declare_cells(p):
        """
        Implement the virtual function from the base class
        Only cells from which something is forwarded have to be declared
        """
        cells = {'args': ecto.Constant(value=p.args),
                 'dataset': ODMLoadDatasetCell(force_focal=p.args.force_focal,
                                               force_ccd=p.args.force_ccd),
                 'resize': ODMResizeCell(resize_to=p.args.resize_to),
                 'opensfm': ODMOpenSfMCell(use_exif_size=False,
                                           feature_process_size=p.args.resize_to,
                                           feature_min_frames=p.args.min_num_features,
                                           processes=p.args.opensfm_processes,
                                           matching_gps_neighbors=p.args.matcher_neighbors,
                                           matching_gps_distance=p.args.matcher_distance),
                 'slam': ODMSlamCell(),
                 'cmvs': ODMCmvsCell(max_images=p.args.cmvs_maxImages),
                 'pmvs': ODMPmvsCell(level=p.args.pmvs_level,
                                     csize=p.args.pmvs_csize,
                                     thresh=p.args.pmvs_threshold,
                                     wsize=p.args.pmvs_wsize,
                                     min_imgs=p.args.pmvs_min_images,
                                     cores=p.args.pmvs_num_cores),
                 'meshing': ODMeshingCell(max_vertex=p.args.mesh_size,
                                          oct_tree=p.args.mesh_octree_depth,
                                          samples=p.args.mesh_samples,
                                          solver=p.args.mesh_solver_divide,
                                          verbose=p.args.verbose),
                 'texturing': ODMMvsTexCell(data_term=p.args.texturing_data_term,
                                            outlier_rem_type=p.args.texturing_outlier_removal_type,
                                            skip_vis_test=p.args.texturing_skip_visibility_test,
                                            skip_glob_seam_leveling=p.args.texturing_skip_global_seam_leveling,
                                            skip_loc_seam_leveling=p.args.texturing_skip_local_seam_leveling,
                                            skip_hole_fill=p.args.texturing_skip_hole_filling,
                                            keep_unseen_faces=p.args.texturing_keep_unseen_faces),
                 'georeferencing': ODMGeoreferencingCell(img_size=p.args.resize_to,
                                                         gcp_file=p.args.gcp,
                                                         use_exif=p.args.use_exif,
                                                         verbose=p.args.verbose),
                 'orthophoto': ODMOrthoPhotoCell(resolution=p.args.orthophoto_resolution,
                                                 verbose=p.args.verbose)
                 }

        return cells

    def configure(self, p, _i, _o):
        tree = types.ODM_Tree(p.args.project_path, p.args.images)
        self.tree = ecto.Constant(value=tree)

        # TODO(dakota) put this somewhere better maybe
        if p.args.time and io.file_exists(tree.benchmarking):
            # Delete the previously made file
            os.remove(tree.benchmarking)
            with open(tree.benchmarking, 'a') as b:
                b.write('ODM Benchmarking file created %s\nNumber of Cores: %s\n\n' % (system.now(), context.num_cores))

    def connections(self, _p):
        if _p.args.video:
            return self.slam_connections(_p)

        # define initial task
        # TODO: What is this?
        # initial_task = _p.args['start_with']
        # initial_task_id = config.processopts.index(initial_task)

        # define the connections like you would for the plasm

        # load the dataset
        connections = [self.tree[:] >> self.dataset['tree']]

        # run resize cell
        connections += [self.tree[:] >> self.resize['tree'],
                        self.args[:] >> self.resize['args'],
                        self.dataset['photos'] >> self.resize['photos']]

        # run opensfm with images from load dataset
        connections += [self.tree[:] >> self.opensfm['tree'],
                        self.args[:] >> self.opensfm['args'],
                        self.resize['photos'] >> self.opensfm['photos']]

        if _p.args.use_opensfm_pointcloud:
            # create odm mesh from opensfm point cloud
            connections += [self.tree[:] >> self.meshing['tree'],
                            self.args[:] >> self.meshing['args'],
                            self.opensfm['reconstruction'] >> self.meshing['reconstruction']]
        else:
            # run cmvs
            connections += [self.tree[:] >> self.cmvs['tree'],
                            self.args[:] >> self.cmvs['args'],
                            self.opensfm['reconstruction'] >> self.cmvs['reconstruction']]

            # run pmvs
            connections += [self.tree[:] >> self.pmvs['tree'],
                            self.args[:] >> self.pmvs['args'],
                            self.cmvs['reconstruction'] >> self.pmvs['reconstruction']]

            # create odm mesh from pmvs point cloud
            connections += [self.tree[:] >> self.meshing['tree'],
                            self.args[:] >> self.meshing['args'],
                            self.pmvs['reconstruction'] >> self.meshing['reconstruction']]

        # create odm texture
        connections += [self.tree[:] >> self.texturing['tree'],
                        self.args[:] >> self.texturing['args'],
                        self.meshing['reconstruction'] >> self.texturing['reconstruction']]

        # create odm georeference
        connections += [self.tree[:] >> self.georeferencing['tree'],
                        self.args[:] >> self.georeferencing['args'],
                        self.resize['photos'] >> self.georeferencing['photos'],
                        self.texturing['reconstruction'] >> self.georeferencing['reconstruction']]

        # create odm orthophoto
        connections += [self.tree[:] >> self.orthophoto['tree'],
                        self.args[:] >> self.orthophoto['args'],
                        self.georeferencing['reconstruction'] >> self.orthophoto['reconstruction']]

        return connections

    def slam_connections(self, _p):
        """Get connections used when running from video instead of images."""
        connections = []

        # run slam cell
        connections += [self.tree[:] >> self.slam['tree'],
                        self.args[:] >> self.slam['args']]

        # run cmvs
        connections += [self.tree[:] >> self.cmvs['tree'],
                        self.args[:] >> self.cmvs['args'],
                        self.slam['reconstruction'] >> self.cmvs['reconstruction']]

        # run pmvs
        connections += [self.tree[:] >> self.pmvs['tree'],
                        self.args[:] >> self.pmvs['args'],
                        self.cmvs['reconstruction'] >> self.pmvs['reconstruction']]

        # create odm mesh
        connections += [self.tree[:] >> self.meshing['tree'],
                        self.args[:] >> self.meshing['args'],
                        self.pmvs['reconstruction'] >> self.meshing['reconstruction']]

        # create odm texture
        connections += [self.tree[:] >> self.texturing['tree'],
                        self.args[:] >> self.texturing['args'],
                        self.meshing['reconstruction'] >> self.texturing['reconstruction']]

        return connections
