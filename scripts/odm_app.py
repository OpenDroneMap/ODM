import ecto
import os

from opendm import context
from opendm import types
from opendm import io
from opendm import system

from dataset import ODMLoadDatasetCell
from run_opensfm import ODMOpenSfMCell
from mve import ODMMveCell
from odm_slam import ODMSlamCell
from odm_meshing import ODMeshingCell
from mvstex import ODMMvsTexCell
from odm_georeferencing import ODMGeoreferencingCell
from odm_orthophoto import ODMOrthoPhotoCell
from odm_dem import ODMDEMCell
from odm_filterpoints import ODMFilterPoints


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
                 'dataset': ODMLoadDatasetCell(verbose=p.args.verbose,
                                               proj=p.args.proj),
                 'opensfm': ODMOpenSfMCell(use_exif_size=False,
                                           feature_process_size=p.args.resize_to,
                                           feature_min_frames=p.args.min_num_features,
                                           processes=p.args.max_concurrency,
                                           matching_gps_neighbors=p.args.matcher_neighbors,
                                           matching_gps_distance=p.args.matcher_distance,
                                           fixed_camera_params=p.args.use_fixed_camera_params,
                                           hybrid_bundle_adjustment=p.args.use_hybrid_bundle_adjustment),
                 'slam': ODMSlamCell(),
                 
                 'mve': ODMMveCell(),

                 'filterpoints': ODMFilterPoints(),
                 
                 'meshing': ODMeshingCell(max_vertex=p.args.mesh_size,
                                          oct_tree=p.args.mesh_octree_depth,
                                          samples=p.args.mesh_samples,
                                          point_weight=p.args.mesh_point_weight,
                                          max_concurrency=p.args.max_concurrency,
                                          verbose=p.args.verbose),
                 'texturing': ODMMvsTexCell(data_term=p.args.texturing_data_term,
                                            outlier_rem_type=p.args.texturing_outlier_removal_type,
                                            skip_vis_test=p.args.texturing_skip_visibility_test,
                                            skip_glob_seam_leveling=p.args.texturing_skip_global_seam_leveling,
                                            skip_loc_seam_leveling=p.args.texturing_skip_local_seam_leveling,
                                            skip_hole_fill=p.args.texturing_skip_hole_filling,
                                            keep_unseen_faces=p.args.texturing_keep_unseen_faces,
                                            tone_mapping=p.args.texturing_tone_mapping),
                 'georeferencing': ODMGeoreferencingCell(gcp_file=p.args.gcp,
                                                         use_exif=p.args.use_exif,
                                                         verbose=p.args.verbose),
                 'dem': ODMDEMCell(max_concurrency=p.args.max_concurrency,
                                   verbose=p.args.verbose),
                 'orthophoto': ODMOrthoPhotoCell(resolution=p.args.orthophoto_resolution,
                                                 no_tiled=p.args.orthophoto_no_tiled,
                                                 compress=p.args.orthophoto_compression,
                                                 bigtiff=p.args.orthophoto_bigtiff,
                                                 build_overviews=p.args.build_overviews,
                                                 max_concurrency=p.args.max_concurrency,
                                                 verbose=p.args.verbose)
                 }

        return cells

    def configure(self, p, _i, _o):
        tree = types.ODM_Tree(p.args.project_path, p.args.images, p.args.gcp)
        self.tree = ecto.Constant(value=tree)

        # TODO(dakota) put this somewhere better maybe
        if p.args.time and io.file_exists(tree.benchmarking):
            # Delete the previously made file
            os.remove(tree.benchmarking)
            with open(tree.benchmarking, 'a') as b:
                b.write('ODM Benchmarking file created %s\nNumber of Cores: %s\n\n' % (system.now(), context.num_cores))

    def connections(self, p):
        if p.args.video:
            return self.slam_connections(p)

        # load the dataset
        connections = [self.tree[:] >> self.dataset['tree'],
                       self.args[:] >> self.dataset['args']]

        # run opensfm with images from load dataset
        connections += [self.tree[:] >> self.opensfm['tree'],
                        self.args[:] >> self.opensfm['args'],
                        self.dataset['reconstruction'] >> self.opensfm['reconstruction']]

        if p.args.use_opensfm_dense or p.args.fast_orthophoto:
            # filter points from opensfm point cloud
            connections += [self.tree[:] >> self.filterpoints['tree'],
                            self.args[:] >> self.filterpoints['args'],
                            self.opensfm['reconstruction'] >> self.filterpoints['reconstruction']]
        else:
            # run mve
            connections += [self.tree[:] >> self.mve['tree'],
                            self.args[:] >> self.mve['args'],
                            self.opensfm['reconstruction'] >> self.mve['reconstruction']]

            # filter points from mve point cloud
            connections += [self.tree[:] >> self.filterpoints['tree'],
                            self.args[:] >> self.filterpoints['args'],
                            self.mve['reconstruction'] >> self.filterpoints['reconstruction']]

        # create mesh
        connections += [self.tree[:] >> self.meshing['tree'],
                        self.args[:] >> self.meshing['args'],
                        self.filterpoints['reconstruction'] >> self.meshing['reconstruction']]

        # create odm texture
        connections += [self.tree[:] >> self.texturing['tree'],
                        self.args[:] >> self.texturing['args'],
                        self.meshing['reconstruction'] >> self.texturing['reconstruction']]

        # create odm georeference
        connections += [self.tree[:] >> self.georeferencing['tree'],
                        self.args[:] >> self.georeferencing['args'],
                        self.texturing['reconstruction'] >> self.georeferencing['reconstruction']]

        # create odm dem
        connections += [self.tree[:] >> self.dem['tree'],
                        self.args[:] >> self.dem['args'],
                        self.georeferencing['reconstruction'] >> self.dem['reconstruction']]

        # create odm orthophoto
        connections += [self.tree[:] >> self.orthophoto['tree'],
                        self.args[:] >> self.orthophoto['args'],
                        self.georeferencing['reconstruction'] >> self.orthophoto['reconstruction']]
        return connections

    def slam_connections(self, p):
        """Get connections used when running from video instead of images."""
        connections = []

        # run slam cell
        connections += [self.tree[:] >> self.slam['tree'],
                        self.args[:] >> self.slam['args']]

        connections += [self.tree[:] >> self.mve['tree'],
                        self.args[:] >> self.mve['args'],
                        self.slam['reconstruction'] >> self.mve['reconstruction']]

        # create odm mesh
        connections += [self.tree[:] >> self.meshing['tree'],
                        self.args[:] >> self.meshing['args'],
                        self.mve['reconstruction'] >> self.meshing['reconstruction']]

        # create odm texture
        connections += [self.tree[:] >> self.texturing['tree'],
                        self.args[:] >> self.texturing['args'],
                        self.meshing['reconstruction'] >> self.texturing['reconstruction']]

        return connections
