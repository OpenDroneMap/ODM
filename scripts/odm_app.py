import ecto
import os

from opendm import context
from opendm import types
from opendm import io
from opendm import system

from dataset import ODMLoadDatasetCell
from run_opensfm import ODMOpenSfMCell
from smvs import ODMSmvsCell
from odm_slam import ODMSlamCell
from odm_meshing import ODMeshingCell
from mvstex import ODMMvsTexCell
from odm_georeferencing import ODMGeoreferencingCell
from odm_orthophoto import ODMOrthoPhotoCell
from odm_dem import ODMDEMCell
from metadataset.setup import SMSetupCell
from metadataset.run_matching import SMMatchingCell
from metadataset.split import SMSplitCell
from metadataset.run_reconstructions import SMReconstructionCell
from metadataset.align import SMAlignCell
from metadataset.run_dense import SMDenseCell
from metadataset.merge import SMMergeCell

class ODMApp(ecto.BlackBox):
    """ODMApp - a class for ODM Activities
    """

    def __init__(self, *args, **kwargs):
        ecto.BlackBox.__init__(self, *args, **kwargs)
        self.tree = None
        self.split_merge = None

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
                                               force_ccd=p.args.force_ccd,
                                               verbose=p.args.verbose,
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
                 'smvs': ODMSmvsCell(alpha=p.args.smvs_alpha,
                                     max_pixels=p.args.depthmap_resolution*p.args.depthmap_resolution,
                                     threads=p.args.max_concurrency,
                                     output_scale=p.args.smvs_output_scale,
                                     shading=p.args.smvs_enable_shading,
                                     gamma_srgb=p.args.smvs_gamma_srgb,
                                     verbose=p.args.verbose),
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
                                                 t_srs=p.args.orthophoto_target_srs,
                                                 no_tiled=p.args.orthophoto_no_tiled,
                                                 compress=p.args.orthophoto_compression,
                                                 bigtiff=p.args.orthophoto_bigtiff,
                                                 build_overviews=p.args.build_overviews,
                                                 max_concurrency=p.args.max_concurrency,
                                                 verbose=p.args.verbose),
                 ### Split-Merge cells
                 'setup': SMSetupCell(verbose=p.args.verbose),
                 'matching': SMMatchingCell(),
                 'split': SMSplitCell(),
                 'reconstruction': SMReconstructionCell(),
                 'align': SMAlignCell(),
                 'dense': SMDenseCell(),
                 'merge': SMMergeCell(merge_overwrite=p.args.merge_overwrite) 
                 }

        return cells

    def configure(self, p, _i, _o):
        tree = types.ODM_Tree(p.args.project_path, p.args.images, p.args.gcp)
        self.tree = ecto.Constant(value=tree)

        self.sm_meta = types.SplitMerge(p.args.name)

        # TODO(dakota) put this somewhere better maybe
        if p.args.time and io.file_exists(tree.benchmarking):
            # Delete the previously made file
            os.remove(tree.benchmarking)
            with open(tree.benchmarking, 'a') as b:
                b.write('ODM Benchmarking file created %s\nNumber of Cores: %s\n\n' % (system.now(), context.num_cores))

    def connections(self, p):
        if p.args.video:
            return self.slam_connections(p)

        if p.args.large:
            return self.sm_connections(p)

        # define initial task
        # TODO: What is this?
        # initial_task = p.args['start_with']
        # initial_task_id = config.processopts.index(initial_task)

        # define the connections like you would for the plasm

        # load the dataset
        connections = [self.tree[:] >> self.dataset['tree'],
                       self.args[:] >> self.dataset['args']]

        # run opensfm with images from load dataset
        connections += [self.tree[:] >> self.opensfm['tree'],
                        self.args[:] >> self.opensfm['args'],
                        self.dataset['reconstruction'] >> self.opensfm['reconstruction']]

        if p.args.use_opensfm_dense or p.args.fast_orthophoto:
            # create odm mesh from opensfm point cloud
            connections += [self.tree[:] >> self.meshing['tree'],
                            self.args[:] >> self.meshing['args'],
                            self.opensfm['reconstruction'] >> self.meshing['reconstruction']]
        else:
            # run smvs

            connections += [self.tree[:] >> self.smvs['tree'],
                            self.args[:] >> self.smvs['args'],
                            self.opensfm['reconstruction'] >> self.smvs['reconstruction']]

            # create odm mesh from smvs point cloud
            connections += [self.tree[:] >> self.meshing['tree'],
                            self.args[:] >> self.meshing['args'],
                            self.smvs['reconstruction'] >> self.meshing['reconstruction']]

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

        connections += [self.tree[:] >> self.smvs['tree'],
                        self.args[:] >> self.smvs['args'],
                        self.slam['reconstruction'] >> self.smvs['reconstruction']]

        # create odm mesh
        connections += [self.tree[:] >> self.meshing['tree'],
                        self.args[:] >> self.meshing['args'],
                        self.smvs['reconstruction'] >> self.meshing['reconstruction']]

        # create odm texture
        connections += [self.tree[:] >> self.texturing['tree'],
                        self.args[:] >> self.texturing['args'],
                        self.meshing['reconstruction'] >> self.texturing['reconstruction']]

        return connections

    def sm_connections(self, p):
        connections = []

        # Setup
        connections += [self.tree[:] >> self.setup['tree'],
                        self.args[:] >> self.setup['args']]

        # matching
        connections += [self.tree[:] >> self.matching['tree'],
                        self.args[:] >> self.matching['args'],
                        self.setup['sm_meta'] >> self.matching['sm_meta']]

        # split
        connections += [self.tree[:] >> self.split['tree'],
                        self.args[:] >> self.split['args'],
                        self.matching['sm_meta'] >> self.split['sm_meta']]

        # reconstruction
        connections += [self.tree[:] >> self.reconstruction['tree'],
                        self.args[:] >> self.reconstruction['args'],
                        self.split['sm_meta'] >> self.reconstruction['sm_meta']]

        # align
        connections += [self.tree[:] >> self.align['tree'],
                        self.args[:] >> self.align['args'],
                        self.reconstruction['sm_meta'] >> self.align['sm_meta']]

        # dense
        connections += [self.tree[:] >> self.dense['tree'],
                        self.args[:] >> self.dense['args'],
                        self.align['sm_meta'] >> self.dense['sm_meta']]

        # merge
        connections += [self.tree[:] >> self.merge['tree'],
                        self.args[:] >> self.merge['args'],
                        self.dense['sm_meta'] >> self.merge['sm_meta']]

        return connections
