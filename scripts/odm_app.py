import os

from opendm import context
from opendm import types
from opendm import io
from opendm import system
from opendm import log

from dataset import ODMLoadDatasetStage
from run_opensfm import ODMOpenSfMStage
from mve import ODMMveStage
from odm_slam import ODMSlamStage
from odm_meshing import ODMeshingStage
from mvstex import ODMMvsTexStage
from odm_georeferencing import ODMGeoreferencingStage
from odm_orthophoto import ODMOrthoPhotoStage
from odm_dem import ODMDEMStage
from odm_filterpoints import ODMFilterPoints
from splitmerge import ODMSplitStage, ODMMergeStage


class ODMApp:
    def __init__(self, args):
        """
        Initializes the application and defines the ODM application pipeline stages
        """
        
        dataset = ODMLoadDatasetStage('dataset', args, 
                                          verbose=args.verbose,
                                          proj=args.proj)
        split = ODMSplitStage('split', args)
        merge = ODMMergeStage('merge', args)
        opensfm = ODMOpenSfMStage('opensfm', args)
        slam = ODMSlamStage('slam', args)
        mve = ODMMveStage('mve', args)
        filterpoints = ODMFilterPoints('odm_filterpoints', args)
        meshing = ODMeshingStage('odm_meshing', args,
                                    max_vertex=args.mesh_size,
                                    oct_tree=args.mesh_octree_depth,
                                    samples=args.mesh_samples,
                                    point_weight=args.mesh_point_weight,
                                    max_concurrency=args.max_concurrency,
                                    verbose=args.verbose)
        texturing = ODMMvsTexStage('mvs_texturing', args,
                                    data_term=args.texturing_data_term,
                                    outlier_rem_type=args.texturing_outlier_removal_type,
                                    skip_vis_test=args.texturing_skip_visibility_test,
                                    skip_glob_seam_leveling=args.texturing_skip_global_seam_leveling,
                                    skip_loc_seam_leveling=args.texturing_skip_local_seam_leveling,
                                    skip_hole_fill=args.texturing_skip_hole_filling,
                                    keep_unseen_faces=args.texturing_keep_unseen_faces,
                                    tone_mapping=args.texturing_tone_mapping)
        georeferencing = ODMGeoreferencingStage('odm_georeferencing', args,
                                                    gcp_file=args.gcp,
                                                    use_exif=args.use_exif,
                                                    verbose=args.verbose)
        dem = ODMDEMStage('odm_dem', args,
                            max_concurrency=args.max_concurrency,
                            verbose=args.verbose)
        orthophoto = ODMOrthoPhotoStage('odm_orthophoto', args,
                                            resolution=args.orthophoto_resolution,
                                            no_tiled=args.orthophoto_no_tiled,
                                            compress=args.orthophoto_compression,
                                            bigtiff=args.orthophoto_bigtiff,
                                            build_overviews=args.build_overviews,
                                            max_concurrency=args.max_concurrency,
                                            verbose=args.verbose)
        
        if not args.video:
            # Normal pipeline
            self.first_stage = dataset

            dataset.connect(split) \
                   .connect(merge) \
                   .connect(opensfm)

            if args.use_opensfm_dense or args.fast_orthophoto:
                opensfm.connect(filterpoints)
            else:
                opensfm.connect(mve) \
                       .connect(filterpoints)
            
            filterpoints \
                .connect(meshing) \
                .connect(texturing) \
                .connect(georeferencing) \
                .connect(dem) \
                .connect(orthophoto)
                
        else:
            # SLAM pipeline
            # TODO: this is broken and needs work
            log.ODM_WARNING("SLAM module is currently broken. We could use some help fixing this. If you know Python, get in touch at https://community.opendronemap.org.")
            self.first_stage = slam

            slam.connect(mve) \
                .connect(meshing) \
                .connect(texturing)

    def execute(self):
        self.first_stage.run()