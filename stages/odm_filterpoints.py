import os

from opendm import log
from opendm import io
from opendm import system
from opendm import context
from opendm import point_cloud
from opendm import types

class ODMFilterPoints(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']

        if not os.path.exists(tree.odm_filterpoints): system.mkdir_p(tree.odm_filterpoints)

        # check if reconstruction was done before
        if not io.file_exists(tree.filtered_point_cloud) or self.rerun():
            if args.fast_orthophoto:
                inputPointCloud = os.path.join(tree.opensfm, 'reconstruction.ply')
            else:
                inputPointCloud = tree.openmvs_model

            point_cloud.filter(inputPointCloud, tree.filtered_point_cloud, 
                                standard_deviation=args.pc_filter, 
                                sample_radius=args.pc_sample,
                                verbose=args.verbose,
                                max_concurrency=args.max_concurrency)
            
        else:
            log.ODM_WARNING('Found a valid point cloud file in: %s' %
                            tree.filtered_point_cloud)
        
        if args.optimize_disk_space:
            os.remove(inputPointCloud)
