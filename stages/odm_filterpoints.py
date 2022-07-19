import os

from opendm import log
from opendm import io
from opendm import system
from opendm import context
from opendm import point_cloud
from opendm import types
from opendm import gsd
from opendm.boundary import boundary_offset, compute_boundary_from_shots

class ODMFilterPoints(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']

        if not os.path.exists(tree.odm_filterpoints): system.mkdir_p(tree.odm_filterpoints)

        inputPointCloud = ""
        
        # check if reconstruction was done before
        if not io.file_exists(tree.filtered_point_cloud) or self.rerun():
            if args.fast_orthophoto:
                inputPointCloud = os.path.join(tree.opensfm, 'reconstruction.ply')
            else:
                inputPointCloud = tree.openmvs_model

            # Check if we need to compute boundary
            if args.auto_boundary:
                if reconstruction.is_georeferenced():
                    if not 'boundary' in outputs:
                        boundary_distance = None

                        if args.auto_boundary_distance > 0:
                            boundary_distance = args.auto_boundary_distance
                        else:
                            avg_gsd = gsd.opensfm_reconstruction_average_gsd(tree.opensfm_reconstruction)
                            if avg_gsd is not None:
                                boundary_distance = avg_gsd * 20 # 20 is arbitrary
                            
                        if boundary_distance is not None:
                            outputs['boundary'] = compute_boundary_from_shots(tree.opensfm_reconstruction, boundary_distance, reconstruction.get_proj_offset())
                            if outputs['boundary'] is None:
                                log.ODM_WARNING("Cannot compute boundary from camera shots")
                        else:
                            log.ODM_WARNING("Cannot compute boundary (GSD cannot be estimated)")
                    else:
                        log.ODM_WARNING("--auto-boundary set but so is --boundary, will use --boundary")
                else:
                    log.ODM_WARNING("Not a georeferenced reconstruction, will ignore --auto-boundary")
                    
            point_cloud.filter(inputPointCloud, tree.filtered_point_cloud, 
                                standard_deviation=args.pc_filter, 
                                sample_radius=args.pc_sample,
                                boundary=boundary_offset(outputs.get('boundary'), reconstruction.get_proj_offset()),
                                verbose=args.verbose,
                                max_concurrency=args.max_concurrency)
            
            # Quick check
            info = point_cloud.ply_info(tree.filtered_point_cloud)
            if info["vertex_count"] == 0:
                extra_msg = ''
                if 'boundary' in outputs:
                    extra_msg = '. Also, since you used a boundary setting, make sure that the boundary polygon you specified covers the reconstruction area correctly.'
                raise system.ExitException("Uh oh! We ended up with an empty point cloud. This means that the reconstruction did not succeed. Have you followed best practices for data acquisition? See https://docs.opendronemap.org/flying/%s" % extra_msg)
        else:
            log.ODM_WARNING('Found a valid point cloud file in: %s' %
                            tree.filtered_point_cloud)
        
        if args.optimize_disk_space and inputPointCloud:
            if os.path.isfile(inputPointCloud):
                os.remove(inputPointCloud)
