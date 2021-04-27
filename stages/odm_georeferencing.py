import os
import struct
import pipes

from opendm import io
from opendm import log
from opendm import types
from opendm import system
from opendm import context
from opendm.cropper import Cropper
from opendm import point_cloud
from opendm.multispectral import get_primary_band_name

class ODMGeoreferencingStage(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']

        if not io.file_exists(tree.odm_georeferencing_model_laz) or self.rerun():
            cmd = ('pdal translate -i "%s" -o \"%s\"' % (tree.filtered_point_cloud, tree.odm_georeferencing_model_laz))
            stages = ["ferry"]
            params = [
                '--filters.ferry.dimensions="views => UserData"',
                '--writers.las.compression="lazip"',
            ]

            if reconstruction.is_georeferenced():
                log.ODM_INFO("Georeferencing point cloud")

                stages.append("transformation")
                params += [
                    '--filters.transformation.matrix="1 0 0 %s 0 1 0 %s 0 0 1 0 0 0 0 1"' % reconstruction.georef.utm_offset(),
                    '--writers.las.offset_x=%s' % reconstruction.georef.utm_east_offset,
                    '--writers.las.offset_y=%s' % reconstruction.georef.utm_north_offset,
                    '--writers.las.offset_z=0',
                    '--writers.las.a_srs="%s"' % reconstruction.georef.proj4()
                ]
                
                system.run(cmd + ' ' + ' '.join(stages) + ' ' + ' '.join(params))

                self.update_progress(50)

                if args.crop > 0:
                    log.ODM_INFO("Calculating cropping area and generating bounds shapefile from point cloud")
                    cropper = Cropper(tree.odm_georeferencing, 'odm_georeferenced_model')
                    
                    if args.fast_orthophoto:
                        decimation_step = 10
                    else:
                        decimation_step = 40
                    
                    # More aggressive decimation for large datasets
                    if not args.fast_orthophoto:
                        decimation_step *= int(len(reconstruction.photos) / 1000) + 1
                        decimation_step = min(decimation_step, 95)
                        
                    try:
                        cropper.create_bounds_gpkg(tree.odm_georeferencing_model_laz, args.crop, 
                                                    decimation_step=decimation_step)
                    except:
                        log.ODM_WARNING("Cannot calculate crop bounds! We will skip cropping")
                        args.crop = 0
            else:
                log.ODM_INFO("Converting point cloud (non-georeferenced)")
                system.run(cmd + ' ' + ' '.join(stages) + ' ' + ' '.join(params))
        
            point_cloud.post_point_cloud_steps(args, tree, self.rerun())
        else:
            log.ODM_WARNING('Found a valid georeferenced model in: %s'
                            % tree.odm_georeferencing_model_laz)

        if args.optimize_disk_space and io.file_exists(tree.odm_georeferencing_model_laz) and io.file_exists(tree.filtered_point_cloud):
            os.remove(tree.filtered_point_cloud)
        

