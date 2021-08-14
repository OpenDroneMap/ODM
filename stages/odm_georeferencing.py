import os
import struct
import pipes
import fiona
import fiona.crs
from collections import OrderedDict

from opendm import io
from opendm import log
from opendm import types
from opendm import system
from opendm import context
from opendm.cropper import Cropper
from opendm import point_cloud
from opendm.multispectral import get_primary_band_name
from opendm.osfm import OSFMContext

class ODMGeoreferencingStage(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']

        # Export GCP information if available

        gcp_export_file = tree.path("odm_georeferencing", "ground_control_points.gpkg")
        gcp_gml_export_file = tree.path("odm_georeferencing", "ground_control_points.gml")

        if reconstruction.has_gcp() and (not io.file_exists(gcp_export_file) or self.rerun()):
            octx = OSFMContext(tree.opensfm)
            gcps = octx.ground_control_points(reconstruction.georef.proj4())

            if len(gcps):
                gcp_schema = {
                    'geometry': 'Point',
                    'properties': OrderedDict([
                        ('id', 'str'),
                        ('observations_count', 'int'),
                        ('observations_list', 'str'),
                        ('triangulated_x', 'float'),
                        ('triangulated_y', 'float'),
                        ('triangulated_z', 'float'),
                        ('error_x', 'float'),
                        ('error_y', 'float'),
                        ('error_z', 'float'),
                    ])
                }

                # Write GeoPackage
                with fiona.open(gcp_export_file, 'w', driver="GPKG", 
                                crs=fiona.crs.from_string(reconstruction.georef.proj4()),
                                schema=gcp_schema) as f:
                    for gcp in gcps:
                        f.write({
                            'geometry': {
                                'type': 'Point',
                                'coordinates': gcp['coordinates'],
                            },
                            'properties': OrderedDict([
                                ('id', gcp['id']),
                                ('observations_count', len(gcp['observations'])),
                                ('observations_list', ",".join(gcp['observations'])),
                                ('triangulated_x', gcp['triangulated'][0]),
                                ('triangulated_y', gcp['triangulated'][1]),
                                ('triangulated_z', gcp['triangulated'][2]),
                                ('error_x', gcp['error'][0]),
                                ('error_y', gcp['error'][1]),
                                ('error_z', gcp['error'][2]),
                            ])
                        })
                
                # Write GML
                try:
                    system.run('ogr2ogr -of GML "{}" "{}"'.format(gcp_gml_export_file, gcp_export_file))
                except Exception as e:
                    log.ODM_WARNING("Cannot generate ground control points GML file: %s" % str(e))
            else:
                log.ODM_WARNING("GCPs could not be loaded for writing to %s" % gcp_export_file)


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

                if reconstruction.has_gcp() and io.file_exists(gcp_gml_export_file):
                    log.ODM_INFO("Embedding GCP info in point cloud")
                    params += [
                        '--writers.las.vlrs="{\\\"filename\\\": \\\"%s\\\", \\\"user_id\\\": \\\"ODM_GCP\\\", \\\"description\\\": \\\"Ground Control Points (GML)\\\"}"' % gcp_gml_export_file
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
        

