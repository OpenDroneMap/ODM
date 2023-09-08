import os
import shutil
import struct
import pipes
import fiona
import fiona.crs
import json
import zipfile
from collections import OrderedDict
from pyproj import CRS

from opendm import io
from opendm import log
from opendm import types
from opendm import system
from opendm import context
from opendm import location
from opendm.cropper import Cropper
from opendm import point_cloud
from opendm.multispectral import get_primary_band_name
from opendm.osfm import OSFMContext
from opendm.boundary import as_polygon, export_to_bounds_files
from opendm.align import compute_alignment_matrix, transform_point_cloud, transform_obj
from opendm.utils import np_to_json

class ODMGeoreferencingStage(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']

        # Export GCP information if available

        gcp_export_file = tree.path("odm_georeferencing", "ground_control_points.gpkg")
        gcp_gml_export_file = tree.path("odm_georeferencing", "ground_control_points.gml")
        gcp_geojson_export_file = tree.path("odm_georeferencing", "ground_control_points.geojson")
        gcp_geojson_zip_export_file = tree.path("odm_georeferencing", "ground_control_points.zip")
        unaligned_model = io.related_file_path(tree.odm_georeferencing_model_laz, postfix="_unaligned")
        if os.path.isfile(unaligned_model) and self.rerun():
            os.unlink(unaligned_model)

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
                                ('observations_list', ",".join([obs['shot_id'] for obs in gcp['observations']])),
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

                # Write GeoJSON
                geojson = {
                    'type': 'FeatureCollection',
                    'features': []
                }

                from_srs = CRS.from_proj4(reconstruction.georef.proj4())
                to_srs = CRS.from_epsg(4326)
                transformer = location.transformer(from_srs, to_srs)

                for gcp in gcps:
                    properties = gcp.copy()
                    del properties['coordinates']

                    geojson['features'].append({
                        'type': 'Feature',
                        'geometry': {
                            'type': 'Point',
                            'coordinates': transformer.TransformPoint(*gcp['coordinates']),
                        },
                        'properties': properties
                    })

                with open(gcp_geojson_export_file, 'w') as f:
                    f.write(json.dumps(geojson, indent=4))
                
                with zipfile.ZipFile(gcp_geojson_zip_export_file, 'w', compression=zipfile.ZIP_LZMA) as f:
                    f.write(gcp_geojson_export_file, arcname=os.path.basename(gcp_geojson_export_file))

            else:
                log.ODM_WARNING("GCPs could not be loaded for writing to %s" % gcp_export_file)

        if not io.file_exists(tree.odm_georeferencing_model_laz) or self.rerun():
            cmd = f'pdal translate -i "{tree.filtered_point_cloud}" -o \"{tree.odm_georeferencing_model_laz}\"'
            stages = ["ferry"]
            params = [
                '--filters.ferry.dimensions="views => UserData"'
            ]

            if reconstruction.is_georeferenced():
                log.ODM_INFO("Georeferencing point cloud")

                stages.append("transformation")
                utmoffset = reconstruction.georef.utm_offset()
                params += [
                    f'--filters.transformation.matrix="1 0 0 {utmoffset[0]} 0 1 0 {utmoffset[1]} 0 0 1 0 0 0 0 1"',
                    f'--writers.las.offset_x={reconstruction.georef.utm_east_offset}' ,
                    f'--writers.las.offset_y={reconstruction.georef.utm_north_offset}',
                    '--writers.las.scale_x=0.001',
                    '--writers.las.scale_y=0.001',
                    '--writers.las.scale_z=0.001',
                    '--writers.las.offset_z=0',
                    f'--writers.las.a_srs="{reconstruction.georef.proj4()}"' # HOBU this should maybe be WKT
                ]

                if reconstruction.has_gcp() and io.file_exists(gcp_geojson_zip_export_file):
                    if os.path.getsize(gcp_geojson_zip_export_file) <= 65535:
                        log.ODM_INFO("Embedding GCP info in point cloud")
                        params += [
                            '--writers.las.vlrs="{\\\"filename\\\": \\\"%s\\\", \\\"user_id\\\": \\\"ODM\\\", \\\"record_id\\\": 2, \\\"description\\\": \\\"Ground Control Points (zip)\\\"}"' % gcp_geojson_zip_export_file.replace(os.sep, "/")
                        ]
                    else:
                        log.ODM_WARNING("Cannot embed GCP info in point cloud, %s is too large" % gcp_geojson_zip_export_file)

                system.run(cmd + ' ' + ' '.join(stages) + ' ' + ' '.join(params))

                self.update_progress(50)

                if args.crop > 0:
                    log.ODM_INFO("Calculating cropping area and generating bounds shapefile from point cloud")
                    cropper = Cropper(tree.odm_georeferencing, 'odm_georeferenced_model')

                    if args.fast_orthophoto:
                        decimation_step = 4
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

                if 'boundary' in outputs and args.crop == 0:
                    log.ODM_INFO("Using boundary JSON as cropping area")

                    bounds_base, _ = os.path.splitext(tree.odm_georeferencing_model_laz)
                    bounds_json = bounds_base + ".bounds.geojson"
                    bounds_gpkg = bounds_base + ".bounds.gpkg"
                    export_to_bounds_files(outputs['boundary'], reconstruction.get_proj_srs(), bounds_json, bounds_gpkg)
            else:
                log.ODM_INFO("Converting point cloud (non-georeferenced)")
                system.run(cmd + ' ' + ' '.join(stages) + ' ' + ' '.join(params))


            stats_dir = tree.path("opensfm", "stats", "codem")
            if os.path.exists(stats_dir) and self.rerun():
                shutil.rmtree(stats_dir)

            if tree.odm_align_file is not None:
                alignment_file_exists = io.file_exists(tree.odm_georeferencing_alignment_matrix)

                if not alignment_file_exists or self.rerun():
                    if alignment_file_exists:
                        os.unlink(tree.odm_georeferencing_alignment_matrix)

                    a_matrix = None
                    try:
                        a_matrix = compute_alignment_matrix(tree.odm_georeferencing_model_laz, tree.odm_align_file, stats_dir)
                    except Exception as e:
                        log.ODM_WARNING("Cannot compute alignment matrix: %s" % str(e))

                    if a_matrix is not None:
                        log.ODM_INFO("Alignment matrix: %s" % a_matrix)

                        # Align point cloud
                        if os.path.isfile(unaligned_model):
                            os.rename(unaligned_model, tree.odm_georeferencing_model_laz)
                        os.rename(tree.odm_georeferencing_model_laz, unaligned_model)

                        try:
                            transform_point_cloud(unaligned_model, a_matrix, tree.odm_georeferencing_model_laz)
                            log.ODM_INFO("Transformed %s" % tree.odm_georeferencing_model_laz)
                        except Exception as e:
                            log.ODM_WARNING("Cannot transform point cloud: %s" % str(e))
                            os.rename(unaligned_model, tree.odm_georeferencing_model_laz)

                        # Align textured models
                        def transform_textured_model(obj):
                            if os.path.isfile(obj):
                                unaligned_obj = io.related_file_path(obj, postfix="_unaligned")
                                if os.path.isfile(unaligned_obj):
                                    os.rename(unaligned_obj, obj)
                                os.rename(obj, unaligned_obj)
                                try:
                                    transform_obj(unaligned_obj, a_matrix, [reconstruction.georef.utm_east_offset, reconstruction.georef.utm_north_offset], obj)
                                    log.ODM_INFO("Transformed %s" % obj)
                                except Exception as e:
                                    log.ODM_WARNING("Cannot transform textured model: %s" % str(e))
                                    os.rename(unaligned_obj, obj)

                        for texturing in [tree.odm_texturing, tree.odm_25dtexturing]:
                            if reconstruction.multi_camera:
                                primary = get_primary_band_name(reconstruction.multi_camera, args.primary_band)
                                for band in reconstruction.multi_camera:
                                    subdir = "" if band['name'] == primary else band['name'].lower()
                                    obj = os.path.join(texturing, subdir, "odm_textured_model_geo.obj")
                                    transform_textured_model(obj)
                            else:
                                obj = os.path.join(texturing, "odm_textured_model_geo.obj")
                                transform_textured_model(obj)

                        with open(tree.odm_georeferencing_alignment_matrix, "w") as f:
                            f.write(np_to_json(a_matrix))
                    else:
                        log.ODM_WARNING("Alignment to %s will be skipped." % tree.odm_align_file)
                else:
                    log.ODM_WARNING("Already computed alignment")
            elif io.file_exists(tree.odm_georeferencing_alignment_matrix):
                os.unlink(tree.odm_georeferencing_alignment_matrix)

            point_cloud.post_point_cloud_steps(args, tree, self.rerun())
        else:
            log.ODM_WARNING('Found a valid georeferenced model in: %s'
                            % tree.odm_georeferencing_model_laz)

        if args.optimize_disk_space and io.file_exists(tree.odm_georeferencing_model_laz) and io.file_exists(tree.filtered_point_cloud):
            os.remove(tree.filtered_point_cloud)


