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

class ODMGeoreferencingStage(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']

        doPointCloudGeo = True
        transformPointCloud = True
        verbose = '-verbose' if self.params.get('verbose') else ''

        class nonloc:
            runs = []

        def add_run(primary=True, band=None):
            subdir = ""
            if not primary and band is not None:
                subdir = band

            # Make sure 2.5D mesh is georeferenced before the 3D mesh
            # Because it will be used to calculate a transform
            # for the point cloud. If we use the 3D model transform,
            # DEMs and orthophoto might not align!
            if not args.use_3dmesh:
                nonloc.runs += [{
                    'georeferencing_dir': os.path.join(tree.odm_25dgeoreferencing, subdir),
                    'texturing_dir': os.path.join(tree.odm_25dtexturing, subdir),
                }]
            
            if not args.skip_3dmodel and (primary or args.use_3dmesh):
                nonloc.runs += [{
                    'georeferencing_dir': tree.odm_georeferencing,
                    'texturing_dir': os.path.join(tree.odm_texturing, subdir),
                }]
        
        if reconstruction.multi_camera:
            for band in reconstruction.multi_camera:
                primary = band == reconstruction.multi_camera[0]
                add_run(primary, band['name'].lower())
        else:
            add_run()

        progress_per_run = 100.0 / len(nonloc.runs)
        progress = 0.0

        for r in nonloc.runs:
            if not io.dir_exists(r['georeferencing_dir']):
                system.mkdir_p(r['georeferencing_dir'])

            odm_georeferencing_model_obj_geo = os.path.join(r['texturing_dir'], tree.odm_georeferencing_model_obj_geo)
            odm_georeferencing_model_obj = os.path.join(r['texturing_dir'], tree.odm_textured_model_obj)
            odm_georeferencing_log = os.path.join(r['georeferencing_dir'], tree.odm_georeferencing_log)
            odm_georeferencing_transform_file = os.path.join(r['georeferencing_dir'], tree.odm_georeferencing_transform_file)
            odm_georeferencing_model_txt_geo_file = os.path.join(r['georeferencing_dir'], tree.odm_georeferencing_model_txt_geo)

            if not io.file_exists(odm_georeferencing_model_obj_geo) or \
               not io.file_exists(tree.odm_georeferencing_model_laz) or self.rerun():

                # odm_georeference definitions
                kwargs = {
                    'bin': context.odm_modules_path,
                    'input_pc_file': tree.filtered_point_cloud,
                    'bundle': tree.opensfm_bundle,
                    'imgs': tree.dataset_raw,
                    'imgs_list': tree.opensfm_bundle_list,
                    'model': odm_georeferencing_model_obj,
                    'log': odm_georeferencing_log,
                    'input_trans_file': tree.opensfm_transformation,
                    'transform_file': odm_georeferencing_transform_file,
                    'coords': tree.odm_georeferencing_coords,
                    'output_pc_file': tree.odm_georeferencing_model_laz,
                    'geo_sys': odm_georeferencing_model_txt_geo_file,
                    'model_geo': odm_georeferencing_model_obj_geo,
                    'verbose': verbose
                }

                if transformPointCloud:
                    kwargs['pc_params'] = '-inputPointCloudFile {input_pc_file} -outputPointCloudFile {output_pc_file}'.format(**kwargs)

                    if reconstruction.is_georeferenced():
                        kwargs['pc_params'] += ' -outputPointCloudSrs %s' % pipes.quote(reconstruction.georef.proj4())
                    else:
                        log.ODM_WARNING('NO SRS: The output point cloud will not have a SRS.')
                else:
                    kwargs['pc_params'] = ''
 
                if io.file_exists(tree.opensfm_transformation) and io.file_exists(tree.odm_georeferencing_coords):
                    log.ODM_INFO('Running georeferencing with OpenSfM transformation matrix')
                    system.run('{bin}/odm_georef -bundleFile {bundle} -inputTransformFile {input_trans_file} -inputCoordFile {coords} '
                               '-inputFile {model} -outputFile {model_geo} '
                               '{pc_params} {verbose} '
                               '-logFile {log} -outputTransformFile {transform_file} -georefFileOutputPath {geo_sys}'.format(**kwargs))
                elif io.file_exists(tree.odm_georeferencing_coords):
                    log.ODM_INFO('Running georeferencing with generated coords file.')
                    system.run('{bin}/odm_georef -bundleFile {bundle} -inputCoordFile {coords} '
                               '-inputFile {model} -outputFile {model_geo} '
                               '{pc_params} {verbose} '
                               '-logFile {log} -outputTransformFile {transform_file} -georefFileOutputPath {geo_sys}'.format(**kwargs))
                else:
                    log.ODM_WARNING('Georeferencing failed. Make sure your '
                                    'photos have geotags in the EXIF or you have '
                                    'provided a GCP file. ')
                    doPointCloudGeo = False # skip the rest of the georeferencing

                if doPointCloudGeo:
                    reconstruction.georef.extract_offsets(odm_georeferencing_model_txt_geo_file)
                    point_cloud.post_point_cloud_steps(args, tree)
                    
                    if args.crop > 0:
                        log.ODM_INFO("Calculating cropping area and generating bounds shapefile from point cloud")
                        cropper = Cropper(tree.odm_georeferencing, 'odm_georeferenced_model')
                        
                        decimation_step = 40 if args.fast_orthophoto or args.use_opensfm_dense else 90
                        
                        # More aggressive decimation for large datasets
                        if not args.fast_orthophoto:
                            decimation_step *= int(len(reconstruction.photos) / 1000) + 1

                        cropper.create_bounds_gpkg(tree.odm_georeferencing_model_laz, args.crop, 
                                                    decimation_step=decimation_step)

                    # Do not execute a second time, since
                    # We might be doing georeferencing for
                    # multiple models (3D, 2.5D, ...)
                    doPointCloudGeo = False
                    transformPointCloud = False
            else:
                log.ODM_WARNING('Found a valid georeferenced model in: %s'
                                % tree.odm_georeferencing_model_laz)

            if args.optimize_disk_space and io.file_exists(tree.odm_georeferencing_model_laz) and io.file_exists(tree.filtered_point_cloud):
                os.remove(tree.filtered_point_cloud)
            
            progress += progress_per_run
            self.update_progress(progress)
