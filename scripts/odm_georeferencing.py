import ecto
import os
import struct
import pipes

from opendm import io
from opendm import log
from opendm import types
from opendm import system
from opendm import context
from opendm.cropper import Cropper


class ODMGeoreferencingCell(ecto.Cell):
    def declare_params(self, params):
        params.declare("gcp_file", 'path to the file containing the ground control '
                            'points used for georeferencing.The file needs to '
                            'be on the following line format: \neasting '
                            'northing height pixelrow pixelcol imagename', 'gcp_list.txt')
        params.declare("use_exif", 'use exif', False)
        params.declare("verbose", 'print additional messages to console', False)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("reconstruction", "list of ODMReconstructions", [])
        outputs.declare("reconstruction", "list of ODMReconstructions", [])

    def process(self, inputs, outputs):

        # Benchmarking
        start_time = system.now_raw()

        log.ODM_INFO('Running ODM Georeferencing Cell')

        # get inputs
        args = inputs.args
        tree = inputs.tree
        reconstruction = inputs.reconstruction
        gcpfile = tree.odm_georeferencing_gcp
        doPointCloudGeo = True
        transformPointCloud = True
        verbose = '-verbose' if self.params.verbose else ''
        geo_ref = reconstruction.georef

        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'odm_georeferencing') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'odm_georeferencing' in args.rerun_from)

        runs = [{
            'georeferencing_dir': tree.odm_georeferencing,
            'texturing_dir': tree.odm_texturing,
            'model': os.path.join(tree.odm_texturing, tree.odm_textured_model_obj)
        }]

        if args.skip_3dmodel:
            runs = []

        if not args.use_3dmesh:
            # Make sure 2.5D mesh is georeferenced before the 3D mesh
            # Because it will be used to calculate a transform
            # for the point cloud. If we use the 3D model transform,
            # DEMs and orthophoto might not align!
            runs.insert(0, {
                    'georeferencing_dir': tree.odm_25dgeoreferencing,
                    'texturing_dir': tree.odm_25dtexturing,
                    'model': os.path.join(tree.odm_25dtexturing, tree.odm_textured_model_obj)
                })

        for r in runs:
            odm_georeferencing_model_obj_geo = os.path.join(r['texturing_dir'], tree.odm_georeferencing_model_obj_geo)
            odm_georeferencing_log = os.path.join(r['georeferencing_dir'], tree.odm_georeferencing_log)
            odm_georeferencing_transform_file = os.path.join(r['georeferencing_dir'], tree.odm_georeferencing_transform_file)
            odm_georeferencing_model_txt_geo_file = os.path.join(r['georeferencing_dir'], tree.odm_georeferencing_model_txt_geo)

            if not io.file_exists(odm_georeferencing_model_obj_geo) or \
               not io.file_exists(tree.odm_georeferencing_model_laz) or rerun_cell:

                # odm_georeference definitions
                kwargs = {
                    'bin': context.odm_modules_path,
                    'bundle': tree.opensfm_bundle,
                    'imgs': tree.dataset_raw,
                    'imgs_list': tree.opensfm_bundle_list,
                    'model': r['model'],
                    'log': odm_georeferencing_log,
                    'input_trans_file': tree.opensfm_transformation,
                    'transform_file': odm_georeferencing_transform_file,
                    'coords': tree.odm_georeferencing_coords,
                    'output_pc_file': tree.odm_georeferencing_model_laz,
                    'geo_sys': odm_georeferencing_model_txt_geo_file,
                    'model_geo': odm_georeferencing_model_obj_geo,
                    'gcp': gcpfile,
                    'verbose': verbose
                }

                if args.fast_orthophoto:
                    kwargs['input_pc_file'] = os.path.join(tree.opensfm, 'reconstruction.ply')
                elif args.use_opensfm_dense:
                    kwargs['input_pc_file'] = tree.opensfm_model
                else:
                    kwargs['input_pc_file'] = tree.smvs_model

                if transformPointCloud:
                    kwargs['pc_params'] = '-inputPointCloudFile {input_pc_file} -outputPointCloudFile {output_pc_file}'.format(**kwargs)

                    if geo_ref and geo_ref.projection and geo_ref.projection.srs:
                        kwargs['pc_params'] += ' -outputPointCloudSrs %s' % pipes.quote(geo_ref.projection.srs)
                    else:
                        log.ODM_WARNING('NO SRS: The output point cloud will not have a SRS.')
                else:
                    kwargs['pc_params'] = ''
                    
                # Check to see if the GCP file exists

                if not self.params.use_exif and (self.params.gcp_file or tree.odm_georeferencing_gcp):
                   log.ODM_INFO('Found %s' % gcpfile)
                   try:
                       system.run('{bin}/odm_georef -bundleFile {bundle} -imagesPath {imgs} -imagesListPath {imgs_list} '
                                  '-inputFile {model} -outputFile {model_geo} '
                                  '{pc_params} {verbose} '
                                  '-logFile {log} -outputTransformFile {transform_file} -georefFileOutputPath {geo_sys} -gcpFile {gcp} '
                                  '-outputCoordFile {coords}'.format(**kwargs))
                   except Exception:
                       log.ODM_EXCEPTION('Georeferencing failed. ')
                       return ecto.QUIT
                elif io.file_exists(tree.opensfm_transformation) and io.file_exists(tree.odm_georeferencing_coords):
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
                    # update images metadata
                    geo_ref.extract_offsets(odm_georeferencing_model_txt_geo_file)
                    reconstruction.georef = geo_ref

                    # XYZ point cloud output
                    if args.pc_csv:
                        log.ODM_INFO("Creating geo-referenced CSV file (XYZ format)")
                        
                        system.run("pdal translate -i \"{}\" "
                            "-o \"{}\" "
                            "--writers.text.format=csv "
                            "--writers.text.order=\"X,Y,Z\" "
                            "--writers.text.keep_unspecified=false ".format(
                                tree.odm_georeferencing_model_laz,
                                tree.odm_georeferencing_xyz_file))
                    
                    # LAS point cloud output
                    if args.pc_las:
                        log.ODM_INFO("Creating geo-referenced LAS file")
                        
                        system.run("pdal translate -i \"{}\" "
                            "-o \"{}\" ".format(
                                tree.odm_georeferencing_model_laz,
                                tree.odm_georeferencing_model_las))
                    
                    if args.crop > 0:
                        log.ODM_INFO("Calculating cropping area and generating bounds shapefile from point cloud")
                        cropper = Cropper(tree.odm_georeferencing, 'odm_georeferenced_model')
                        
                        decimation_step = 40 if args.fast_orthophoto or args.use_opensfm_dense else 90
                        
                        # More aggressive decimation for large datasets
                        if not args.fast_orthophoto:
                            decimation_step *= int(len(reconstruction.photos) / 1000) + 1

                        cropper.create_bounds_shapefile(tree.odm_georeferencing_model_laz, args.crop, 
                                                    decimation_step=decimation_step,
                                                    outlier_radius=20 if args.fast_orthophoto else 2)

                    # Do not execute a second time, since
                    # We might be doing georeferencing for
                    # multiple models (3D, 2.5D, ...)
                    doPointCloudGeo = False
                    transformPointCloud = False
            else:
                log.ODM_WARNING('Found a valid georeferenced model in: %s'
                                % tree.odm_georeferencing_model_laz)

        outputs.reconstruction = reconstruction

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'Georeferencing')

        log.ODM_INFO('Running ODM Georeferencing Cell - Finished')
        return ecto.OK if args.end_with != 'odm_georeferencing' else ecto.QUIT
