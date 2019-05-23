import os, json
from shutil import copyfile

from opendm import io
from opendm import log
from opendm import system
from opendm import context
from opendm import types
from opendm import gsd
from opendm.dem import commands, utils
from opendm.cropper import Cropper

class ODMDEMStage(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        las_model_found = io.file_exists(tree.odm_georeferencing_model_laz)

        log.ODM_INFO('Classify: ' + str(args.pc_classify))
        log.ODM_INFO('Create DSM: ' + str(args.dsm))
        log.ODM_INFO('Create DTM: ' + str(args.dtm))
        log.ODM_INFO('DEM input file {0} found: {1}'.format(tree.odm_georeferencing_model_laz, str(las_model_found)))

        # define paths and create working directories
        odm_dem_root = tree.path('odm_dem')
        if not io.dir_exists(odm_dem_root):
            system.mkdir_p(odm_dem_root)

        if args.pc_classify and las_model_found:
            pc_classify_marker = os.path.join(odm_dem_root, 'pc_classify_done.txt')

            if not io.file_exists(pc_classify_marker) or self.rerun():
                log.ODM_INFO("Classifying {} using Simple Morphological Filter".format(tree.odm_georeferencing_model_laz))
                commands.classify(tree.odm_georeferencing_model_laz,
                                  args.smrf_scalar, 
                                  args.smrf_slope, 
                                  args.smrf_threshold, 
                                  args.smrf_window,
                                  verbose=args.verbose
                                )

                with open(pc_classify_marker, 'w') as f:
                    f.write('Classify: smrf\n')
                    f.write('Scalar: {}\n'.format(args.smrf_scalar))
                    f.write('Slope: {}\n'.format(args.smrf_slope))
                    f.write('Threshold: {}\n'.format(args.smrf_threshold))
                    f.write('Window: {}\n'.format(args.smrf_window))
            
        progress = 20
        self.update_progress(progress)

        # Do we need to process anything here?
        if (args.dsm or args.dtm) and las_model_found:
            dsm_output_filename = os.path.join(odm_dem_root, 'dsm.tif')
            dtm_output_filename = os.path.join(odm_dem_root, 'dtm.tif')

            if (args.dtm and not io.file_exists(dtm_output_filename)) or \
                (args.dsm and not io.file_exists(dsm_output_filename)) or \
                self.rerun():

                products = []
                if args.dsm: products.append('dsm')
                if args.dtm: products.append('dtm')
                
                resolution = gsd.cap_resolution(args.dem_resolution, tree.opensfm_reconstruction, gsd_error_estimate=-3, ignore_gsd=args.ignore_gsd)
                radius_steps = [(resolution / 100.0) / 2.0]
                for _ in range(args.dem_gapfill_steps - 1):
                    radius_steps.append(radius_steps[-1] * 2) # 2 is arbitrary, maybe there's a better value?

                for product in products:
                    commands.create_dem(
                            tree.odm_georeferencing_model_laz,
                            product,
                            output_type='idw' if product == 'dtm' else 'max',
                            radiuses=map(str, radius_steps),
                            gapfill=args.dem_gapfill_steps > 0,
                            outdir=odm_dem_root,
                            resolution=resolution / 100.0,
                            decimation=args.dem_decimation,
                            verbose=args.verbose,
                            max_workers=args.max_concurrency,
                            keep_unfilled_copy=args.dem_euclidean_map
                        )

                    dem_geotiff_path = os.path.join(odm_dem_root, "{}.tif".format(product))
                    bounds_file_path = os.path.join(tree.odm_georeferencing, 'odm_georeferenced_model.bounds.gpkg')

                    if args.crop > 0:
                        # Crop DEM
                        Cropper.crop(bounds_file_path, dem_geotiff_path, utils.get_dem_vars(args))

                    if args.dem_euclidean_map:
                        unfilled_dem_path = io.related_file_path(dem_geotiff_path, postfix=".unfilled")
                        
                        if args.crop > 0:
                            # Crop unfilled DEM
                            Cropper.crop(bounds_file_path, unfilled_dem_path, utils.get_dem_vars(args))

                        commands.compute_euclidean_map(unfilled_dem_path, 
                                            io.related_file_path(dem_geotiff_path, postfix=".euclideand"), 
                                            overwrite=True)
                    
                    progress += 30
                    self.update_progress(progress)
            else:
                log.ODM_WARNING('Found existing outputs in: %s' % odm_dem_root)
        else:
            log.ODM_WARNING('DEM will not be generated')
