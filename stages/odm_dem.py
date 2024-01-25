import os, json, math
from shutil import copyfile

from opendm import io
from opendm import log
from opendm import system
from opendm import context
from opendm import types
from opendm import gsd
from opendm.dem import commands, utils
from opendm.cropper import Cropper
from opendm import pseudogeo
from opendm.tiles.tiler import generate_dem_tiles
from opendm.cogeo import convert_to_cogeo

class ODMDEMStage(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']

        dem_input = tree.odm_georeferencing_model_laz
        pc_model_found = io.file_exists(dem_input)
        ignore_resolution = False
        pseudo_georeference = False
        
        if not reconstruction.is_georeferenced():
            log.ODM_WARNING("Not georeferenced, using ungeoreferenced point cloud...")
            ignore_resolution = True
            pseudo_georeference = True

        resolution = gsd.cap_resolution(args.dem_resolution, tree.opensfm_reconstruction, 
                                        gsd_scaling=1.0,
                                        ignore_gsd=args.ignore_gsd,
                                        ignore_resolution=ignore_resolution and args.ignore_gsd,
                                        has_gcp=reconstruction.has_gcp())

        log.ODM_INFO('Create DSM: ' + str(args.dsm))
        log.ODM_INFO('Create DTM: ' + str(args.dtm))
        log.ODM_INFO('DEM input file {0} found: {1}'.format(dem_input, str(pc_model_found)))

        # define paths and create working directories
        odm_dem_root = tree.path('odm_dem')
        if not io.dir_exists(odm_dem_root):
            system.mkdir_p(odm_dem_root)

        progress = 20
        self.update_progress(progress)

        # Do we need to process anything here?
        if (args.dsm or args.dtm) and pc_model_found:
            dsm_output_filename = os.path.join(odm_dem_root, 'dsm.tif')
            dtm_output_filename = os.path.join(odm_dem_root, 'dtm.tif')

            if (args.dtm and not io.file_exists(dtm_output_filename)) or \
                (args.dsm and not io.file_exists(dsm_output_filename)) or \
                self.rerun():

                products = []

                if args.dsm or (args.dtm and args.dem_euclidean_map): products.append('dsm')
                if args.dtm: products.append('dtm')

                radius_steps = commands.get_dem_radius_steps(tree.filtered_point_cloud_stats, args.dem_gapfill_steps, resolution)

                for product in products:
                    commands.create_dem(
                            dem_input,
                            product,
                            output_type='idw' if product == 'dtm' else 'max',
                            radiuses=list(map(str, radius_steps)),
                            gapfill=args.dem_gapfill_steps > 0,
                            outdir=odm_dem_root,
                            resolution=resolution / 100.0,
                            decimation=args.dem_decimation,
                            max_workers=args.max_concurrency,
                            with_euclidean_map=args.dem_euclidean_map,
                            max_tiles=None if reconstruction.has_geotagged_photos() else math.ceil(len(reconstruction.photos) / 2)
                        )

                    dem_geotiff_path = os.path.join(odm_dem_root, "{}.tif".format(product))
                    bounds_file_path = os.path.join(tree.odm_georeferencing, 'odm_georeferenced_model.bounds.gpkg')

                    if args.crop > 0 or args.boundary:
                        # Crop DEM
                        Cropper.crop(bounds_file_path, dem_geotiff_path, utils.get_dem_vars(args), keep_original=not args.optimize_disk_space)

                    if pseudo_georeference:
                        pseudogeo.add_pseudo_georeferencing(dem_geotiff_path)

                    if args.tiles:
                        generate_dem_tiles(dem_geotiff_path, tree.path("%s_tiles" % product), args.max_concurrency, resolution)
                    
                    if args.cog:
                        convert_to_cogeo(dem_geotiff_path, max_workers=args.max_concurrency)

                    progress += 40
                    self.update_progress(progress)
            else:
                log.ODM_WARNING('Found existing outputs in: %s' % odm_dem_root)
        else:
            log.ODM_WARNING('DEM will not be generated')
