import ecto, os, json
from shutil import copyfile

from opendm import io
from opendm import log
from opendm import system
from opendm import context
from opendm import types


class ODMDEMCell(ecto.Cell):
    def declare_params(self, params):
        params.declare("verbose", 'print additional messages to console', False)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("reconstruction", "list of ODMReconstructions", [])

    def process(self, inputs, outputs):
        # Benchmarking
        start_time = system.now_raw()

        log.ODM_INFO('Running ODM DEM Cell')

        # get inputs
        args = self.inputs.args
        tree = self.inputs.tree
        las_model_found = io.file_exists(tree.odm_georeferencing_model_las)
        env_paths = [context.superbuild_bin_path]

        # Just to make sure
        l2d_module_installed = True
        try:
            system.run('l2d_classify --help > /dev/null', env_paths)
        except:
            log.ODM_WARNING('lidar2dems is not installed properly')
            l2d_module_installed = False

        log.ODM_INFO('Create DSM: ' + str(args.dsm))
        log.ODM_INFO('Create DTM: ' + str(args.dtm))
        log.ODM_INFO('DEM input file {0} found: {1}'.format(tree.odm_georeferencing_model_las, str(las_model_found)))

        # Do we need to process anything here?
        if (args.dsm or args.dtm) and las_model_found and l2d_module_installed:

            # define paths and create working directories
            odm_dem_root = tree.path('odm_dem')
            system.mkdir_p(odm_dem_root)

            dsm_output_filename = os.path.join(odm_dem_root, 'dsm.tif')
            dtm_output_filename = os.path.join(odm_dem_root, 'dtm.tif')

            # check if we rerun cell or not
            rerun_cell = (args.rerun is not None and
                          args.rerun == 'odm_dem') or \
                         (args.rerun_all) or \
                         (args.rerun_from is not None and
                          'odm_dem' in args.rerun_from)

            if (args.dtm and not io.file_exists(dtm_output_filename)) or \
                (args.dsm and not io.file_exists(dsm_output_filename)) or \
                rerun_cell:
                 
                # Process with lidar2dems
                terrain_params_map = {
                    'flatnonforest': (1, 3), 
                    'flatforest': (1, 2), 
                    'complexnonforest': (5, 2), 
                    'complexforest': (10, 2)
                }
                terrain_params = terrain_params_map[args.dem_terrain_type.lower()]             

                kwargs = {
                    'verbose': '-v' if self.params.verbose else '',
                    'slope': terrain_params[0],
                    'cellsize': terrain_params[1],
                    'outdir': odm_dem_root,
                    'site': ''
                }

                if args.crop > 0:
                    bounds_shapefile_path = os.path.join(tree.odm_georeferencing, 'odm_georeferenced_model.bounds.shp')
                    if os.path.exists(bounds_shapefile_path):
                        kwargs['site'] = '-s {}'.format(bounds_shapefile_path)

                l2d_params = '--slope {slope} --cellsize {cellsize} ' \
                             '{verbose} ' \
                             '-o {site} ' \
                             '--outdir {outdir}'.format(**kwargs)

                approximate = '--approximate' if args.dem_approximate else ''

                # Classify only if we need a DTM
                run_classification = args.dtm

                if run_classification:
                    system.run('l2d_classify {0} --decimation {1} '
                               '{2} --initialDistance {3} {4}'.format(
                        l2d_params, args.dem_decimation, approximate, 
                        args.dem_initial_distance, tree.odm_georeferencing), env_paths)
                else:
                    log.ODM_INFO("Will skip classification, only DSM is needed")
                    l2d_classified_pattern = 'odm_georeferenced_model.bounds-0_l2d_s{slope}c{cellsize}.las' if args.crop > 0 else 'l2d_s{slope}c{cellsize}.las'
                    copyfile(tree.odm_georeferencing_model_las, os.path.join(odm_dem_root, l2d_classified_pattern.format(**kwargs)))

                products = []
                if args.dsm: products.append('dsm') 
                if args.dtm: products.append('dtm')

                radius_steps = [args.dem_resolution]
                for _ in range(args.dem_gapfill_steps - 1):
                    radius_steps.append(radius_steps[-1] * 3) # 3 is arbitrary, maybe there's a better value?

                for product in products:
                    demargs = {
                        'product': product,
                        'indir': odm_dem_root,
                        'l2d_params': l2d_params,
                        'maxsd': args.dem_maxsd,
                        'maxangle': args.dem_maxangle,
                        'resolution': args.dem_resolution,
                        'radius_steps': ' '.join(map(str, radius_steps)),
                        'gapfill': '--gapfill' if args.dem_gapfill_steps > 0 else '',
                        
                        # If we didn't run a classification, we should pass the decimate parameter here
                        'decimation': '--decimation {0}'.format(args.dem_decimation) if not run_classification else ''
                    }

                    system.run('l2d_dems {product} {indir} {l2d_params} '
                               '--maxsd {maxsd} --maxangle {maxangle} '
                               '--resolution {resolution} --radius {radius_steps} '
                               '{decimation} '
                               '{gapfill} '.format(**demargs), env_paths)

                    # Rename final output
                    if product == 'dsm':
                        dsm_pattern = 'odm_georeferenced_model.bounds-0_dsm.idw.tif' if args.crop > 0 else 'dsm.idw.tif'
                        os.rename(os.path.join(odm_dem_root, dsm_pattern), dsm_output_filename)
                    elif product == 'dtm':
                        dtm_pattern = 'odm_georeferenced_model.bounds-0_dsm.idw.tif' if args.crop > 0 else 'dtm.idw.tif'
                        os.rename(os.path.join(odm_dem_root, dtm_pattern), dtm_output_filename)

            else:
                log.ODM_WARNING('Found existing outputs in: %s' % odm_dem_root)
        else:
            log.ODM_WARNING('DEM will not be generated')

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'Dem')

        log.ODM_INFO('Running ODM DEM Cell - Finished')
        return ecto.OK if args.end_with != 'odm_dem' else ecto.QUIT
