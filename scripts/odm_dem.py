import ecto, os, math

from opendm import io
from opendm import log
from opendm import system
from opendm import context
from opendm import types


class ODMDemCell(ecto.Cell):
    def declare_params(self, params):
        params.declare("verbose", 'print additional messages to console', False)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("reconstruction", "list of ODMReconstructions", [])

    def process(self, inputs, outputs):
        # Benchmarking
        start_time = system.now_raw()

        log.ODM_INFO('Running ODM DEM Cell')

        # get inputs
        args = self.inputs.args
        las_model_exists = io.file_exists(tree.odm_georeferencing_model_las)

        # Just to make sure
        l2d_module_installed = True
        try:
            system.run('l2d_classify --help')
        except:
            log.ODM_WARNING('lidar2dems is not installed properly')
            l2d_module_installed = False

        log.ODM_INFO('Create DSM: ' + str(args.dsm))
        log.ODM_INFO('Create DTM: ' + str(args.dtm))
        log.ODM_INFO('{0} exists: {1}'.format(tree.odm_georeferencing_model_las, str(las_model_exists)))

        # Do we need to process anything here?
        if (args.dsm or args.dtm) and las_model_exists and l2d_module_installed:

            # define paths and create working directories
            odm_dem_root = tree.path('odm_dem')
            system.mkdir_p(odm_dem_root)

            dsm_output_filename = os.path.join(odm_dem_root, 'dsm.idw.tif')
            dtm_output_filename = os.path.join(odm_dem_root, 'dtm.idw.tif')

            # check if we rerun cell or not
            rerun_cell = (args.rerun is not None and
                          args.rerun == 'odm_dem') or \
                         (args.rerun_all) or \
                         (args.rerun_from is not None and
                          'odm_dem' in args.rerun_from)

            if (args.dtm and not io.file_exists(dtm_output_filename)) or \
                (args.dsm and not io.file_exists(dsm_output_filename)) or \
                rerun_cell:

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
                    'approximate': '-a' if args.dem_approximate else '',
                    'decimation': args.dem_decimation,
                }

                l2d_params = '--slope {slope} --cellsize {cellsize} ' \
                             '{verbose} {approximate} --decimation {decimation} ' \
                             '-o ' \
                             '--outdir {outdir}'.format(**kwargs)

                system.run('l2d_classify {0} {1}'.format(l2d_params, tree.odm_georeferencing))

                products = []
                if args.dsm: products.append('dsm') 
                if args.dtm: products.append('dtm')

                radius_steps = [args.dem_resolution]
                for _ in range(args.dem_gapfill_steps - 1):
                    radius_steps.append(radius_steps[-1] * math.sqrt(2))

                for product in products:
                    demargs = {
                        'product': product,
                        'indir': odm_dem_root,
                        'l2d_params': l2d_params,
                        'maxsd': args.dem_maxsd,
                        'maxangle': args.dem_maxangle,
                        'resolution': args.dem_resolution,
                        'radius_steps': ' '.join(map(str, radius_steps))
                        'gapfill': '--gapfill' if len(args.dem_gapfill_steps) > 0 else ''
                    }

                    system.run('l2d_dems {product} {indir} {l2d_params} '
                               '--maxsd {maxsd} --maxangle {maxangle} '
                               '--resolution {resolution} --radius {radius_steps} '
                               '{gapfill} '.format(**demargs))


            else:
                log.ODM_WARNING('Found existing outputs in: %s' % odm_dem_root)
        else:
            log.ODM_WARNING('DEM will not be generated')

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'Dem')

        log.ODM_INFO('Running ODM DEM Cell - Finished')
        return ecto.OK if args.end_with != 'odm_dem' else ecto.QUIT
