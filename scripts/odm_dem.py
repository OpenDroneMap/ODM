import ecto, os, json
from shutil import copyfile

from opendm import io
from opendm import log
from opendm import system
from opendm import context
from opendm import types
from opendm.dem import commands
from opendm.cropper import Cropper


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

        log.ODM_INFO('Classify: ' + str(args.pc_classify != "none"))
        log.ODM_INFO('Create DSM: ' + str(args.dsm))
        log.ODM_INFO('Create DTM: ' + str(args.dtm))
        log.ODM_INFO('DEM input file {0} found: {1}'.format(tree.odm_georeferencing_model_las, str(las_model_found)))

        # Setup terrain parameters
        terrain_params_map = {
            'flatnonforest': (1, 3), 
            'flatforest': (1, 2), 
            'complexnonforest': (5, 2), 
            'complexforest': (10, 2)
        }
        terrain_params = terrain_params_map[args.dem_terrain_type.lower()]             
        slope, cellsize = terrain_params

        if args.pc_classify != "none" and las_model_found:
            log.ODM_INFO("Classifying {} using {}".format(tree.odm_georeferencing_model_las, args.pc_classify))
            commands.classify(tree.odm_georeferencing_model_las, 
                              args.pc_classify == "smrf",
                              slope,
                              cellsize,
                              approximate=args.dem_approximate,
                              initialDistance=args.dem_initial_distance,
                              verbose=args.verbose
                            )

        # Do we need to process anything here?
        if (args.dsm or args.dtm) and las_model_found:

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

                products = []
                if args.dsm: products.append('dsm') 
                if args.dtm: products.append('dtm')

                radius_steps = [args.dem_resolution]
                for _ in range(args.dem_gapfill_steps - 1):
                    radius_steps.append(radius_steps[-1] * 3) # 3 is arbitrary, maybe there's a better value?

                for product in products:
                    commands.create_dems(
                            [tree.odm_georeferencing_model_las], 
                            product,
                            radius=map(str, radius_steps),
                            gapfill=True,
                            outdir=odm_dem_root,
                            resolution=args.dem_resolution,
                            maxsd=args.dem_maxsd,
                            maxangle=args.dem_maxangle,
                            decimation=args.dem_decimation,
                            verbose=args.verbose
                        )

                    if args.crop > 0:
                        bounds_shapefile_path = os.path.join(tree.odm_georeferencing, 'odm_georeferenced_model.bounds.shp')
                        if os.path.exists(bounds_shapefile_path):
                            Cropper.crop(bounds_shapefile_path, os.path.join(odm_dem_root, "{}.tif".format(product)), {
                                'TILED': 'YES',
                                'COMPRESS': 'LZW',
                                'BLOCKXSIZE': 512,
                                'BLOCKYSIZE': 512,
                                'NUM_THREADS': 'ALL_CPUS'
                            })
            else:
                log.ODM_WARNING('Found existing outputs in: %s' % odm_dem_root)
        else:
            log.ODM_WARNING('DEM will not be generated')

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'Dem')

        log.ODM_INFO('Running ODM DEM Cell - Finished')
        return ecto.OK if args.end_with != 'odm_dem' else ecto.QUIT
