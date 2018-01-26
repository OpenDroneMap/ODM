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

        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'odm_dem') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'odm_dem' in args.rerun_from)

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

        # define paths and create working directories
        odm_dem_root = tree.path('odm_dem')
        if not io.dir_exists(odm_dem_root):
            system.mkdir_p(odm_dem_root)

        if args.pc_classify != "none" and las_model_found:
            pc_classify_marker = os.path.join(odm_dem_root, 'pc_classify_done.txt')

            if not io.file_exists(pc_classify_marker) or rerun_cell:
                log.ODM_INFO("Classifying {} using {}".format(tree.odm_georeferencing_model_las, args.pc_classify))
                commands.classify(tree.odm_georeferencing_model_las, 
                                  args.pc_classify == "smrf",
                                  slope,
                                  cellsize,
                                  approximate=args.dem_approximate,
                                  initialDistance=args.dem_initial_distance,
                                  verbose=args.verbose
                                )
                with open(pc_classify_marker, 'w') as f: 
                    f.write('Classify: {}\n'.format(args.pc_classify))
                    f.write('Slope: {}\n'.format(slope))
                    f.write('Cellsize: {}\n'.format(cellsize))
                    f.write('Approximate: {}\n'.format(args.dem_approximate))
                    f.write('InitialDistance: {}\n'.format(args.dem_initial_distance))

        # Do we need to process anything here?
        if (args.dsm or args.dtm) and las_model_found:
            dsm_output_filename = os.path.join(odm_dem_root, 'dsm.tif')
            dtm_output_filename = os.path.join(odm_dem_root, 'dtm.tif')

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
