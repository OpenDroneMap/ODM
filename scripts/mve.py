import ecto, shutil, os, glob, math

from opendm import log
from opendm import io
from opendm import system
from opendm import context
from opendm import point_cloud

class ODMMveCell(ecto.Cell):
    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("reconstruction", "ODMReconstruction", [])
        outputs.declare("reconstruction", "list of ODMReconstructions", [])

    def process(self, inputs, outputs):
        # Benchmarking
        start_time = system.now_raw()

        log.ODM_INFO('Running MVE Cell')

        # get inputs
        tree = inputs.tree
        args = inputs.args
        reconstruction = inputs.reconstruction
        photos = reconstruction.photos

        if not photos:
            log.ODM_ERROR('Not enough photos in photos array to start MVE')
            return ecto.QUIT

        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'mve') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'mve' in args.rerun_from)

        # check if reconstruction was done before
        if not io.file_exists(tree.mve_model) or rerun_cell:
            # cleanup if a rerun
            if io.dir_exists(tree.mve_path) and rerun_cell:
                shutil.rmtree(tree.mve_path)

            # make bundle directory
            if not io.file_exists(tree.mve_bundle):
                system.mkdir_p(tree.mve_path)
                system.mkdir_p(io.join_paths(tree.mve_path, 'bundle'))
                io.copy(tree.opensfm_image_list, tree.mve_image_list)
                io.copy(tree.opensfm_bundle, tree.mve_bundle)

            # mve makescene wants the output directory
            # to not exists before executing it (otherwise it
            # will prompt the user for confirmation)
            if io.dir_exists(tree.mve):
                shutil.rmtree(tree.mve)

            # run mve makescene
            if not io.dir_exists(tree.mve_views):
                system.run('%s %s %s' % (context.makescene_path, tree.mve_path, tree.mve), env_vars={'OMP_NUM_THREADS': args.max_concurrency})

            # Compute mve output scale based on depthmap_resolution
            max_width = 0
            max_height = 0
            for photo in photos:
                max_width = max(photo.width, max_width)
                max_height = max(photo.height, max_height)

            max_pixels = args.depthmap_resolution * args.depthmap_resolution
            if max_width * max_height <= max_pixels:
                mve_output_scale = 0
            else:
                ratio = float(max_width * max_height) / float(max_pixels)
                mve_output_scale = int(math.ceil(math.log(ratio) / math.log(4.0)))

            dmrecon_config = [
                "-s%s" % mve_output_scale,
	            "--progress=silent",
                "--local-neighbors=2",
                "--force",
            ]

            # Run MVE's dmrecon
            log.ODM_INFO('                                                                               ')
            log.ODM_INFO('                                    ,*/**                                      ')
            log.ODM_INFO('                                  ,*@%*/@%*                                    ')
            log.ODM_INFO('                                ,/@%******@&*.                                 ')
            log.ODM_INFO('                              ,*@&*********/@&*                                ')
            log.ODM_INFO('                            ,*@&**************@&*                              ')
            log.ODM_INFO('                          ,/@&******************@&*.                           ')
            log.ODM_INFO('                        ,*@&*********************/@&*                          ')
            log.ODM_INFO('                      ,*@&**************************@&*.                       ')
            log.ODM_INFO('                    ,/@&******************************&&*,                     ')
            log.ODM_INFO('                  ,*&&**********************************@&*.                   ')
            log.ODM_INFO('                ,*@&**************************************@&*.                 ')
            log.ODM_INFO('              ,*@&***************#@@@@@@@@@%****************&&*,               ')
            log.ODM_INFO('            .*&&***************&@@@@@@@@@@@@@@****************@@*.             ')
            log.ODM_INFO('          .*@&***************&@@@@@@@@@@@@@@@@@%****(@@%********@@*.           ')
            log.ODM_INFO('        .*@@***************%@@@@@@@@@@@@@@@@@@@@@#****&@@@@%******&@*,         ')
            log.ODM_INFO('      .*&@****************@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@/*****@@*.       ')
            log.ODM_INFO('    .*@@****************@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@%*************@@*.     ')
            log.ODM_INFO('  .*@@****/***********@@@@@&**(@@@@@@@@@@@@@@@@@@@@@@@#*****************%@*,   ')
            log.ODM_INFO(' */@*******@*******#@@@@%*******/@@@@@@@@@@@@@@@@@@@@********************/@(,  ')
            log.ODM_INFO(' ,*@(********&@@@@@@#**************/@@@@@@@#**(@@&/**********************@&*   ')
            log.ODM_INFO('   *#@/*******************************@@@@@***&@&**********************&@*,    ')
            log.ODM_INFO('     *#@#******************************&@@@***@#*********************&@*,      ')
            log.ODM_INFO('       */@#*****************************@@@************************@@*.        ')
            log.ODM_INFO('         *#@/***************************/@@/*********************%@*,          ')
            log.ODM_INFO('           *#@#**************************#@@%******************%@*,            ')
            log.ODM_INFO('             */@#*************************(@@@@@@@&%/********&@*.              ')
            log.ODM_INFO('               *(@(*********************************/%@@%**%@*,                ')
            log.ODM_INFO('                 *(@%************************************%@**                  ')
            log.ODM_INFO('                   **@%********************************&@*,                    ')
            log.ODM_INFO('                     *(@(****************************%@/*                      ')
            log.ODM_INFO('                       ,(@%************************#@/*                        ')
            log.ODM_INFO('                         ,*@%********************&@/,                          ')
            log.ODM_INFO('                           */@#****************#@/*                            ')
            log.ODM_INFO('                             ,/@&************#@/*                              ')
            log.ODM_INFO('                               ,*@&********%@/,                                ')
            log.ODM_INFO('                                 */@#****(@/*                                  ')
            log.ODM_INFO('                                   ,/@@@@(*                                    ')
            log.ODM_INFO('                                     .**,                                      ')
            log.ODM_INFO('')
            log.ODM_INFO("Running dense reconstruction. This might take a while. Please be patient, the process is not dead or hung.")
            log.ODM_INFO("                              Process is running")
            system.run('%s %s %s' % (context.dmrecon_path, ' '.join(dmrecon_config), tree.mve), env_vars={'OMP_NUM_THREADS': args.max_concurrency})

            scene2pset_config = [
                "-F%s" % mve_output_scale
            ]

            # run scene2pset
            system.run('%s %s "%s" "%s"' % (context.scene2pset_path, ' '.join(scene2pset_config), tree.mve, tree.mve_model), env_vars={'OMP_NUM_THREADS': args.max_concurrency})
        else:
            log.ODM_WARNING('Found a valid MVE reconstruction file in: %s' %
                            tree.mve_model)

        outputs.reconstruction = reconstruction

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'MVE')

        log.ODM_INFO('Running ODM MVE Cell - Finished')
        return ecto.OK if args.end_with != 'mve' else ecto.QUIT
