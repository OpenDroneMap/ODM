import ecto, shutil, os, glob

from opendm import log
from opendm import io
from opendm import system
from opendm import context


class ODMMveCell(ecto.Cell):
#    def declare_params(self, params):
#        params.declare("output_scale", "scale of optimization", 2)

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
                system.run('%s %s %s' % (context.makescene_path, tree.mve_path, tree.mve))

            # config
            config = [
            ]

            # run mve
            system.run('%s %s %s' % (context.mve_path, ' '.join(config), tree.mve))
            system.run('%s %s %s' % (context.mve_path_pc, ' '.join(config), tree.mve))
            
            # find and rename the output file for simplicity
            mve_files = glob.glob(os.path.join(tree.mve, 'mve-*'))
            mve_files.sort(key=os.path.getmtime) # sort by last modified date
            if len(mve_files) > 0:
                old_file = mve_files[-1]
                if not (io.rename_file(old_file, tree.mve_model)):
                    log.ODM_WARNING("File %s does not exist, cannot be renamed. " % old_file)
            else:
                log.ODM_WARNING("Cannot find a valid point cloud (mve-XX.ply) in %s. Check the console output for errors." % tree.mve)
        else:
            log.ODM_WARNING('Found a valid MVE reconstruction file in: %s' %
                            tree.mve_model)

        outputs.reconstruction = reconstruction

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'MVE')

        log.ODM_INFO('Running ODM MVE Cell - Finished')
        return ecto.OK if args.end_with != 'mve' else ecto.QUIT
