import ecto, shutil, os, glob

from opendm import log
from opendm import io
from opendm import system
from opendm import context


class ODMSmvsCell(ecto.Cell):
    def declare_params(self, params):
        params.declare("threads", "max number of threads", context.num_cores)
        params.declare("alpha", "Regularization parameter", 1)
        params.declare("max_pixels", "max pixels for reconstruction", 1700000)
        params.declare("output_scale", "scale of optimization", 2)
        params.declare("shading", "Enable shading-aware model", False)
        params.declare("gamma_srgb", "Apply inverse SRGB gamma correction", False)
        params.declare("verbose", "Increase debug level", False)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("reconstruction", "ODMReconstruction", [])
        outputs.declare("reconstruction", "list of ODMReconstructions", [])

    def process(self, inputs, outputs):

        # Benchmarking
        start_time = system.now_raw()

        log.ODM_INFO('Running SMVS Cell')

        # get inputs
        tree = inputs.tree
        args = inputs.args
        reconstruction = inputs.reconstruction
        photos = reconstruction.photos

        if not photos:
            log.ODM_ERROR('Not enough photos in photos array to start SMVS')
            return ecto.QUIT

        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'smvs') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'smvs' in args.rerun_from)

        # check if reconstruction was done before
        if not io.file_exists(tree.smvs_model) or rerun_cell:
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
            if io.dir_exists(tree.smvs):
                shutil.rmtree(tree.smvs)

            # run mve makescene
            if not io.dir_exists(tree.mve_views):
                system.run('%s %s %s' % (context.makescene_path, tree.mve_path, tree.smvs))

            # config
            config = [
                "-t%s" % self.params.threads,
                "-a%s" % self.params.alpha,
                "--max-pixels=%s" % int(self.params.max_pixels),
                "-o%s" % self.params.output_scale,
                "--debug-lvl=%s" % ('1' if self.params.verbose else '0'),
                "%s" % '-S' if self.params.shading else '',
                "%s" % '-g' if self.params.gamma_srgb and self.params.shading else '',
                "--force" if rerun_cell else ''
            ]

            # run smvs
            system.run('%s %s %s' % (context.smvs_path, ' '.join(config), tree.smvs))
            
            # find and rename the output file for simplicity
            smvs_files = glob.glob(os.path.join(tree.smvs, 'smvs-*'))
            smvs_files.sort(key=os.path.getmtime) # sort by last modified date
            if len(smvs_files) > 0:
                old_file = smvs_files[-1]
                if not (io.rename_file(old_file, tree.smvs_model)):
                    log.ODM_WARNING("File %s does not exist, cannot be renamed. " % old_file)
            else:
                log.ODM_WARNING("Cannot find a valid point cloud (smvs-XX.ply) in %s. Check the console output for errors." % tree.smvs)
        else:
            log.ODM_WARNING('Found a valid SMVS reconstruction file in: %s' %
                            tree.smvs_model)

        outputs.reconstruction = reconstruction

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'SMVS')

        log.ODM_INFO('Running ODM SMVS Cell - Finished')
        return ecto.OK if args.end_with != 'smvs' else ecto.QUIT
