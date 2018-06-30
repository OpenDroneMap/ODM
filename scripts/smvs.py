import ecto

from opendm import log
from opendm import io
from opendm import system
from opendm import context


class ODMSmvsCell(ecto.Cell):
    def declare_params(self, params):
        params.declare("threads", "max number of threads", context.num_cores)
        params.declare("alpha", "Regularization parameter", 1)
        params.declare("scale", "input scale", 1)
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

            # make bundle directory
            if not io.file_exists(tree.mve_bundle):
                system.mkdir_p(tree.mve_path) #TODO: check permissions/what happens when rerun
                system.mkdir_p(io.join_paths(tree.mve_path, 'bundle'))
                io.copy(tree.opensfm_image_list, tree.mve_image_list)
                io.copy(tree.opensfm_bundle, tree.mve_bundle)

            # run mve makescene
            if not io.dir_exists(tree.mve_path):
                system.run('%s %s %s' % (context.makescene_path, tree.mve_path, tree.smvs))

            # config
            config = [
                "-t%s" % self.params.threads,
                "-a%s" % self.params.alpha,
                "-s%s" % self.params.scale,
                "-o%s" % self.params.output_scale,
                "--debug-lvl=%s" % '1' if self.params.verbose else '0',
                "%s" % '-S' if self.params.shading else '',
                "%s" % '-g' if self.params.gamma_srgb and self.params.shading else '',
                "--force" if rerun_cell else ''
            ]

            # run smvs
            system.run('%s %s %s' % (context.smvs_path, ' '.join(config), tree.smvs))
            # rename the file for simplicity
            old_file = io.join_paths(tree.smvs, 'smvs-%s%s.ply' %
                ('S' if self.params.shading else 'B', self.params.scale))
            if not (io.rename_file(old_file, tree.smvs_model)):
                log.ODM_WARNING("File %s does not exist, cannot be renamed. " % old_file)


        else:
            log.ODM_WARNING('Found a valid SMVS reconstruction file in: %s' %
                            tree.smvs_model)

        outputs.reconstruction = reconstruction

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'SMVS')

        log.ODM_INFO('Running ODM SMVS Cell - Finished')
        return ecto.OK if args.end_with != 'smvs' else ecto.QUIT
