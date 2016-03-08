import ecto

from opendm import io
from opendm import log
from opendm import system
from opendm import context


class ODMPmvsCell(ecto.Cell):
    def declare_params(self, params):
        params.declare("level", 'The level in the image pyramid that is used '
                                'for the computation', 1)
        params.declare("csize", 'Cell size controls the density of reconstructions', 2)
        params.declare("thresh", 'A patch reconstruction is accepted as a success '
                                 'and kept, if its associcated photometric consistency '
                                 'measure is above this threshold.', 0.7)
        params.declare("wsize", 'pmvs samples wsize x wsize pixel colors from '
                                'each image to compute photometric consistency '
                                'score. For example, when wsize=7, 7x7=49 pixel '
                                'colors are sampled in each image. Increasing the '
                                'value leads to more stable reconstructions, but '
                                'the program becomes slower.', 7)
        params.declare("min_imgs", 'Each 3D point must be visible in at least '
                                   'minImageNum images for being reconstructed. 3 is '
                                   'suggested in general.', 3)
        params.declare("cores", 'The maximum number of cores to use in dense '
                                ' reconstruction.', context.num_cores)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("reconstruction", "list of ODMReconstructions", [])
        outputs.declare("reconstruction", "list of ODMReconstructions", [])

    def process(self, inputs, outputs):

        # Benchmarking
        start_time = system.now_raw()

        log.ODM_INFO('Running OMD PMVS Cell')

        # get inputs
        args = self.inputs.args
        tree = self.inputs.tree

        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'pmvs') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'pmvs' in args.rerun_from)

        if not io.file_exists(tree.pmvs_model) or rerun_cell:
            log.ODM_DEBUG('Creating dense pointcloud in: %s' % tree.pmvs_model)

            kwargs = {
                'bin': context.cmvs_opts_path,
                'prefix': tree.pmvs_rec_path,
                'level': self.params.level,
                'csize': self.params.csize,
                'thresh': self.params.thresh,
                'wsize': self.params.wsize,
                'min_imgs': self.params.min_imgs,
                'cores': self.params.cores
            }

            # generate pmvs2 options
            system.run('{bin} {prefix}/ {level} {csize} {thresh} {wsize} '
                       '{min_imgs} {cores}'.format(**kwargs))

            # run pmvs2
            system.run('%s %s/ option-0000' %
                       (context.pmvs2_path, tree.pmvs_rec_path))

        else:
            log.ODM_WARNING('Found a valid PMVS file in %s' % tree.pmvs_model)

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'PMVS')

        log.ODM_INFO('Running ODM PMVS Cell - Finished')
        return ecto.OK if args.end_with != 'pmvs' else ecto.QUIT
