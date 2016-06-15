import ecto

from opendm import io
from opendm import log
from opendm import system
from opendm import context


class ODMCmvsCell(ecto.Cell):

    def declare_params(self, params):
        params.declare("max_images", 'The maximum number of images '
                                     'per cluster', 500)
        params.declare("cores", 'The maximum number of cores to use '
                                'in dense reconstruction.', context.num_cores)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "Struct with paths", [])
        inputs.declare("reconstruction", "list of ODMReconstructions", [])
        outputs.declare("reconstruction", "list of ODMReconstructions", [])

    def process(self, inputs, outputs):

        # Benchmarking
        start_time = system.now_raw()
        
        log.ODM_INFO('Running ODM CMVS Cell')

        # get inputs 
        args = self.inputs.args
        tree = self.inputs.tree

        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'cmvs') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'cmvs' in args.rerun_from)

        if not io.file_exists(tree.pmvs_bundle) or rerun_cell:
            log.ODM_DEBUG('Writing CMVS vis in: %s' % tree.pmvs_bundle)

            # copy bundle file to pmvs dir
            from shutil import copyfile
            copyfile(tree.opensfm_bundle, 
                     tree.pmvs_bundle)

            kwargs = {
                'bin': context.cmvs_path,
                'prefix': self.inputs.tree.pmvs_rec_path,
                'max_images': self.params.max_images,
                'cores': self.params.cores
            }

            # run cmvs
            system.run('{bin} {prefix}/ {max_images} {cores}'.format(**kwargs))
        else:
            log.ODM_WARNING('Found a valid CMVS file in: %s' % 
                            tree.pmvs_bundle)

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'CMVS')

        log.ODM_INFO('Running ODM CMVS Cell - Finished')
        return ecto.OK if args.end_with != 'cmvs' else ecto.QUIT
