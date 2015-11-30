import ecto

from opendm import io
from opendm import log
from opendm import system
from opendm import context

class ODMPmvsCell(ecto.Cell):  

    def declare_io(self, params, inputs, outputs):
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("reconstruction_path", "Clusters output. list of reconstructions", [])
        outputs.declare("model_path", "Clusters output. list of reconstructions", [])

    def process(self, inputs, outputs):
        
        log.ODM_INFO('Running OMD PMVS Cell')

        # get inputs
        args = self.inputs.args
        rec_path = self.inputs.reconstruction_path

        # the path to create the model
        model_path = io.join_paths(rec_path, 'models/pmvs_options.txt.ply')
        # attach created model to output
        self.outputs.model_path = model_path

        if not io.file_exists(model_path):
            log.ODM_DEBUG('Creating dense pointcloud in: %s' % model_path)

            # run pmvs2
            system.run('%s %s/ pmvs_options.txt ' % (context.pmvs2_path, rec_path))
        else:
            log.ODM_WARNING('Found a valid PMVS file in %s' % model_path)

        log.ODM_INFO('Running OMD PMVS Cell - Finished')
        return ecto.OK if args['end_with'] != 'pmvs' else ecto.QUIT
