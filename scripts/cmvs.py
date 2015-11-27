import ecto

from opendm import io
from opendm import log
from opendm import system
from opendm import context

class ODMCmvsCell(ecto.Cell):  

    def declare_io(self, params, inputs, outputs):
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("reconstruction_path", "Clusters output. list of reconstructions", [])

    def process(self, inputs, outputs):
        
        log.ODM_INFO('Running OMD CMVS Cell')

        # get inputs
        args = self.inputs.args
        rec_path = self.inputs.reconstruction_path

        # the path to create the model
        model_path = io.join_paths(rec_path, 'models/pmvs_options.txt.ply')

        if io.file_exists(model_path):
            log.ODM_WARNING('Found a valid PMVS file')
            log.ODM_INFO('Running OMD OpenSfm Cell - Finished')
            return ecto.OK if args['end_with'] != 'opensfm' else ecto.QUIT

        # run pmvs2
        system.run('%s %s/ pmvs_options.txt ' % (context.pmvs2_path, rec_path))

        log.ODM_INFO('Running OMD CMVS Cell - Finished')
        return ecto.OK if args['end_with'] != 'cmvs' else ecto.QUIT
