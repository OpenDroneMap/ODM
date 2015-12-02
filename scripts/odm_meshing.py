import ecto

from opendm import log
from opendm import io
from opendm import system
from opendm import context

class ODMeshingCell(ecto.Cell):  

    def declare_io(self, params, inputs, outputs):
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("model_path", "Clusters output. list of reconstructions", [])
        outputs.declare("mesh_path", "Clusters output. list of reconstructions", [])

    def process(self, inputs, outputs):
        
        log.ODM_INFO('Running OMD Meshing Cell')

        # get inputs
        args = self.inputs.args
        model_path = self.inputs.model_path
        project_path = io.absolute_path_file(args['project_path'])

        # define paths and create working directories
        odm_meshing = io.join_paths(project_path, 'odm_meshing')
        system.mkdir_p(odm_meshing)
        output_file = io.join_paths(odm_meshing, 'odm_mesh.ply')
        log_file = io.join_paths(odm_meshing, 'odm_meshing_log.txt')

        self.outputs.mesh_path = output_file

        # check if we rerun cell or not
        rerun_cell = args['run_only'] is not None \
            and args['run_only'] == 'odm_meshing'

        if not io.file_exists(output_file) or rerun_cell:
            log.ODM_DEBUG('Writting odm mesh file in: %s' % output_file)

            # run meshing binary
            system.run('%s/odm_meshing -inputFile %s -outputFile %s '          \
                '-logFile %s -maxVertexCount %s -octreeDepth %s '              \
                '-samplesPerNode %s -solverDivide %s' %                        \
                (context.odm_modules_path, model_path, output_file, log_file,  \
                str(args['odm_meshing_maxVertexCount']),                       \
                str(args['odm_meshing_octreeDepth']),                          \
                str(args['odm_meshing_samplesPerNode']),                       \
                str(args['odm_meshing_solverDivide'])))
        else:
            log.ODM_WARNING('Found a valid odm mesh file in: %s' % 
                (output_file))
        
        log.ODM_INFO('Running OMD Meshing Cell - Finished')
        return ecto.OK if args['end_with'] != 'odm_meshing' else ecto.QUIT