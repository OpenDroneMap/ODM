import ecto

from opendm import log
from opendm import io
from opendm import system
from opendm import context

class ODMTexturingCell(ecto.Cell):  

    def declare_io(self, params, inputs, outputs):
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("model_path", "Clusters output. list of reconstructions", [])
        outputs.declare("texture_path", "Clusters output. list of reconstructions", [])

    def process(self, inputs, outputs):
        
        log.ODM_INFO('Running OMD Texturing Cell')

        # get inputs
        args = self.inputs.args
        input_model_path = self.inputs.model_path
        project_path = io.absolute_path_file(args['project_path'])

        # define paths and create working directories
        odm_meshing = io.join_paths(project_path, 'odm_meshing')
        odm_texturing = io.join_paths(project_path, 'odm_texturing')
        system.mkdir_p(odm_texturing)

        output_file = io.join_paths(odm_texturing, 'odm_textured_model.obj')

        # check if we rerun cell or not
        rerun_cell = args['run_only'] is not None \
            and args['run_only'] == 'odm_texturing'

        if not io.file_exists(output_file) or rerun_cell:
            log.ODM_DEBUG('Writting odm textured file in: %s' % output_file)

            # odm_texturing definitions
            kwargs = {
                'bin': context.odm_modules_path,
                'out_dir': odm_texturing,
                'bundle': io.join_paths(project_path,'opensfm/bundle_r000.out'),
                'imgs_list': io.join_paths(project_path, 'opensfm/image_list.txt'),
                'model': io.join_paths(odm_meshing, 'odm_mesh.ply'),
                'log': io.join_paths(odm_texturing, 'odm_texturing_log.txt'),
                'bsize': str(args['resize_to']),
                'res': str(args['odm_texturing_textureResolution']),
                'wsize': str(args['odm_texturing_textureWithSize'])
            }

            # run texturing binary
            system.run('{bin}/odm_texturing -bundleFile {bundle} '     \
                '-imagesListPath {imgs_list} -inputModelPath {model} ' \
                '-outputFolder {out_dir}/ -textureResolution {res} '   \
                '-bundleResizedTo {bsize} -textureWithSize {wsize} '   \
                '-logFile {log}'.format(**kwargs))
        else:
            log.ODM_WARNING('Found a valid odm texture file in: %s' % output_file)
        
        self.outputs.texture_path = output_file
        
        log.ODM_INFO('Running OMD Texturing Cell - Finished')
        return ecto.OK if args['end_with'] != 'odm_texturing' else ecto.QUIT