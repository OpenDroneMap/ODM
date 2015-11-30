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
        odm_texturing = io.join_paths(project_path, 'odm_texturing')
        system.mkdir_p(odm_texturing)

        images_path = io.join_paths(project_path, 'images_resize')
        bundle_file = io.join_paths(project_path, 'opensfm/bundle_r000.out')
        images_list_path = io.join_paths(project_path, 'opensfm/list_r000.out')

        output_file = io.join_paths(odm_texturing, 'odm_mesh.ply')
        log_file = io.join_paths(odm_texturing, 'odm_texturing_log.txt')

        if not io.file_exists(output_file):
            # run texturing binary
            system.run('%s/odm_texturing -bundleFile %s '                     \
                '-imagesPath %s/ -imagesListPath %s -inputModelPath %s '      \
                '-outputFolder %s/ -textureResolution %s -bundleResizedTo %s '\
                '-textureWithSize %s -logFile %s' %                           \
                (context.odm_modules_path, bundle_file, images_path,          \
                images_list_path, input_model_path, odm_texturing,            \
                str(args['odm_texturing_textureResolution']),                 \
                str(args['resize_to']),                                       \
                str(args['odm_texturing_textureWithSize']), log_file))
        else:
            log.ODM_WARNING('Found a valid odm texture file in: %s' % output_file)
        
        self.outputs.texture_path = output_file
        
        log.ODM_INFO('Running OMD Texturing Cell - Finished')
        return ecto.OK if args['end_with'] != 'odm_texturing' else ecto.QUIT