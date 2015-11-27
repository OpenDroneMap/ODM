import os
import ecto

from opendm import context
from opendm import io
from opendm import types
from opendm import log

class ODMLoadDatasetCell(ecto.Cell):

    def declare_params(self, params):
        pass

    def declare_io(self, params, inputs, outputs):
        inputs.declare("args", "The application arguments.", {})
        outputs.declare("photos", "Clusters output. list of ODMPhoto's", [])

    def process(self, inputs, outputs):
        # check if the extension is sopported
        def supported_extension(file_name):
            (pathfn, ext) = os.path.splitext(file_name)
            return ext.lower() in context.supported_extensions

        log.ODM_INFO('Running ODM Load Dataset Cell')

        # get parameters
        args = self.inputs.args
        project_path = io.absolute_path_file(args['project_path'])
        images_dir = io.join_paths(project_path, 'images')

        log.ODM_DEBUG('Loading dataset from: %s' % images_dir)

        # find files in the given directory
        files = io.get_files_list(images_dir)

        # filter images for its extension type
        # by now only 'jpg' and 'jpeg are supported
        files = [f for f in files if supported_extension(f)]

        if files:
            photos = []

            # create ODMPhoto list
            for f in files:
                path_file = io.join_paths(images_dir, f)
                photos.append(types.ODMPhoto(path_file, args))
            
            log.ODM_INFO('Found %s usable images' % len(photos))            
        else:
            log.ODM_ERROR('Not enough supported images in %s' % images_dir)
            return ecto.QUIT

        # append photos to cell output
        outputs.photos = photos

        log.ODM_INFO('Running ODM Load Dataset Cell - Finished')
        return ecto.OK