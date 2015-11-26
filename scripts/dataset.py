import os
import ecto

from opendm import context
from opendm import io
from opendm import types
from opendm import log

class ODMLoadDatasetCell(ecto.Cell):    
#    def declare_params(self, params):
#        params.declare("args", "The application arguments.", {})

    def declare_io(self, params, inputs, outputs):
        inputs.declare("images_dir", "The directory to the images to load.", "")
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
        images_dir = io.absolute_path_file(self.inputs.images_dir)

        log.ODM_DEBUG('Loading dataset from: %s' % images_dir)

        # find files in the given directory
        files = io.get_files_list(images_dir)

        # filter images for its extension type
        # by now only 'jpg' and 'jpeg are supported
        files = [f for f in files if supported_extension(f)]

        photos = []

        if len(files) < 1:
            log.ODM_ERROR('Not enough supported images in %s' % images_dir)
        else:
            # create ODMPhoto list
            for f in files:
                path_file = io.join_paths(images_dir, f)
                photos.append(types.ODMPhoto(path_file, args))
            log.ODM_INFO('Found %s usable images' % len(photos))

        # append photos to cell output
        outputs.photos = photos

        log.ODM_INFO('Running ODM Load Dataset Cell - Finished')
