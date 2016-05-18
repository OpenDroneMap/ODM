import os
import ecto

from opendm import context
from opendm import io
from opendm import types
from opendm import log


class ODMLoadDatasetCell(ecto.Cell):

    def declare_params(self, params):
        params.declare("force_focal", 'Override the focal length information for the '
                       'images', None)
        params.declare("force_ccd", 'Override the ccd widht information for the '
                       'images', None)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        outputs.declare("photos", "list of ODMPhoto's", [])

    def process(self, inputs, outputs):
        # check if the extension is sopported
        def supported_extension(file_name):
            (pathfn, ext) = os.path.splitext(file_name)
            return ext.lower() in context.supported_extensions

        log.ODM_INFO('Running ODM Load Dataset Cell')

        # get inputs
        tree = self.inputs.tree

        # set images directory
        images_dir = tree.dataset_resize

        if not io.dir_exists(images_dir):
            images_dir = tree.dataset_raw
            if not io.dir_exists(images_dir):
                log.ODM_ERROR("You must put your pictures into an <images> directory")
                return ecto.QUIT

        log.ODM_DEBUG('Loading dataset from: %s' % images_dir)

        # find files in the given directory
        files = io.get_files_list(images_dir)

        # filter images for its extension type
        files = [f for f in files if supported_extension(f)]

        if files:
            # create ODMPhoto list
            photos = []
            for f in files:
                path_file = io.join_paths(images_dir, f)
                photo = types.ODM_Photo(path_file,
                                        self.params.force_focal,
                                        self.params.force_ccd)
                photos.append(photo)
            
            log.ODM_INFO('Found %s usable images' % len(photos))            
        else:
            log.ODM_ERROR('Not enough supported images in %s' % images_dir)
            return ecto.QUIT

        # append photos to cell output
        outputs.photos = photos

        log.ODM_INFO('Running ODM Load Dataset Cell - Finished')
        return ecto.OK
