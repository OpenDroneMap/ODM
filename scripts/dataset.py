import os
import ecto

from functools import partial
from multiprocessing import Pool
from opendm import context
from opendm import io
from opendm import types
from opendm import log
from opendm import system
from shutil import copyfile


def make_odm_photo(force_focal, force_ccd, path_file):
    return types.ODM_Photo(path_file,
                           force_focal,
                           force_ccd)


class ODMLoadDatasetCell(ecto.Cell):

    def declare_params(self, params):
        params.declare("force_focal", 'Override the focal length information for the '
                       'images', None)
        params.declare("force_ccd", 'Override the ccd width information for the '
                       'images', None)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        outputs.declare("photos", "list of ODMPhotos", [])

    def process(self, inputs, outputs):
        # check if the extension is supported
        def supported_extension(file_name):
            (pathfn, ext) = os.path.splitext(file_name)
            return ext.lower() in context.supported_extensions

        # Get supported images from dir
        def get_images(in_dir):
            # filter images for its extension type
            log.ODM_DEBUG(in_dir)
            return [f for f in io.get_files_list(in_dir) if supported_extension(f)]

        log.ODM_INFO('Running ODM Load Dataset Cell')

        # get inputs
        tree = self.inputs.tree

        # get images directory
        input_dir = tree.input_images
        images_dir = tree.dataset_raw
        resize_dir = tree.dataset_resize

        # Check first if a project already exists. This is a mediocre way to check, by checking the resize dir
        if io.dir_exists(resize_dir):
            log.ODM_DEBUG("resize dir: %s" % resize_dir)
            images_dir = resize_dir
        # if first time running, create project directory and copy images over to project/images
        else:
            if not io.dir_exists(images_dir):
                log.ODM_INFO("Project directory %s doesn't exist. Creating it now. " % images_dir)
                system.mkdir_p(images_dir)
                copied = [copyfile(io.join_paths(input_dir, f), io.join_paths(images_dir, f)) for f in get_images(input_dir)]

        log.ODM_DEBUG('Loading dataset from: %s' % images_dir)

        files = get_images(images_dir)

        if files:
            # create ODMPhoto list
            path_files = [io.join_paths(images_dir, f) for f in files]
            photos = Pool().map(
                partial(make_odm_photo, self.params.force_focal, self.params.force_ccd),
                path_files
            )
            
            log.ODM_INFO('Found %s usable images' % len(photos))            
        else:
            log.ODM_ERROR('Not enough supported images in %s' % images_dir)
            return ecto.QUIT

        # append photos to cell output
        outputs.photos = photos

        log.ODM_INFO('Running ODM Load Dataset Cell - Finished')
        return ecto.OK
