import os

import log
import context
import datatypes

def load_dataset(images_dir, args):

    # check if the extension is sopported
    def supported_extension(file_name):
        (pathfn, ext) = os.path.splitext(file_name)
        return ext.lower() in context.supported_extensions

    log.ODM_DEBUG('Loading dataset from: %s' % images_dir)

    # find files in the given directory
    files = os.listdir(images_dir)

    # filter images for its extension type
    # by now only 'jpg' and 'jpeg are supported
    files = [f for f in files if supported_extension(f)]

    if len(files) < 1:
        log.ODM_ERROR('Not found enough supported images in %s' % images_dir)
        return

    photos = []

    # create ODMPhoto list
    for f in files:
        path_file = os.path.join(images_dir, f)
        photos.append(datatypes.ODMPhoto(path_file, args))

    log.ODM_INFO('Found %s usable images' % len(photos))
    return photos

def extract_file_from_path_file(path_file):
    path, file = os.path.split(path_file)
    return file

def extract_path_from_file(file):
    path_file = os.path.abspath(os.path.dirname(file))
    path, file = os.path.split(path_file)
    return path

def join_paths(path1, path2):
    return os.path.join(path1, path2)

