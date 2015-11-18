import os

import context
import datatypes

def load_dataset(images_dir, args, photos):

    # check if the extension is sopported
    def supported_extension(file_name):
        (pathfn, ext) = os.path.splitext(file_name)
        return ext.lower() in context.supported_extensions

    # find files in the given directory
    files = os.listdir(images_dir)

    # filter images for its extension type
    # by now only 'jpg' and 'jpeg are supported
    files = [f for f in files if supported_extension(f)]

    #photos = []

    # create ODMPhoto list
    for f in files:
        file_name = os.path.join(images_dir, f)
        photos.append(datatypes.ODMPhoto(file_name, args))

    #return photos