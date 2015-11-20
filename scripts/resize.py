import cv2
import pyexiv2

from opendm import log
from opendm import system
from opendm import dataset

def resize(project_path, args, photos):

    log.ODM_INFO('Preparing images - %s' % project_path)

    # check if we have input data
    if len(photos) == 0:
        log.ODM_WARNING('Photos array is empty - Proceed to load images')
        images_dir = dataset.join_paths(project_path, 'images')
        photos = dataset.load_dataset(images_dir, args)

    # preconditions
    if len(photos) < 1:
        log.ODM_ERROR('Not enough photos in photos to resize')
        return False

    # create working directory
    working_dir = dataset.join_paths(project_path, 'images_resize')
    system.mkdir_p(working_dir)

    # loop over photos
    for photo in photos:
        try:
            # open and resize image with opencv
            img = cv2.imread(photo.path_file)
            # compute new size
            max_side = max(photo.width, photo.height)
            ratio = float(args['resize_to']) / float(max_side)
            img_r = cv2.resize(img, None, fx=ratio, fy=ratio)
            # write image with opencv
            new_path_file = dataset.join_paths(working_dir, photo.file_name)
            cv2.imwrite(new_path_file, img_r)
            # read metadata with pyexiv2
            old_meta = pyexiv2.ImageMetadata(photo.path_file)
            new_meta = pyexiv2.ImageMetadata(new_path_file)
            old_meta.read()
            new_meta.read()
            # copy metadata
            old_meta.copy(new_meta)
            new_meta.write()
            # log message
            log.ODM_DEBUG('Resized image %s | dimensions: %s' % \
                (photo.file_name, img_r.shape))

            # TODO(edgar): update photos array

        except cv2.error as e:
            # something went wrong with this image
            log.ODM_ERROR('Could not resize image %s' % photo.file_name)
            log.ODM_ERROR('%s' % e)
            return False

    log.ODM_INFO('Resized %s images' % len(photos))
    return True