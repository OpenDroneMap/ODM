import cv2
import pyexiv2

import log
import system
import dataset

def resize(images_dir, args, photos):

    log.ODM_DEBUG('Preparing images - %s' % images_dir)

    if len(photos) < 1:
        log.ODM_ERROR('Not enough photos in photos to resize')
        return False

    # create directory for resized images
    images_resize_dir = dataset.join_paths(images_dir, 'images_resize')
    system.mkdir_p(images_resize_dir)

    # define the new size
    new_size = (args['resize_to'], args['resize_to'])

    for photo in photos:
        try:
            # open and resize image with opencv
            img = cv2.imread(photo.path_file)
            img_r = cv2.resize(img, new_size)
            # write image with opencv
            new_path_file = dataset.join_paths(images_resize_dir, photo.file_name)
            cv2.imwrite(new_path_file, img_r)
            # read metadata
            old_meta = pyexiv2.ImageMetadata(photo.path_file)
            old_meta.read()
            new_meta = pyexiv2.ImageMetadata(new_path_file)
            new_meta.read()
            # copy metadata
            old_meta.copy(new_meta)
            new_meta.write()
            # log message
            log.ODM_DEBUG('Resized image %s | dimensions: %s' % \
                (photo.file_name, img_r.shape))
        except cv2.error as e:
            # something went wrong with this image
            log.ODM_ERROR('Could not resize image %s' % photo.file_name)
            log.ODM_ERROR('%s' % e)
            return False

    log.ODM_DEBUG('Resized %s images' % len(photos))
    return True