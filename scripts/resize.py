import ecto
import cv2
import pyexiv2

from functools import partial
from multiprocessing import Pool
from opendm import log
from opendm import system
from opendm import io
from opendm import types


def resize(src_dir, target_dir, resize_to, rerun_cell, photo):
    # define image paths
    path_file = photo.path_file
    new_path_file = io.join_paths(target_dir, photo.filename)
    # set raw image path in case we want to rerun cell
    if io.file_exists(new_path_file) and rerun_cell:
        path_file = io.join_paths(src_dir, photo.filename)

    if not io.file_exists(new_path_file) or rerun_cell:
        # open and resize image with opencv
        img = cv2.imread(path_file)
        # compute new size
        max_side = max(img.shape[0], img.shape[1])
        if max_side <= resize_to:
            log.ODM_WARNING('Resize Parameter is greater than the largest side of the image')
        ratio = float(resize_to) / float(max_side)
        img_r = cv2.resize(img, None, fx=ratio, fy=ratio)
        # write image with opencv
        cv2.imwrite(new_path_file, img_r)
        # read metadata with pyexiv2
        old_meta = pyexiv2.ImageMetadata(path_file)
        new_meta = pyexiv2.ImageMetadata(new_path_file)
        old_meta.read()
        new_meta.read()
        # copy metadata
        old_meta.copy(new_meta)
        # update metadata size
        new_meta['Exif.Photo.PixelXDimension'] = img_r.shape[0]
        new_meta['Exif.Photo.PixelYDimension'] = img_r.shape[1]
        new_meta.write()
        # update photos array with new values
        photo.path_file = new_path_file
        photo.width = img_r.shape[0]
        photo.height = img_r.shape[1]
        photo.update_focal()

        # log message
        log.ODM_DEBUG('Resized %s | dimensions: %s' %
                      (photo.filename, img_r.shape))
    else:
        # log message
        log.ODM_WARNING('Already resized %s | dimensions: %s x %s' %
                        (photo.filename, photo.width, photo.height))

    return photo


class ODMResizeCell(ecto.Cell):
    def declare_params(self, params):
        params.declare("resize_to", "resizes images by the largest side", 2400)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", [])
        inputs.declare("photos", "Clusters inputs. list of ODMPhoto's", [])
        outputs.declare("photos", "Clusters output. list of ODMPhoto's", [])

    def process(self, inputs, outputs):

        # Benchmarking
        start_time = system.now_raw()

        log.ODM_INFO('Running ODM Resize Cell')

        # get inputs
        args = self.inputs.args
        tree = self.inputs.tree
        photos = self.inputs.photos

        if not photos:
            log.ODM_ERROR('Not enough photos in photos to resize')
            return ecto.QUIT

        if self.params.resize_to <= 0:
            log.ODM_ERROR('Resize parameter must be greater than 0')
            return ecto.QUIT

        # create working directory
        system.mkdir_p(tree.dataset_resize)

        log.ODM_DEBUG('Resizing dataset to: %s' % tree.dataset_resize)

        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'resize') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'resize' in args.rerun_from)

        # loop over photos
        photos = Pool().map(
            partial(resize,
                    tree.dataset_raw,
                    tree.dataset_resize,
                    self.params.resize_to,
                    rerun_cell),
            photos
        )

        log.ODM_INFO('Resized %s images' % len(photos))

        # append photos to cell output
        self.outputs.photos = photos

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'Resizing')

        log.ODM_INFO('Running ODM Resize Cell - Finished')
        return ecto.OK if args.end_with != 'resize' else ecto.QUIT
