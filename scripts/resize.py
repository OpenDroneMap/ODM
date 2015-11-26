import ecto
import cv2
import pyexiv2

from opendm import log
from opendm import system
from opendm import io

class OMDResizeCell(ecto.Cell):    
    def declare_params(self, params):
        params.declare("args", "The application arguments.", [])

    def declare_io(self, params, inputs, outputs):
        inputs.declare("project_path", "Clusters output. list of tuples", [])
        inputs.declare("photos", "Clusters inputs. list of ODMPhoto's", [])
        outputs.declare("photos", "Clusters output. list of ODMPhoto's", [])

    def process(self, inputs, outputs):
        # get parameters
        args = self.params.args
        # get inputs
        photos = inputs.photos
        project_path = inputs.project_path

        # preconditions
        if len(photos) < 1:
            log.ODM_ERROR('Not enough photos in photos to resize')
        else:
            # create working directory
            working_dir = io.join_paths(project_path, 'images_resize')
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
                    new_path_file = io.join_paths(working_dir, photo.file_name)
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
                    photos[photo] = datatypes.ODMPhoto(photo.path_file, args)

                except cv2.error as e:
                    # something went wrong with this image
                    log.ODM_ERROR('Could not resize image %s' % photo.file_name)
                    log.ODM_ERROR('%s' % e)

            log.ODM_INFO('Resized %s images' % len(photos))
            # append photos to cell output
            outputs.photos = photos