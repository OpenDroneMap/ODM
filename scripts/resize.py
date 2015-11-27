import ecto
import cv2
import pyexiv2

from opendm import log
from opendm import system
from opendm import io
from opendm import types

class ODMResizeCell(ecto.Cell):    
    def declare_params(self, params):
        pass

    def declare_io(self, params, inputs, outputs):
        inputs.declare("args", "The application arguments.", [])
        inputs.declare("photos", "Clusters inputs. list of ODMPhoto's", [])
        outputs.declare("photos", "Clusters output. list of ODMPhoto's", [])

    def process(self, inputs, outputs):

        log.ODM_INFO('Running ODM Resize Cell')

        # get inputs
        args = self.inputs.args
        photos = self.inputs.photos
        project_path = io.absolute_path_file(args['project_path'])
        
        # loop over photos array
        if photos:
            # create working directory
            resizing_dir = io.join_paths(project_path, 'images_resize')
            system.mkdir_p(resizing_dir)

            log.ODM_DEBUG('Resizing dataset to: %s' % resizing_dir)

            # loop over photos
            for photo in photos:
                try:
                    # open and resize image with opencv
                    img = cv2.imread(photo.path_file)

                    # TODO(edgar): check if resize is needed, else copy img.
                    # Try to avoid oversampling!

                    # compute new size
                    max_side = max(photo.width, photo.height) \
                        if photo.width and photo.height \
                        else max(img.shape[0], img.shape[0])
                    ratio = float(args['resize_to']) / float(max_side)
                    img_r = cv2.resize(img, None, fx=ratio, fy=ratio)
                    # write image with opencv
                    new_path_file = io.join_paths(resizing_dir, photo.filename)
                    cv2.imwrite(new_path_file, img_r)
                    # read metadata with pyexiv2
                    old_meta = pyexiv2.ImageMetadata(photo.path_file)
                    new_meta = pyexiv2.ImageMetadata(new_path_file)
                    old_meta.read()
                    new_meta.read()
                    # copy metadata
                    old_meta.copy(new_meta)
                    new_meta.write()
                    # update photos array with new values
                    photo.path_file = new_path_file
                    photo.width = img_r.shape[0]
                    photo.height = img_r.shape[1]
                    photo.update_focal()

                    # log message
                    log.ODM_DEBUG('Resized image %s | dimensions: %s' % \
                        (photo.filename, img_r.shape))

                except cv2.error as e:
                    # something went wrong with this image
                    # TODO(edgar): remove photo from array
                    log.ODM_ERROR('Could not resize image %s' % photo.file_name)
                    log.ODM_ERROR('%s' % e)
        else:
            log.ODM_ERROR('Not enough photos in photos to resize')
            return ecto.QUIT

        log.ODM_INFO('Resized %s images' % len(photos))
        
        # append photos to cell output
        self.outputs.photos = photos

        log.ODM_INFO('Running ODM Resize Cell - Finished')
        return ecto.OK if args['end_with'] != 'resize' else ecto.QUIT