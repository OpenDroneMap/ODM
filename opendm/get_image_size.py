from PIL import Image
import cv2

from opendm import log

Image.MAX_IMAGE_PIXELS = None

def get_image_size(file_path, fallback_on_error=True):
    """
    Return (width, height) for a given img file
    """
    try:
        with Image.open(file_path) as img:
            width, height = img.size
    except Exception as e:
        if fallback_on_error:
            log.ODM_WARNING("Cannot read %s with PIL, fallback to cv2: %s" % (file_path, str(e)))
            img = cv2.imread(file_path)
            width = img.shape[1]
            height = img.shape[0]
        else:
            raise e

    return (width, height)