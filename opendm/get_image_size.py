import pillow_jxl # registers .jxl support with Pillow
from PIL import Image
import cv2
import rawpy
import rasterio
import warnings
from opendm import log

Image.MAX_IMAGE_PIXELS = None

def get_image_size(file_path, fallback_on_error=True):
    """
    Return (width, height) for a given img file
    """
    
    try:
        if file_path[-4:].lower() in [".dng", ".raw", ".nef"]:
            with rawpy.imread(file_path) as img:
                s = img.sizes
                width, height = s.raw_width, s.raw_height
        else:
            with Image.open(file_path) as img:
                width, height = img.size
    except Exception as e:
        if fallback_on_error:
            log.ODM_WARNING("Cannot read %s with image library, using fallback: %s" % (file_path, str(e)))

            # Some TIFFs (e.g. Workswell WIRIS radiometric images) lack the
            # PhotometricInterpretation tag, which older Pillow and OpenCV
            # refuse to read. GDAL handles them, so try rasterio first.
            if file_path[-5:].lower() == ".tiff" or file_path[-4:].lower() == ".tif":
                try:
                    with warnings.catch_warnings():
                        warnings.simplefilter("ignore", rasterio.errors.NotGeoreferencedWarning)
                        with rasterio.open(file_path, 'r') as img:
                            return (img.width, img.height)
                except Exception as rio_e:
                    log.ODM_WARNING("Cannot read %s with rasterio: %s" % (file_path, str(rio_e)))

            img = cv2.imread(file_path)
            if img is None:
                raise e
            width = img.shape[1]
            height = img.shape[0]
        else:
            raise e

    return (width, height)
