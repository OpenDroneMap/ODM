import pillow_jxl # registers .jxl support with Pillow
from PIL import Image
import cv2
import rawpy
from opendm import log
from pathlib import Path
import rasterio as rio

Image.MAX_IMAGE_PIXELS = None

def get_image_size(file_path, fallback_on_error=True):
    """
    Return (width, height) for a given img file
    """
    
    try:
        extension = Path(file_path).suffix.lower()
        if extension in [".dng", ".raw", ".nef"]:
            with rawpy.imread(file_path) as img:
                s = img.sizes
                width, height = s.raw_width, s.raw_height
        elif extension == ".tif":
            with rio.open(file_path) as f:
                width, height = f.width, f.height
        else:
            with Image.open(file_path) as img:
                width, height = img.size
    except Exception as e:
        if fallback_on_error:
            log.ODM_WARNING("Cannot read %s with image library, fallback to cv2: %s" % (file_path, str(e)))
            img = cv2.imread(file_path)
            width = img.shape[1]
            height = img.shape[0]
        else:
            raise e

    return (width, height)