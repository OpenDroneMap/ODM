from opendm import log
import cv2

def resize_to_match(image, match_photo = None):
    """
    Resize images to match the dimension of another photo
    :param image  numpy array containing image data to resize
    :param match_photo ODM_Photo whose dimensions should be used for resize
    :return numpy array with resized image data
    """
    if match_photo is not None:
        h, w, _ = image.shape
        if w != match_photo.width or h != match_photo.height:
            image = cv2.resize(image, None, 
                    fx=match_photo.width/w, 
                    fy=match_photo.height/h,
                    interpolation=cv2.INTER_LANCZOS4)
    return image

def dn_to_temperature(photo, image):
    """
    Convert Digital Number values to temperature (C) values
    :param photo ODM_Photo
    :param image numpy array containing image data
    :param resize_to_photo ODM_Photo that photo should be resized to (to match its dimensions)
    :return numpy array with temperature (C) image values
    """

    image = image.astype("float32")

    # Handle thermal bands
    if photo.is_thermal():
        # Every camera stores thermal information differently
        # The following will work for MicaSense Altum cameras
        # but not necessarily for others
        if photo.camera_make == "MicaSense" and photo.camera_model == "Altum":
            image -= (273.15 * 100.0) # Convert Kelvin to Celsius
            image *= 0.01
            return image
        else:
            log.ODM_WARNING("Unsupported camera [%s %s], thermal band will have digital numbers." % (photo.camera_make, photo.camera_model))
    else:
        log.ODM_WARNING("Tried to radiometrically calibrate a non-thermal image with temperature values (%s)" % photo.filename)
        return image

