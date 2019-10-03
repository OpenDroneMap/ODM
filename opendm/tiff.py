from PIL import Image
import numpy


def Read_tiff(image_name):
    # I opened with PIL like TIFF image
    im = Image.open("a_image.tif")  # a_image is a tiff file
    im.show()
    imarray = numpy.array(im)  # change it to an array
    return imarray
