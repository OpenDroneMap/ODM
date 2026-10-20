import unittest
import os
import shutil
import struct
import tempfile
from unittest import mock

import numpy as np

from opendm.get_image_size import get_image_size
from opendm.photo import ODM_Photo


def write_tiff_without_photometric(path, width, height):
    """
    Write a minimal little-endian, uncompressed, 16-bit single band TIFF
    without the PhotometricInterpretation tag (262), like the radiometric
    TIFFs produced by Workswell WIRIS cameras.
    """
    pixels = (np.arange(width * height, dtype="<u2") % 4096 + 20000).tobytes()
    make = b"Workswell\0"
    model = b"WIRIS Pro\0"

    entries = [
        # (tag, type, count, value) ; type 3 = SHORT, 4 = LONG, 2 = ASCII
        (256, 4, 1, width),
        (257, 4, 1, height),
        (258, 3, 1, 16),
        (259, 3, 1, 1),
        (271, 2, len(make), None),
        (272, 2, len(model), None),
        (273, 4, 1, None),
        (277, 3, 1, 1),
        (278, 4, 1, height),
        (279, 4, 1, len(pixels)),
    ]
    ifd_offset = 8
    ifd_size = 2 + len(entries) * 12 + 4
    make_offset = ifd_offset + ifd_size
    model_offset = make_offset + len(make)
    pixels_offset = model_offset + len(model)
    pixels_offset += pixels_offset % 2

    ifd = struct.pack("<H", len(entries))
    for tag, typ, count, value in entries:
        if tag == 271:
            value = make_offset
        elif tag == 272:
            value = model_offset
        elif tag == 273:
            value = pixels_offset

        if typ == 3:
            ifd += struct.pack("<HHIHH", tag, typ, count, value, 0)
        else:
            ifd += struct.pack("<HHII", tag, typ, count, value)
    ifd += struct.pack("<I", 0)

    with open(path, "wb") as f:
        f.write(b"II*\0" + struct.pack("<I", ifd_offset))
        f.write(ifd)
        f.write(make)
        f.write(model)
        f.write(b"\0" * (pixels_offset - model_offset - len(model)))
        f.write(pixels)


class TestGetImageSize(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.tiff = os.path.join(self.tmpdir, "01-08-44-569-radiometric.tiff")
        write_tiff_without_photometric(self.tiff, 64, 48)

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_tiff_without_photometric(self):
        self.assertEqual(get_image_size(self.tiff), (64, 48))

    def test_tiff_without_photometric_pil_failure(self):
        # Pillow < 9.3 cannot identify these files and OpenCV fails with
        # "failed TIFFGetField(tif, TIFFTAG_PHOTOMETRIC, &photometric)"
        with mock.patch("opendm.get_image_size.Image.open", side_effect=OSError("cannot identify image file")):
            self.assertEqual(get_image_size(self.tiff), (64, 48))

            photo = ODM_Photo(self.tiff)
            self.assertEqual(photo.width, 64)
            self.assertEqual(photo.height, 48)
            self.assertEqual(photo.camera_make, "Workswell")
            self.assertEqual(photo.camera_model, "WIRIS Pro")
            self.assertEqual(photo.bits_per_sample, 16)

    def test_unreadable_image_raises(self):
        bad = os.path.join(self.tmpdir, "bad.tiff")
        with open(bad, "wb") as f:
            f.write(b"not an image")

        with self.assertRaises(Exception):
            get_image_size(bad)

        with self.assertRaises(Exception):
            get_image_size(bad, fallback_on_error=False)


if __name__ == '__main__':
    unittest.main()
