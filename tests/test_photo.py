import unittest
import os
import shutil

from opendm.photo import ODM_Photo


class TestPhoto(unittest.TestCase):
    def setUp(self):
        pass

    def test_jpeg_xl(self):
        photo = ODM_Photo("tests/assets/test_0001.jxl")
        self.assertEqual(photo.width, 1280)
        self.assertEqual(photo.height, 1024)
        self.assertEqual(photo.camera_make, "Lab 308")
        self.assertEqual(photo.camera_model, "Smart Pilot Cam")
        self.assertEqual(photo.orientation, 1)
        self.assertEqual(photo.utc_time, 1755297953000.0)
        self.assertEqual(photo.latitude, 44.058116008051655)
        self.assertEqual(photo.longitude, -121.31559406946837)
        self.assertEqual(photo.altitude, 2019.845)
        self.assertEqual(photo.yaw, 105.780071)
        self.assertEqual(photo.pitch, 40)
        self.assertEqual(photo.roll, 0)

if __name__ == '__main__':
    unittest.main()