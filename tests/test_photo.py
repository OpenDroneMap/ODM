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

    def _gps_std(self, xy_stddev, z_stddev, gps_accuracy=None):
        # Return the gps block to_opensfm_exif() builds for the given stored stddev.
        photo = ODM_Photo("tests/assets/test_0001.jxl")
        photo.gps_xy_stddev = xy_stddev
        photo.gps_z_stddev = z_stddev
        return photo.to_opensfm_exif(gps_accuracy=gps_accuracy)["gps"]

    def test_gps_std_per_axis(self):
        # Horizontal and vertical are passed through independently.
        gps = self._gps_std(0.03, 0.05)
        self.assertAlmostEqual(gps["latitude_std"], 0.03)
        self.assertAlmostEqual(gps["longitude_std"], 0.03)
        self.assertAlmostEqual(gps["altitude_std"], 0.05)
        self.assertAlmostEqual(gps["dop"], 0.05)

    def test_gps_std_equal_axes(self):
        # Equal values stay equal, no vertical multiplier.
        gps = self._gps_std(0.05, 0.05)
        self.assertAlmostEqual(gps["latitude_std"], 0.05)
        self.assertAlmostEqual(gps["altitude_std"], 0.05)

    def test_gps_std_missing_vertical(self):
        # Missing vertical defaults to 3x horizontal.
        gps = self._gps_std(0.03, None)
        self.assertAlmostEqual(gps["latitude_std"], 0.03)
        self.assertAlmostEqual(gps["altitude_std"], 0.09)

    def test_gps_std_from_gps_accuracy(self):
        # No stored values: use gps_accuracy, vertical is 3x.
        gps = self._gps_std(None, None, gps_accuracy=2.0)
        self.assertAlmostEqual(gps["latitude_std"], 2.0)
        self.assertAlmostEqual(gps["altitude_std"], 6.0)

    def test_gps_std_default(self):
        # No stored values and no gps_accuracy: default to 10.0, vertical is 3x.
        gps = self._gps_std(None, None)
        self.assertAlmostEqual(gps["latitude_std"], 10.0)
        self.assertAlmostEqual(gps["altitude_std"], 30.0)

if __name__ == '__main__':
    unittest.main()