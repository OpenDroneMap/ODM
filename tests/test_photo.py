import math
import unittest
import os
import shutil

import numpy as np

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

    def _camera_from_ypr(self, yaw, pitch, roll, latitude, longitude):
        # Camera rotation OpenSfM builds from the OPK priors ODM computes.
        from opensfm import geometry
        photo = ODM_Photo("tests/assets/test_0001.jxl")
        photo.yaw, photo.pitch, photo.roll = yaw, pitch, roll
        photo.latitude, photo.longitude, photo.altitude = latitude, longitude, 500.0
        photo.compute_opk()
        R = geometry.rotation_from_opk(*[math.radians(a) for a in (photo.omega, photo.phi, photo.kappa)])
        # world (topocentric ENU) directions of the optical axis and of "image up"
        return (photo.omega, photo.phi, photo.kappa), R.T.dot([0, 0, 1]), R.T.dot([0, -1, 0])

    def test_opk_does_not_depend_on_location(self):
        # Same attitude must give the same OPK anywhere on Earth.
        a, _, _ = self._camera_from_ypr(31.0, 20.0, 0.0, 33.69, 73.07)
        b, _, _ = self._camera_from_ypr(31.0, 20.0, 0.0, -41.29, 174.78)
        c, _, _ = self._camera_from_ypr(31.0, 20.0, 0.0, 44.06, -121.32)
        np.testing.assert_allclose(a, b, atol=1e-6)
        np.testing.assert_allclose(a, c, atol=1e-6)

    def test_opk_nadir_camera_looks_down_with_top_towards_yaw(self):
        for yaw in (0.0, 90.0, 200.0, -31.4):
            _, axis, up = self._camera_from_ypr(yaw, 0.0, 0.0, 33.69, 73.07)
            np.testing.assert_allclose(axis, [0, 0, -1], atol=1e-6)
            bearing = math.degrees(math.atan2(up[0], up[1]))
            self.assertAlmostEqual((bearing - yaw + 180) % 360 - 180, 0.0, places=4)

    def test_opk_oblique_camera_tilts_towards_yaw(self):
        # pitch 30 = camera 60 degrees below the horizon (DJI gimbal -60)
        _, axis, _ = self._camera_from_ypr(45.0, 30.0, 0.0, 33.69, 73.07)
        self.assertAlmostEqual(math.degrees(math.asin(-axis[2])), 60.0, places=4)
        self.assertAlmostEqual(math.degrees(math.atan2(axis[0], axis[1])), 45.0, places=4)

if __name__ == '__main__':
    unittest.main()