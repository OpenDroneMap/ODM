import time
import unittest
import os
import shutil

from opendm import camera


class TestCamera(unittest.TestCase):
    def setUp(self):
        if os.path.exists("tests/assets/output"):
            shutil.rmtree("tests/assets/output")
        os.makedirs("tests/assets/output")

    def test_camera(self):
        c = camera.get_cameras_from_opensfm("tests/assets/reconstruction.json")
        self.assertEqual(len(c.keys()), 1)
        camera_id = list(c.keys())[0]
        self.assertTrue('v2 ' not in camera_id)

        self.assertRaises(RuntimeError, camera.get_cameras_from_opensfm, 'tests/assets/nonexistent.json')
        self.assertRaises(ValueError, camera.get_cameras_from_opensfm, 'tests/assets/gcp_extras.txt')
        self.assertFalse('k1_prior' in c[camera_id])
        
        # Add bogus field
        c[camera_id]['test'] = 0

        osfm_c = camera.get_opensfm_camera_models(c)
        self.assertEqual(len(osfm_c.keys()), 1)
        c1 = osfm_c[list(osfm_c.keys())[0]]
        self.assertTrue('k1_prior' in c1)
        self.assertTrue('k2_prior' in c1)
        self.assertFalse('test' in c1)
        self.assertEqual(c1['k1'], c1['k1_prior'])
        self.assertEqual(c1['k2'], c1['k2_prior'])
        self.assertEqual(c1['focal'], c1['focal_prior'])
        self.assertTrue('width_prior' not in c1)
        

if __name__ == '__main__':
    unittest.main()