import time
import unittest
import os
import shutil

from opendm.gcp import GCPFile


class TestGcp(unittest.TestCase):
    def setUp(self):
        if os.path.exists("tests/assets/output"):
            shutil.rmtree("tests/assets/output")
        os.makedirs("tests/assets/output")
        # pass

    def test_utm_north(self):
        gcp = GCPFile("tests/assets/gcp_utm_north_valid.txt")
        self.assertTrue(gcp.exists())
        self.assertEqual(gcp.wgs84_utm_zone(), "WGS84 UTM 16N")
    
    def test_latlon_south(self):
        gcp = GCPFile("tests/assets/gcp_latlon_south.txt")
        self.assertTrue(gcp.exists())
        self.assertEqual(gcp.wgs84_utm_zone(), "WGS84 UTM 48S")
    
    def test_latlon(self):
        gcp = GCPFile("tests/assets/gcp_latlon_valid.txt")
        self.assertTrue(gcp.exists())
        self.assertEqual(gcp.wgs84_utm_zone(), "WGS84 UTM 16N")

    def test_utm_conversion(self):
        gcp = GCPFile("tests/assets/gcp_latlon_valid.txt")
        copy = GCPFile(gcp.create_utm_copy("tests/assets/output/gcp_utm.txt"))
        self.assertTrue(copy.exists())
        self.assertEqual(copy.raw_srs, "WGS84 UTM 16N")
        self.assertEqual(copy.get_entry(0).x, 609865.7077054137)
        self.assertEqual(copy.get_entry(0).y, 4950688.361817497)
    
    def test_utm_conversion_feet(self):
        gcp = GCPFile("tests/assets/gcp_michigan_feet_valid.txt")
        copy = GCPFile(gcp.create_utm_copy("tests/assets/output/gcp_utm_z.txt"))
        self.assertTrue(copy.exists())
        self.assertEqual(copy.raw_srs, "WGS84 UTM 16N")
        self.assertEqual(round(copy.get_entry(0).x, 3), 609925.818)
        self.assertEqual(round(copy.get_entry(0).y, 3), 4950688.772)
        self.assertEqual(round(copy.get_entry(0).z, 3), 563.199)

    def test_filtered_copy(self):
        gcp = GCPFile('tests/assets/gcp_latlon_valid.txt')
        self.assertTrue(gcp.exists())
        self.assertEqual(gcp.entries_count(), 2)
        copy = GCPFile(gcp.make_filtered_copy('tests/assets/output/filtered_copy.txt', 'tests/assets/images', min_images=1))
        self.assertTrue(copy.exists())
        self.assertEqual(copy.entries_count(), 1)

    def test_null_gcp(self):
        gcp = GCPFile(None)
        self.assertFalse(gcp.exists())

    def test_gcp_extras(self):
        gcp = GCPFile('tests/assets/gcp_extras.txt')
        self.assertEqual(gcp.get_entry(0).extras, 'gcp1')

        copy = GCPFile(gcp.create_utm_copy("tests/assets/output/gcp_utm_no_extras.txt", include_extras=False))
        self.assertTrue(copy.exists())
        self.assertEqual(copy.get_entry(0).extras, '')

if __name__ == '__main__':
    unittest.main()