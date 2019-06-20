import time
import unittest
import os
from opendm.gcp import GCPFile

class TestRemote(unittest.TestCase):
    def setUp(self):
        pass
    
    def test_utm_north(self):
        gcp = GCPFile("tests/assets/gcp_utm_north_valid.txt")
        self.assertTrue(gcp.exists())
        self.assertEqual(gcp.wgs84_utm_zone(), "WGS84 UTM 16N")
    
    def test_epsg(self):
        gcp = GCPFile("tests/assets/gcp_epsg_valid.txt")
        self.assertTrue(gcp.exists())
        self.assertEqual(gcp.wgs84_utm_zone(), "WGS84 UTM 16N")

if __name__ == '__main__':
    unittest.main()