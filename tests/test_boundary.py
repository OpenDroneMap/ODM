import unittest

from opendm.boundary import load_boundary


class TestBoundary(unittest.TestCase):
    def setUp(self):
        # A simple WGS84 (EPSG:4326) GeoJSON polygon
        self.boundary = {
            'type': 'Feature',
            'properties': {'name': 'boundary'},
            'geometry': {
                'type': 'Polygon',
                'coordinates': [[
                    [-91.99544, 46.84260],
                    [-91.99417, 46.84260],
                    [-91.99417, 46.84337],
                    [-91.99544, 46.84337],
                    [-91.99544, 46.84260],
                ]]
            }
        }

    def test_load_boundary_without_reprojection(self):
        # Without a target CRS the coordinates are returned unchanged
        coords = load_boundary(self.boundary)
        self.assertEqual(len(coords), 5)
        self.assertAlmostEqual(coords[0][0], -91.99544, places=5)
        self.assertAlmostEqual(coords[0][1], 46.84260, places=5)

    def test_load_boundary_reprojects_to_utm(self):
        # Regression test for https://github.com/OpenDroneMap/ODM/issues/2039
        # On fiona >= 1.10, fiona.crs.to_string() returns "EPSG:4326" instead of a
        # PROJ4 string, which made CRS.from_proj4() raise
        # "CRSError: Invalid PROJ string: EPSG:4326". GeoJSON is always WGS84
        # (RFC 7946), so load_boundary must reproject without raising.
        utm15n = '+proj=utm +zone=15 +datum=WGS84 +units=m +no_defs'
        coords = load_boundary(self.boundary, utm15n)
        self.assertEqual(len(coords), 5)
        # Reprojected coordinates are in meters (UTM), not degrees
        for x, y in coords:
            self.assertGreater(abs(x), 1000)
            self.assertGreater(abs(y), 1000)


if __name__ == '__main__':
    unittest.main()
