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
        # "CRSError: Invalid PROJ string: EPSG:4326". load_boundary must reproject
        # a WGS84 GeoJSON boundary without raising.
        utm15n = '+proj=utm +zone=15 +datum=WGS84 +units=m +no_defs'
        coords = load_boundary(self.boundary, utm15n)
        self.assertEqual(len(coords), 5)
        # Reprojected coordinates are in meters (UTM), not degrees
        for x, y in coords:
            self.assertGreater(abs(x), 1000)
            self.assertGreater(abs(y), 1000)

    def test_load_boundary_honors_non_wgs84_source_crs(self):
        # GeoJSON may declare a CRS other than WGS84 (allowed by RFC 7946 and
        # written in practice by tools like QGIS). load_boundary must reproject
        # from the file's declared CRS, not assume EPSG:4326.
        boundary_utm = {
            'type': 'FeatureCollection',
            'crs': {'type': 'name',
                    'properties': {'name': 'urn:ogc:def:crs:EPSG::32635'}},
            'features': [{
                'type': 'Feature',
                'properties': {},
                'geometry': {
                    'type': 'Polygon',
                    'coordinates': [[
                        [500000, 4540000],
                        [501000, 4540000],
                        [501000, 4541000],
                        [500000, 4541000],
                        [500000, 4540000],
                    ]]
                }
            }]
        }
        wgs84 = '+proj=longlat +datum=WGS84 +no_defs'
        coords = load_boundary(boundary_utm, wgs84)
        self.assertEqual(len(coords), 5)
        # EPSG:32635 (UTM 35N) easting 500000 / northing ~4540000 is ~27E, ~41N.
        # If the source CRS were wrongly assumed to be WGS84, these would not land here.
        for lon, lat in coords:
            self.assertTrue(26 < lon < 28, "unexpected lon: %s" % lon)
            self.assertTrue(40 < lat < 42, "unexpected lat: %s" % lat)


if __name__ == '__main__':
    unittest.main()
