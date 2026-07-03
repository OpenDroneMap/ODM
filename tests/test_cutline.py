import os
import tempfile
import unittest

import fiona
from shapely.geometry import Polygon, mapping

from opendm.cutline import largest_polygon


class TestCutline(unittest.TestCase):
    def test_largest_polygon_from_single_polygon(self):
        """ Tests that the largest polygon from a single polygon is the polygon itself """

        p = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)]) # 1x1 square
        largest = largest_polygon([p])

        self.assertEqual(largest.geom_type, "Polygon")
        self.assertAlmostEqual(largest.area, 1.0)

    def test_largest_polygon_from_multipolygon_union(self):
        """ Tests that the largest polygon from a multipolygon union is the largest polygon """

        small = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)]) # 1x1 square (area = 1)
        large = Polygon([(0, 0), (3, 0), (3, 2), (0, 2)]) # 3x2 rectangle (area = 6)
        largest = largest_polygon([small, large])
        self.assertEqual(largest.geom_type, "Polygon")
        self.assertAlmostEqual(largest.area, 6.0) # matches large polygon

    def test_polygon_writes_to_gpkg_with_polygon_schema(self):
        """ Tests that fiona can write our polygon to a GPKG file"""

        geom = largest_polygon([
            Polygon([(0, 0), (1, 0), (1, 1), (0, 1)]),
            Polygon([(2, 0), (5, 0), (5, 2), (2, 2)]),
        ])
        self.assertEqual(mapping(geom)["type"], "Polygon")

        # Write out the polygon to quickly verify that Fiona can still process
        # our largest polygon.
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "cutline.gpkg")
            with fiona.open(
                path,
                "w",
                driver="GPKG",
                crs="EPSG:4326",
                schema={"geometry": "Polygon", "properties": {}},
            ) as sink:
                sink.write({"geometry": mapping(geom), "properties": {}})

            with fiona.open(path) as src:
                feature = next(iter(src))
                self.assertEqual(feature["geometry"]["type"], "Polygon")


if __name__ == "__main__":
    unittest.main()
