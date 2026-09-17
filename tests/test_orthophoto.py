import os
import shutil
import unittest

import numpy as np
import rasterio
from rasterio.transform import Affine

from opendm.orthophoto import _read_window_gated


class TestReadWindowGated(unittest.TestCase):
    """_read_window_gated must be pixel-identical to a plain boundless read.

    It is an optimization that avoids rasterio's boundless VRT path for the
    common in-bounds / fully-out-of-bounds cases, so for every window position
    its result must equal what ds.read(..., boundless=True) would produce.
    """

    WIDTH = 10
    HEIGHT = 8
    COUNT = 4  # 3 bands + alpha, like the orthophotos merge() operates on

    def setUp(self):
        self.tmp = "tests/assets/output"
        if os.path.exists(self.tmp):
            shutil.rmtree(self.tmp)
        os.makedirs(self.tmp)
        self.path = os.path.join(self.tmp, "gated_src.tif")

        # A distinct value per (band, row, col) so a wrong window/offset is visible.
        self.data = np.arange(
            self.COUNT * self.HEIGHT * self.WIDTH, dtype=np.uint8
        ).reshape(self.COUNT, self.HEIGHT, self.WIDTH)

        profile = {
            "driver": "GTiff",
            "width": self.WIDTH,
            "height": self.HEIGHT,
            "count": self.COUNT,
            "dtype": "uint8",
            "nodata": 0,
            "transform": Affine.translation(100.0, 200.0) * Affine.scale(0.5, -0.5),
        }
        with rasterio.open(self.path, "w", **profile) as dst:
            dst.write(self.data)

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def _boundless_reference(self, ds, window):
        (r0, r1), (c0, c1) = window
        shape = (self.COUNT, r1 - r0, c1 - c0)
        out = np.zeros(shape, dtype=np.uint8)
        return ds.read(out=out, window=window, boundless=True, masked=False)

    def _assert_matches_boundless(self, window):
        (r0, r1), (c0, c1) = window
        dst_shape = (self.COUNT, r1 - r0, c1 - c0)
        with rasterio.open(self.path) as ds:
            gated = _read_window_gated(ds, window, dst_shape, np.uint8)
            reference = self._boundless_reference(ds, window)
        self.assertEqual(gated.shape, dst_shape)
        self.assertEqual(gated.dtype, np.uint8)
        np.testing.assert_array_equal(gated, reference)
        return gated

    def test_fully_inside(self):
        gated = self._assert_matches_boundless(((2, 5), (3, 7)))
        # Sanity: the fixture is non-trivial, so a real read is not all zeros.
        self.assertTrue(gated.any())

    def test_fully_inside_full_extent(self):
        self._assert_matches_boundless(((0, self.HEIGHT), (0, self.WIDTH)))

    def test_fully_outside_left(self):
        gated = self._assert_matches_boundless(((0, 3), (-5, -2)))
        self.assertFalse(gated.any())  # nothing overlaps -> all zeros

    def test_fully_outside_right(self):
        self._assert_matches_boundless(((0, 3), (self.WIDTH + 2, self.WIDTH + 5)))

    def test_fully_outside_top(self):
        self._assert_matches_boundless(((-4, -1), (0, 3)))

    def test_fully_outside_bottom(self):
        self._assert_matches_boundless(((self.HEIGHT + 1, self.HEIGHT + 4), (0, 3)))

    def test_partial_top_left_corner(self):
        self._assert_matches_boundless(((-2, 3), (-2, 4)))

    def test_partial_bottom_right_corner(self):
        self._assert_matches_boundless(
            ((self.HEIGHT - 2, self.HEIGHT + 3), (self.WIDTH - 3, self.WIDTH + 2))
        )

    def test_partial_straddles_full_width(self):
        # In-bounds vertically, straddling both left and right edges at once.
        self._assert_matches_boundless(((1, 4), (-2, self.WIDTH + 2)))


if __name__ == "__main__":
    unittest.main()
