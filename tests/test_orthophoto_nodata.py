import os
import shutil
import tempfile
import unittest

import numpy as np
import rasterio
from rasterio.enums import ColorInterp
from rasterio.transform import from_origin

from opendm.orthophoto import set_float_orthophoto_nodata, FLOAT_ORTHOPHOTO_NODATA


def write_ortho(path, data, alpha, descriptions, dtype):
    count = data.shape[0] + 1
    profile = {
        'driver': 'GTiff', 'width': data.shape[2], 'height': data.shape[1], 'count': count,
        'dtype': dtype, 'crs': 'EPSG:32633', 'transform': from_origin(500000, 5000000, 0.05, 0.05),
        'tiled': True, 'blockxsize': 16, 'blockysize': 16, 'compress': 'deflate'
    }
    with rasterio.open(path, 'w', **profile) as dst:
        # Must be set before writing compressed data, otherwise it's not persisted
        dst.colorinterp = [ColorInterp.gray] * (count - 1) + [ColorInterp.alpha]
        dst.write(np.concatenate([data, alpha[np.newaxis]]).astype(dtype))
        for i, d in enumerate(descriptions):
            dst.set_band_description(i + 1, d)


class TestOrthophotoNoData(unittest.TestCase):
    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.ortho = os.path.join(self.tmpdir, 'odm_orthophoto.tif')

        h, w = 40, 50  # not a multiple of the block size
        self.alpha = np.zeros((h, w), dtype=np.float32)
        self.alpha[5:35, 10:45] = 255
        self.data = np.zeros((4, h, w), dtype=np.float32)
        self.data[:, self.alpha == 255] = 0.25
        self.data[0, 5, 10] = -0.03   # negative reflectance inside (Red)
        self.data[3, 34, 44] = -1.4   # negative reflectance inside (RedEdge)
        self.data[1, 0, 0] = 0.5      # stray value outside of the valid area

    def tearDown(self):
        shutil.rmtree(self.tmpdir)

    def test_float_orthophoto(self):
        write_ortho(self.ortho, self.data, self.alpha, ['Red', 'Green', 'NIR', 'RedEdge'], 'float32')
        stats = set_float_orthophoto_nodata(self.ortho)

        self.assertEqual(stats['nodata_pixels'], int(np.count_nonzero(self.alpha == 0)))
        self.assertEqual(stats['clamped_values'], 2)
        self.assertEqual(stats['changed_values'], 3)

        with rasterio.open(self.ortho) as src:
            self.assertEqual(src.nodata, FLOAT_ORTHOPHOTO_NODATA)
            self.assertEqual(src.descriptions[:4], ('Red', 'Green', 'NIR', 'RedEdge'))
            self.assertEqual(src.colorinterp[-1], ColorInterp.alpha)
            arr = src.read()

        empty = self.alpha == 0
        self.assertTrue(np.all(arr[:4, empty] == FLOAT_ORTHOPHOTO_NODATA))
        self.assertTrue(np.all(arr[4] == self.alpha))  # alpha untouched
        self.assertEqual(arr[0, 5, 10], 0)
        self.assertEqual(arr[3, 34, 44], 0)
        self.assertTrue(np.all(arr[:4, ~empty] >= 0))
        self.assertAlmostEqual(float(arr[1, 20, 20]), 0.25)

        # NDVI over valid pixels is now within [-1, 1]
        # (clamped values are 0 == NoData, GIS band math skips them)
        with np.errstate(divide='ignore', invalid='ignore'):
            valid = ~empty & (arr[0] != FLOAT_ORTHOPHOTO_NODATA) & (arr[2] != FLOAT_ORTHOPHOTO_NODATA)
            r, n = arr[0, valid].astype(np.float64), arr[2, valid].astype(np.float64)
            ndvi = (n - r) / (n + r)
        self.assertTrue(np.all(np.isfinite(ndvi)))
        self.assertTrue(np.all((ndvi >= -1) & (ndvi <= 1)))

        # Idempotent
        stats2 = set_float_orthophoto_nodata(self.ortho)
        self.assertEqual(stats2['clamped_values'], 0)
        self.assertEqual(stats2['changed_values'], 0)
        with rasterio.open(self.ortho) as src:
            self.assertEqual(src.nodata, FLOAT_ORTHOPHOTO_NODATA)
            self.assertTrue(np.array_equal(src.read(), arr))

        # No leftover temporary files
        self.assertEqual(os.listdir(self.tmpdir), ['odm_orthophoto.tif'])

    def test_thermal_band_not_clamped(self):
        data = self.data[:1].copy()
        data[0, 10, 20] = -12.5  # valid temperature (C)
        write_ortho(self.ortho, data, self.alpha, ['LWIR'], 'float32')
        stats = set_float_orthophoto_nodata(self.ortho)
        self.assertEqual(stats['clamped_values'], 0)
        with rasterio.open(self.ortho) as src:
            self.assertEqual(src.nodata, FLOAT_ORTHOPHOTO_NODATA)
            arr = src.read(1)
        self.assertEqual(arr[10, 20], -12.5)
        self.assertTrue(np.all(arr[self.alpha == 0] == FLOAT_ORTHOPHOTO_NODATA))

    def test_integer_orthophotos_untouched(self):
        for dtype in ['uint8', 'uint16']:
            data = np.zeros((3, 40, 50), dtype=dtype)
            data[:, self.alpha == 255] = 100
            write_ortho(self.ortho, data, self.alpha.astype(dtype), ['', '', ''], dtype)
            with open(self.ortho, 'rb') as f:
                before = f.read()

            self.assertIsNone(set_float_orthophoto_nodata(self.ortho))

            with open(self.ortho, 'rb') as f:
                self.assertEqual(f.read(), before)
            with rasterio.open(self.ortho) as src:
                self.assertIsNone(src.nodata)


if __name__ == '__main__':
    unittest.main()
