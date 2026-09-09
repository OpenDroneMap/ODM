import numpy as np
from skimage import measure


def region_stats(image, region, saturation_threshold=None):
    """
    Compute statistics for an image over a polygonal region.

    :param image: 2D numpy array (single band)
    :param region: list/array of (x, y) coordinate tuples describing the polygon
    :param saturation_threshold: optional value above which a pixel is considered saturated
    :return: (mean, std, num_pixels, saturated_fraction)
    """
    region = np.asarray(region, dtype=float)

    # skimage uses (row, col) ordering, image coordinates are (x, y)
    rev_pts = np.fliplr(region)
    h, w = image.shape[:2]
    mask = measure.grid_points_in_poly((h, w), rev_pts)

    num_pixels = int(mask.sum())
    if num_pixels == 0:
        return None, None, 0, 0.0

    pixels = image[mask]
    mean_value = float(pixels.mean())
    stdev = float(pixels.std())

    saturated_fraction = 0.0
    if saturation_threshold is not None:
        saturated_fraction = float((pixels > saturation_threshold).sum()) / num_pixels

    return mean_value, stdev, num_pixels, saturated_fraction
