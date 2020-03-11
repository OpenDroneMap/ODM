import numpy as np
from scipy.spatial import ConvexHull
from .types import BoxBounds, PolyBounds

def calculate_convex_hull_bounds(points):
    hull = ConvexHull(points)
    return PolyBounds(points[hull.vertices])

def box_from_point_and_size(center, width, height):
    return BoxBounds(center[0] - width / 2, center[0] + width / 2, center[1] - height / 2, center[1] + height / 2)

def box_from_cloud(point_cloud):
    xy = point_cloud.get_xy()
    [x_min, y_min] = np.amin(xy, axis=0)
    [x_max, y_max] = np.amax(xy, axis=0)
    return BoxBounds(x_min, x_max, y_min, y_max)
