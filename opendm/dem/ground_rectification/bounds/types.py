import numpy as np
from scipy.spatial import Delaunay
from ..point_cloud import PointCloud

EPSILON = 0.00001

class PolyBounds(object):
    def __init__(self, points):
        self.__points = points
        self.__delaunay = Delaunay(points)
        [x_min, y_min] = np.amin(points, axis=0)
        [x_max, y_max] = np.amax(points, axis=0)
        self._corners = (x_min, x_max, y_min, y_max)

    def keep_points_inside(self, point_cloud):
        """Return a new point cloud with the points from the given cloud that are inside the bounds"""
        mask = self.calculate_mask(point_cloud)
        return point_cloud[mask]

    def percentage_of_points_inside(self, points):
        if isinstance(points, PointCloud):
            points = points.get_xy()
        mask = self.calculate_mask(points)
        return np.count_nonzero(mask) * 100 / points.shape[0]

    def calculate_mask(self, points):
        """Calculate the mask that would filter out the points outside the bounds"""
        if isinstance(points, PointCloud):
            points = points.get_xy()
        return self.__delaunay.find_simplex(points) >= 0

    def center(self):
        (x_min, x_max, y_min, y_max) = self._corners
        return ((x_min + x_max) / 2, (y_min + y_max) / 2)

    def corners(self):
        return self._corners

class BoxBounds(object):
    def __init__(self, x_min, x_max, y_min, y_max):
        self._corners = (x_min, x_max, y_min, y_max)

    def keep_points_inside(self, point_cloud):
        """Return a new point cloud with the points from the given cloud that are inside the bounds"""
        mask = self.calculate_mask(point_cloud)
        return point_cloud[mask]

    def percentage_of_points_inside(self, points):
        if isinstance(points, PointCloud):
            points = points.get_xy()
        mask = self.calculate_mask(points)
        return np.count_nonzero(mask) * 100 / points.shape[0]

    def calculate_mask(self, points):
        """Calculate the mask that would filter out the points outside the bounds"""
        if isinstance(points, PointCloud):
            points = points.get_xy()
        (x_min, x_max, y_min, y_max) = self._corners
        min = np.array([x_min, y_min])
        max = np.array([x_max, y_max])

        return np.all(np.logical_and(min <= points, points <= max), axis=1)

    def center(self):
        (x_min, x_max, y_min, y_max) = self._corners
        return ((x_min + x_max) / 2, (y_min + y_max) / 2)

    def corners(self):
        return self._corners

    def area(self):
        (x_min, x_max, y_min, y_max) = self._corners
        return (x_max - x_min) *  (y_max - y_min)

    def divide_by_point(self, point):
        """Divide the box into four boxes, marked by the point. It is assumed that the point is inside the box"""
        [x_point, y_point] = point
        (x_min, x_max, y_min, y_max) = self._corners
        return [
            BoxBounds(x_min, x_point,           y_min, y_point),
            BoxBounds(x_point + EPSILON, x_max, y_min, y_point),
            BoxBounds(x_min, x_point,           y_point + EPSILON, y_max),
            BoxBounds(x_point + EPSILON, x_max, y_point + EPSILON, y_max)
        ]
