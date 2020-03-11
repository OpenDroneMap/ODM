import numpy as np
from numpy.lib.recfunctions import append_fields

class PointCloud:
    """Representation of a 3D point cloud"""
    def __init__(self, xy, z, classification, rgb, indices, extra_dimensions, extra_dimensions_metadata):
        self.xy = xy
        self.z = z
        self.classification = classification
        self.rgb = rgb
        self.indices = indices
        self.extra_dimensions = extra_dimensions
        self.extra_dimensions_metadata = extra_dimensions_metadata

    @staticmethod
    def with_dimensions(x, y, z, classification, red, green, blue, indices=None):
        xy = np.column_stack((x, y))
        rgb = np.column_stack((red, green, blue))
        indices = indices if indices is not None else np.arange(0, len(x))
        return PointCloud(xy, z, classification, rgb, indices, { }, { })

    @staticmethod
    def with_xy(xy):
        [x, y] = np.hsplit(xy, 2)
        empty = np.empty(xy.shape[0])
        return PointCloud.with_dimensions(x.ravel(), y.ravel(), empty, np.empty(xy.shape[0], dtype=np.uint8), empty, empty, empty)

    def __getitem__(self, mask):
        masked_dimensions = { name: values[mask] for name, values in self.extra_dimensions.items() }
        return PointCloud(self.xy[mask], self.z[mask], self.classification[mask], self.rgb[mask], self.indices[mask], masked_dimensions, self.extra_dimensions_metadata)

    def concatenate(self, other_cloud):
        for name, dimension in self.extra_dimensions_metadata.items():
            if name not in other_cloud.extra_dimensions:
                dimension.assign_default(other_cloud)
        for name, dimension in other_cloud.extra_dimensions_metadata.items():
            if name not in self.extra_dimensions:
                dimension.assign_default(self)
        new_indices = np.arange(len(self.indices), len(self.indices) + len(other_cloud.indices))
        self.xy = np.concatenate((self.xy, other_cloud.xy))
        self.z = np.concatenate((self.z, other_cloud.z))
        self.classification = np.concatenate((self.classification, other_cloud.classification))
        self.rgb = np.concatenate((self.rgb, other_cloud.rgb))
        self.indices = np.concatenate((self.indices, new_indices))
        self.extra_dimensions = { name: np.concatenate((values, other_cloud.extra_dimensions[name])) for name, values in self.extra_dimensions.items() }

    def update(self, other_cloud):
        for name, dimension in self.extra_dimensions_metadata.items():
            if name not in other_cloud.extra_dimensions:
                dimension.assign_default(other_cloud)
        for name, dimension in other_cloud.extra_dimensions_metadata.items():
            if name not in self.extra_dimensions:
                dimension.assign_default(self)
        self.xy[other_cloud.indices] = other_cloud.xy
        self.z[other_cloud.indices] = other_cloud.z
        self.classification[other_cloud.indices] = other_cloud.classification
        self.rgb[other_cloud.indices] = other_cloud.rgb
        for name, values in self.extra_dimensions.items():
            values[other_cloud.indices] = other_cloud.extra_dimensions[name]

    def add_dimension(self, dimension, values):
        self.extra_dimensions[dimension.get_name()] = values
        self.extra_dimensions_metadata[dimension.get_name()] = dimension

    def get_xy(self):
        return self.xy

    def get_z(self):
        return self.z

    def len(self):
        return len(self.z)

    def get_extra_dimension_values(self, name):
        return self.extra_dimensions[name]

    def get_bounding_box(self):
        [x_min, y_min] = np.amin(self.xy, axis=0)
        [x_max, y_max] = np.amax(self.xy, axis=0)
        z_min = min(self.z)
        z_max = max(self.z)
        return BoundingBox3D(x_min, x_max, y_min, y_max, z_min, z_max)


class BoundingBox3D:
    def __init__(self, x_min, x_max, y_min, y_max, z_min, z_max):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z_min = z_min
        self.z_max = z_max

    def keep_points_inside(self, point_cloud):
        min = np.array([self.x_min, self.y_min, self.z_min])
        max = np.array([self.x_max, self.y_max, self.z_max])

        arr = np.column_stack((point_cloud.get_xy(), point_cloud.get_z()))
        mask = np.all(np.logical_and(min <= arr, arr <= max), axis=1)

        return point_cloud[mask]
