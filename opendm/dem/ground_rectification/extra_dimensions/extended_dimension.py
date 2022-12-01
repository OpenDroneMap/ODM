import numpy as np
from .dimension import Dimension

class ExtendedDimension(Dimension):
    """Whether the point was added or was already on the original point cloud"""

    def __init__(self):
        super(ExtendedDimension, self).__init__()

    def assign_default(self, point_cloud):
        default = np.full(point_cloud.len(), 0, dtype=np.uint16)
        super(ExtendedDimension, self)._set_values(point_cloud, default)

    def assign(self, *point_clouds, **kwargs):
        for point_cloud in point_clouds:
            added = np.full(point_cloud.len(), 1, dtype=np.uint16)
            super(ExtendedDimension, self)._set_values(point_cloud, added)

    def get_name(self):
        return 'extended'

    def get_las_type(self):
        return 3
