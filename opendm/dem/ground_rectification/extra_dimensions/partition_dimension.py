import numpy as np
from .dimension import Dimension

class PartitionDimension(Dimension):
    """Group points by partition"""

    def __init__(self, name):
        super(PartitionDimension, self).__init__()
        self.counter = 1
        self.name = name

    def assign_default(self, point_cloud):
        default = np.full(point_cloud.len(), 0)
        super(PartitionDimension, self)._set_values(point_cloud, default)

    def assign(self, *point_clouds, **kwargs):
        for point_cloud in point_clouds:
            super(PartitionDimension, self)._set_values(point_cloud, np.full(point_cloud.len(), self.counter))
        self.counter += 1

    def get_name(self):
        return self.name

    def get_las_type(self):
        return 'uint32'
