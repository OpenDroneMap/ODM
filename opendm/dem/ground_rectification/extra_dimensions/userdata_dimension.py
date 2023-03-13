import numpy as np
from .dimension import Dimension

class UserDataDimension(Dimension):
    """A dimension that stores the user data of a point cloud."""

    def __init__(self):
        super(UserDataDimension, self).__init__()

    def assign_default(self, point_cloud):
        default = np.full(point_cloud.len(), 0, dtype=np.uint8)
        super(UserDataDimension, self)._set_values(point_cloud, default)

    def assign(self, *point_clouds, **kwargs):

        # Simply copy the value of the UserData dimension from the original point cloud
        # to the new point cloud
        for point_cloud in point_clouds:
            super(UserDataDimension, self)._set_values(point_cloud, point_cloud.user_data)

    def get_name(self):
        return 'UserData'

    def get_las_type(self):
        return 'uint8'
