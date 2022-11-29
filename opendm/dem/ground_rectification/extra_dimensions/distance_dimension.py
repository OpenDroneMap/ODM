import numpy as np
from sklearn.linear_model import RANSACRegressor
from .dimension import Dimension

class DistanceDimension(Dimension):
    """Assign each point the distance to the estimated ground"""

    def __init__(self):
        super(DistanceDimension, self).__init__()

    def assign_default(self, point_cloud):
        default = np.full(point_cloud.len(), -1)
        super(DistanceDimension, self)._set_values(point_cloud, default)

    def assign(self, *point_clouds, **kwargs):
        for point_cloud in point_clouds:
            xy = point_cloud.get_xy()

            # Calculate RANSCAC model
            model = RANSACRegressor().fit(xy, point_cloud.get_z())

            # Calculate angle between estimated plane and XY plane
            angle = self.__calculate_angle(model)
            if angle >= 45:
                # If the angle is higher than 45 degrees, then don't calculate the difference, since it will probably be way off
                diff = np.full(point_cloud.len(), 0)
            else:
                predicted = model.predict(xy)
                diff = point_cloud.get_z() - predicted
                # Ignore the diff when the diff is below the ground
                diff[diff < 0] = 0
            super(DistanceDimension, self)._set_values(point_cloud, diff)

    def get_name(self):
        return 'distance_to_ground'

    def get_las_type(self):
        return 'float64'

    def __calculate_angle(self, model):
        "Calculate the angle between the estimated plane and the XY plane"
        a = model.estimator_.coef_[0]
        b = model.estimator_.coef_[1]
        angle = np.arccos(1 / np.sqrt(a ** 2 + b ** 2 + 1))
        return np.degrees(angle)
