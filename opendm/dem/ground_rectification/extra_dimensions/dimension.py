import numpy as np
from abc import ABCMeta, abstractmethod

class Dimension(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        super(Dimension, self).__init__()

    @abstractmethod
    def assign(self, *point_clouds, **kwargs):
        "Assign a value to the points on the partition"

    @abstractmethod
    def assign_default(self, point_cloud):
        "Assign a default value"

    @abstractmethod
    def get_name(self):
        "Return the name of the dimension"

    @abstractmethod
    def get_las_type(self):
        "Return the type of the values stored"

    def _set_values(self, point_cloud, values):
        point_cloud.add_dimension(self, values)
