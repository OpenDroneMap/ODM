from abc import ABCMeta, abstractmethod

class PartitionPlan(object):
    """We want to partition the ground in different areas. There are many ways to do so, and each of them will be a different partition plan."""
    __metaclass__ = ABCMeta

    def __init__(self):
        super(PartitionPlan, self).__init__()

    @abstractmethod
    def execute(self):
        """This method is expected to return a list of Partition instances"""

class Partition:
    def __init__(self, point_cloud, **kwargs):
        self.point_cloud = point_cloud
        self.bounds = kwargs['bounds']
