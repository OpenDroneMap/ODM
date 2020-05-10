from .partition_plan import PartitionPlan, Partition
from ..bounds.utils import box_from_cloud

class OnePartition(PartitionPlan):
    """This partition plan does nothing. It returns all the cloud points in one partition."""

    def __init__(self, point_cloud):
        super(OnePartition, self).__init__()
        self.point_cloud = point_cloud

    def execute(self, **kwargs):
        bounds = box_from_cloud(self.point_cloud)
        return [Partition(self.point_cloud, bounds=bounds)]
