import numpy as np
from abc import abstractmethod
from ..bounds.utils import box_from_cloud
from .partition_plan import PartitionPlan, Partition

class QuadPartitions(PartitionPlan):
    """This partition plan starts with one big partition that includes the whole point cloud. It then divides it into four partitions, based on some criteria.
       Each of these partitions are then divided into four other partitions and so on. The algorithm has two possible stopping criterias:
       if subdividing a partition would imply that one of the new partitions contains fewer that a given amount of points, or that one of the new partitions as an area smaller that the given size,
       then the partition is not divided."""

    def __init__(self, point_cloud):
        super(QuadPartitions, self).__init__()
        self.point_cloud = point_cloud

    @abstractmethod
    def choose_divide_point(self, point_cloud, bounding_box):
        """Given a point cloud and a bounding box, calculate the point that will be used to divide the partition by four"""

    def execute(self, **kwargs):
        initial_bounding_box = box_from_cloud(self.point_cloud)
        return self._divide_until(self.point_cloud, initial_bounding_box, kwargs['min_points'], kwargs['min_area'])

    def _divide_until(self, point_cloud, bounding_box, min_points, min_area):
        dividing_point = self.choose_divide_point(point_cloud, bounding_box)
        new_boxes = bounding_box.divide_by_point(dividing_point)

        for new_box in new_boxes:
            if new_box.area() < min_area:
                return [Partition(point_cloud, bounds=bounding_box)] # If by dividing, I break the minimum area threshold, don't do it

        subdivisions = []

        for new_box in new_boxes:
            mask = new_box.calculate_mask(point_cloud)
            if np.count_nonzero(mask) < min_points:
                return [Partition(point_cloud, bounds=bounding_box)] # If by dividing, I break the minimum amount of points in a zone, don't do it

            subdivisions += self._divide_until(point_cloud[mask], new_box, min_points, min_area)

        return subdivisions

class UniformPartitions(QuadPartitions):
    """This kind of partitioner takes the current bounding box, and divides it by four uniform partitions"""

    def __init__(self, point_cloud):
        super(UniformPartitions, self).__init__(point_cloud)

    def choose_divide_point(self, point_cloud, bounding_box):
        return bounding_box.center()

class MedianPartitions(QuadPartitions):
    """This kind of partitioner takes the current point cloud, and divides it by the median, so that all four new partitions have the same amount of points"""

    def __init__(self, point_cloud):
        super(MedianPartitions, self).__init__(point_cloud)

    def choose_divide_point(self, point_cloud, bounding_box):
        return np.median(point_cloud.get_xy(), axis=0)
