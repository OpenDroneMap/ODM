from sklearn.cluster import DBSCAN
from sklearn.neighbors import BallTree
import numpy as np
import math

from ..bounds.utils import box_from_cloud, calculate_convex_hull_bounds
from ..bounds.types import BoxBounds
from ..grid.builder import build_grid
from ..point_cloud import PointCloud
from .partition_plan import PartitionPlan, Partition

DEFAULT_DISTANCE = 5
MIN_PERCENTAGE_OF_POINTS_IN_CONVEX_HULL = 90
EPSILON = 0.0001

class SurroundingPartitions(PartitionPlan):

    def __init__(self, point_cloud):
        super(SurroundingPartitions, self).__init__()
        self.point_cloud = point_cloud
        self.chebyshev_ball_tree = BallTree(point_cloud.xy, metric='chebyshev')
        self.manhattan_ball_tree = BallTree(point_cloud.xy, metric='manhattan')

    def execute(self, **kwargs):
        distance = kwargs['distance'] if 'distance' in kwargs else DEFAULT_DISTANCE
        bounds = kwargs['bounds'] if 'bounds' in kwargs else box_from_cloud(self.point_cloud)
        min_points = kwargs['min_points']
        min_area = kwargs['min_area']

        result = ExecutionResult(self.point_cloud.len())
        grid = build_grid(bounds, self.point_cloud, distance)

        if grid.shape[0] >= 1:
            db = DBSCAN(eps=distance + EPSILON, min_samples=1, metric='manhattan', n_jobs=-1).fit(grid)
            clusters = set(db.labels_)

            for cluster in clusters:
                cluster_members = grid[db.labels_ == cluster]
                point_cloud_neighbors, point_cloud_neighbors_mask = self.__find_cluster_neighbors(cluster_members, distance)

                if self.__is_cluster_surrounded(cluster_members, point_cloud_neighbors):
                    result.add_cluster_partition(cluster_members, point_cloud_neighbors, point_cloud_neighbors_mask)
                else:
                    point_cloud_neighbors, point_cloud_neighbors_mask, bounding_box = self.__find_points_for_non_surrounded_cluster(bounds, cluster_members, distance, min_area, min_points)
                    result.add_zone_partition(cluster_members, point_cloud_neighbors, point_cloud_neighbors_mask, bounding_box)

        return result.build_result(self.point_cloud)

    def __find_points_for_non_surrounded_cluster(self, bounds, cluster_members, distance, min_area, min_points):
        (center_x, center_y) = bounds.center()

        [x_min, y_min] = np.amin(cluster_members, axis=0)
        [x_max, y_max] = np.amax(cluster_members, axis=0)

        x = [x_min - distance, x_max + distance]
        y = [y_min - distance, y_max + distance]

        # Find the indices of the corner closest to the center of the point cloud
        closest_x_idx = np.argmin(np.abs(x - center_x))
        closest_y_idx = np.argmin(np.abs(y - center_y))

        # Calculate the direction to where the box should grow
        x_dir = -1 if closest_x_idx == 0 else 1
        y_dir = -1 if closest_y_idx == 0 else 1

        bounding_box = BoxBounds(x[0], x[1], y[0], y[1])
        while bounding_box.area() < min_area:
            x[closest_x_idx] += distance * x_dir
            y[closest_y_idx] += distance * y_dir
            bounding_box = BoxBounds(x[0], x[1], y[0], y[1])

        mask = bounding_box.calculate_mask(self.point_cloud)
        while len(mask) < min_points:
            x[closest_x_idx] += distance * x_dir
            y[closest_y_idx] += distance * y_dir
            bounding_box = BoxBounds(x[0], x[1], y[0], y[1])
            mask = bounding_box.calculate_mask(self.point_cloud)

        return self.point_cloud[mask], mask, bounding_box

    def __is_cluster_surrounded(self, cluster_members, point_cloud_neighbors):
        convex_hull = calculate_convex_hull_bounds(point_cloud_neighbors.get_xy())
        ratio = convex_hull.percentage_of_points_inside(cluster_members)
        return ratio > MIN_PERCENTAGE_OF_POINTS_IN_CONVEX_HULL

    def __find_cluster_neighbors(self, cluster_members, distance):
        mask_per_point = self.manhattan_ball_tree.query_radius(cluster_members, distance * 3)
        all_neighbor_mask = np.concatenate(mask_per_point)
        point_cloud_neighbors = self.point_cloud[all_neighbor_mask]
        return point_cloud_neighbors, all_neighbor_mask

class ExecutionResult:
    def __init__(self, cloud_size):
        self.partitions = [ ]
        self.marked_as_neighbors = np.zeros(cloud_size, dtype=bool)

    def add_cluster_partition(self, cluster_members, point_cloud_neighbors, point_cloud_neighbors_mask):
        convex_hull = calculate_convex_hull_bounds(np.concatenate((point_cloud_neighbors.get_xy(), cluster_members)))
        self.marked_as_neighbors[point_cloud_neighbors_mask] = True
        self.partitions.append(Partition(point_cloud_neighbors, bounds=convex_hull))

    def add_zone_partition(self, cluster_members, point_cloud_neighbors, point_cloud_neighbors_mask, bounding_box):
        self.marked_as_neighbors[point_cloud_neighbors_mask] = True
        self.partitions.append(Partition(point_cloud_neighbors, bounds=bounding_box))

    def build_result(self, whole_point_cloud):
        remaining_cloud = whole_point_cloud[~self.marked_as_neighbors]
        new_bounds = box_from_cloud(remaining_cloud)
        self.partitions.insert(0, Partition(remaining_cloud, bounds=new_bounds))
        return self.partitions
