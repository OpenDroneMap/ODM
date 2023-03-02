import argparse
import numpy as np
from os import path
from sklearn.neighbors import BallTree
from sklearn.linear_model import RANSACRegressor
from .extra_dimensions.distance_dimension import DistanceDimension
from .extra_dimensions.partition_dimension import PartitionDimension
from .extra_dimensions.extended_dimension import ExtendedDimension
from .grid.builder import build_grid
from .bounds.utils import calculate_convex_hull_bounds
from .io.las_io import read_cloud, write_cloud
from .partition.selector import select_partition_plan
from .point_cloud import PointCloud

EPSILON = 0.00001

def run_rectification(**kwargs):
    header, point_cloud = read_cloud(kwargs['input'])

    if 'reclassify_plan' in kwargs and kwargs['reclassify_plan'] is not None:
        point_cloud = reclassify_cloud(point_cloud, kwargs['reclassify_plan'], kwargs['reclassify_threshold'], kwargs['min_points'], kwargs['min_area'])

    if 'extend_plan' in kwargs and kwargs['extend_plan'] is not None:
        point_cloud = extend_cloud(point_cloud, kwargs['extend_plan'], kwargs['extend_grid_distance'], kwargs['min_points'], kwargs['min_area'])

    write_cloud(header, point_cloud, kwargs['output'])

def reclassify_cloud(point_cloud, plan, threshold, min_points, min_area):
    # Get only ground
    ground_cloud = point_cloud[point_cloud.classification == 2]

    # Get the partition plan, according to the specified criteria
    partition_plan = select_partition_plan(plan, ground_cloud)

    # Execute the partition plan, and get all the partitions
    partitions = [result for result in partition_plan.execute(min_points=min_points, min_area=min_area)]

    # Add 'distance to ground' and 'partition number' dimensions to the cloud
    for dimension in [DistanceDimension(), PartitionDimension('reclassify_partition')]:

        # Calculate new dimension for partition
        for partition in partitions:
            dimension.assign(partition.point_cloud)

            # Update new data to the original point cloud
            point_cloud.update(partition.point_cloud)

    # Calculate the points that need to be reclassified
    mask = point_cloud.get_extra_dimension_values('distance_to_ground') > threshold

    # Reclassify them as 'unclassified'
    point_cloud.classification[mask] = 1

    return point_cloud

def extend_cloud(point_cloud, plan, distance, min_points, min_area):
    # Get only ground
    ground_cloud = point_cloud[point_cloud.classification == 2]

    # Read the bounds file
    bounds = calculate_convex_hull_bounds(ground_cloud.get_xy())

    # Generate a grid of 2D points inside the bounds, with a distance of 'distance' between them
    grid_2d = build_grid(bounds, ground_cloud, distance)

    # Create a new point cloud
    grid_3d = PointCloud.with_xy(grid_2d)

    # Get the partition plan, according to the specified criteria
    partition_plan = select_partition_plan(plan, ground_cloud)

    # Execute the partition plan, and get all the partitions
    partitions = partition_plan.execute(distance=distance, min_points=min_points, min_area=min_area, bounds=bounds)

    # Create dimensions
    partition_dimension = PartitionDimension('extend_partition')
    extended_dimension = ExtendedDimension()

    for partition in partitions:
        # Keep the grid point that are inside the partition
        grid_inside = partition.bounds.keep_points_inside(grid_3d)

        if grid_inside.len() > 0:
            # In each partition, calculate the altitude of the grid points
            new_points = __calculate_new_points(grid_inside, partition.point_cloud)

            # Assign the dimension values
            partition_dimension.assign(new_points, partition.point_cloud)
            extended_dimension.assign(new_points)

            # Update the original 3d grid with the new calculated points
            grid_3d.update(new_points)

        else:
            # Assign the original points the correct partition
            partition_dimension.assign(partition.point_cloud)

        # Update new information to the original point cloud
        point_cloud.update(partition.point_cloud)


    # Calculate the bounding box of the original cloud
    bbox = point_cloud.get_bounding_box()

    # Remove points that might have ended up outside the bbox
    grid_3d = bbox.keep_points_inside(grid_3d)

    # Add the new grid points to the original cloud
    point_cloud.concatenate(grid_3d)

    # Add the new points to the original point cloud
    return point_cloud

def __calculate_new_points(grid_points_inside, partition_point_cloud):
    # Calculate RANSCAC model
    model = RANSACRegressor().fit(partition_point_cloud.get_xy(), partition_point_cloud.get_z())

    # With the ransac model, calculate the altitude for each grid point
    grid_points_altitude = model.predict(grid_points_inside.get_xy())

    # Calculate color for new points
    [avg_red, avg_green, avg_blue] = np.mean(partition_point_cloud.rgb, axis=0)
    red = np.full(grid_points_inside.len(), avg_red)
    green = np.full(grid_points_inside.len(), avg_green)
    blue = np.full(grid_points_inside.len(), avg_blue)

    # Classify all new points as ground
    classification = np.full(grid_points_inside.len(), 2, dtype=np.uint8)

    # Split xy into columns
    [x, y] = np.hsplit(grid_points_inside.get_xy(), 2)

    # Return point cloud
    return PointCloud.with_dimensions(x.ravel(), y.ravel(), grid_points_altitude, classification, red, green, blue, grid_points_inside.indices)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='This script takes a pre-classified point cloud, and then it re-clasiffies wrongly classified ground point to non-ground points and finally adds ground points where needed.')
    parser.add_argument('input', type=str, help='The path where to find the pre-classified point cloud.')
    parser.add_argument('output', type=str, help='The path where to save the rectified point cloud.')
    parser.add_argument('--reclassify_plan', type=str, help='The partition plan to use reclasiffication. Must be one of(one, uniform, median, surrounding)')
    parser.add_argument('--reclassify_threshold', type=float, help='Every point with a distance to the estimated ground that is higher than the threshold will be reclassified as non ground', default=5)
    parser.add_argument('--extend_plan', type=str, help='The partition plan to use for extending the ground. Must be one of(one, uniform, median, surrounding)')
    parser.add_argument('--extend_grid_distance', type=float, help='The distance between points on the grid that will be added to the point cloud.', default=5)
    parser.add_argument('--min_area', type=int, help='Some partition plans need a minimum area as a stopping criteria.', default=750)
    parser.add_argument('--min_points', type=int, help='Some partition plans need a minimum number of points as a stopping criteria.', default=500)

    args = parser.parse_args()

    if args.reclassify_plan is None and args.extend_plan is None:
        raise Exception("Please set a reclassifying or extension plan. Otherwise there is nothing for me to do.")

    run(input=args.input, reclassify_plan=args.reclassify_plan, reclassify_threshold=args.reclassify_threshold, \
        extend_plan=args.extend_plan, extend_grid_distance=args.extend_grid_distance, \
        output=args.output, min_points=args.min_points, min_area=args.min_area, debug=False)
