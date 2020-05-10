import numpy as np
from sklearn.neighbors import BallTree

EPSILON = 0.00001

def build_grid(bounds, point_cloud, distance):
    """First, a 2D grid is built with a distance of 'distance' between points, inside the given bounds.
       Then, only points that don't have a point cloud neighbour closer than 'distance' are left. The rest are filtered out."""

    # Generate a grid of 2D points inside the bounds, with a distance of 'distance' between them
    grid = __build_grid(bounds, distance)

    # Filter out grid points outside the bounds (makes sense if bounds are not squared)
    grid_inside = bounds.keep_points_inside(grid)

    # Filter out the grid points that have a neighbor  closer than 'distance' from the given point cloud
    return __calculate_lonely_points(grid_inside, point_cloud, distance)

def __build_grid(bounds, distance):
    x_min, x_max, y_min, y_max = bounds.corners()
    grid = [[x, y] for x in np.arange(x_min, x_max + distance, distance) for y in np.arange(y_min, y_max + distance, distance)]
    return np.array(grid)

def __calculate_lonely_points(grid, point_cloud, distance):
    # Generate BallTree for point cloud
    ball_tree = BallTree(point_cloud.get_xy(), metric='manhattan')

    # Calculate for each of the points in the grid, the amount of neighbors in the original ground cloud
    count = ball_tree.query_radius(grid, distance - EPSILON, count_only=True)

    # Return only the points in the grid that don't have a neighbor
    return grid[count == 0]
