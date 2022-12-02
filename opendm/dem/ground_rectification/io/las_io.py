# TODO: Move to pylas when project migrates to python3

import laspy
import numpy as np
from ..point_cloud import PointCloud

def read_cloud(point_cloud_path):
    # Open point cloud and read its properties
    las_file = laspy.read(point_cloud_path)
    header = las_file.header

    x = las_file.x.scaled_array()
    y = las_file.y.scaled_array()
    z = las_file.z.scaled_array()

    cloud = PointCloud.with_dimensions(x, y, z, las_file.classification.array, las_file.red, las_file.green, las_file.blue)

    # Return the result
    return header, cloud

def write_cloud(header, point_cloud, output_point_cloud_path, write_extra_dimensions=False):
    # Open output file
    output_las_file = laspy.LasData(header)

    if write_extra_dimensions:
        extra_dims = [laspy.ExtraBytesParams(name=name, type=dimension.get_las_type(), description="Dimension added by Ground Extend") for name, dimension in point_cloud.extra_dimensions_metadata.items()]
        output_las_file.add_extra_dims(extra_dims)
        # Assign dimension values
        for dimension_name, values in point_cloud.extra_dimensions.items():
            setattr(output_las_file, dimension_name, values)

    # Adapt points to scale and offset
    [x, y] = np.hsplit(point_cloud.xy, 2)
    output_las_file.x = x.ravel()
    output_las_file.y = y.ravel()
    output_las_file.z = point_cloud.z

    # Set color
    [red, green, blue] = np.hsplit(point_cloud.rgb, 3)
    output_las_file.red = red.ravel()
    output_las_file.green = green.ravel()
    output_las_file.blue = blue.ravel()

    # Set classification
    output_las_file.classification = point_cloud.classification.astype(np.uint8)

    output_las_file.write(output_point_cloud_path)