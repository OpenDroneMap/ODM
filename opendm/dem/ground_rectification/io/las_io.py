# TODO: Move to pylas when project migrates to python3

from laspy.file import File
from laspy.header import Header
import numpy as np
from ..point_cloud import PointCloud

def read_cloud(point_cloud_path):
    # Open point cloud and read its properties
    las_file = File(point_cloud_path, mode='r')
    header = (las_file.header.copy(), las_file.header.scale, las_file.header.offset,las_file.header.evlrs, las_file.header.vlrs)
    [x_scale, y_scale, z_scale] = las_file.header.scale
    [x_offset, y_offset, z_offset] = las_file.header.offset

    # Calculate the real coordinates
    x = las_file.X * x_scale + x_offset
    y = las_file.Y * y_scale + y_offset
    z = las_file.Z * z_scale + z_offset

    cloud = PointCloud.with_dimensions(x, y, z, las_file.Classification, las_file.red, las_file.green, las_file.blue)

    # Close the file
    las_file.close()

    # Return the result
    return header, cloud

def write_cloud(header, point_cloud, output_point_cloud_path, write_extra_dimensions=False):
    (h, scale, offset, evlrs, vlrs) = header

    # Open output file
    output_las_file = File(output_point_cloud_path, mode='w', header=h, evlrs=evlrs, vlrs=vlrs)

    if write_extra_dimensions:
        # Create new dimensions
        for name, dimension in point_cloud.extra_dimensions_metadata.items():
            output_las_file.define_new_dimension(name=name, data_type=dimension.get_las_type(), description="Dimension added by Ground Extend")

        # Assign dimension values
        for dimension_name, values in point_cloud.extra_dimensions.items():
            setattr(output_las_file, dimension_name, values)

    # Adapt points to scale and offset
    [x_scale, y_scale, z_scale] = scale
    [x_offset, y_offset, z_offset] = offset
    [x, y] = np.hsplit(point_cloud.xy, 2)
    output_las_file.X = (x.ravel() - x_offset) / x_scale
    output_las_file.Y = (y.ravel() - y_offset) / y_scale
    output_las_file.Z = (point_cloud.z - z_offset) / z_scale

    # Set color
    [red, green, blue] = np.hsplit(point_cloud.rgb, 3)
    output_las_file.red = red.ravel()
    output_las_file.green = green.ravel()
    output_las_file.blue = blue.ravel()

    # Set classification
    output_las_file.Classification = point_cloud.classification.astype(np.uint8)

    # Set header
    output_las_file.header.scale = scale
    output_las_file.header.offset = offset

    # Close files
    output_las_file.close()
