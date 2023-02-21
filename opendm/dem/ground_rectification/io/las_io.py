# TODO: Move to pylas when project migrates to python3

import time
import laspy
import pdal
import numpy as np
from opendm import log
from ..point_cloud import PointCloud
import pdb

def read_cloud(point_cloud_path):

    # Open point cloud and read its properties using pdal
    pipeline = pdal.Pipeline('[{"type":"readers.las","filename":"%s"}]' % point_cloud_path)
    cnt = pipeline.execute()

    log.ODM_INFO("pdal arrays: %s" % pipeline.arrays)

    dimensions = pipeline.schema['schema']['dimensions']
    #log.ODM_INFO("Type: %s" % type(pipeline.schema))
    log.ODM_INFO("Dimensions: %s" % dimensions)

    # The x column index is the index of the object with the name 'X'
    x_index = next((index for (index, d) in enumerate(dimensions) if d['name'] == 'X'), None)
    y_index = next((index for (index, d) in enumerate(dimensions) if d['name'] == 'Y'), None)
    z_index = next((index for (index, d) in enumerate(dimensions) if d['name'] == 'Z'), None)
    classification_index = next((index for (index, d) in enumerate(dimensions) if d['name'] == 'Classification'), None)
    red_index = next((index for (index, d) in enumerate(dimensions) if d['name'] == 'Red'), None)
    green_index = next((index for (index, d) in enumerate(dimensions) if d['name'] == 'Green'), None)
    blue_index = next((index for (index, d) in enumerate(dimensions) if d['name'] == 'Blue'), None)

    # Log indices
    log.ODM_INFO("x_index: %s" % x_index)
    log.ODM_INFO("y_index: %s" % y_index)
    log.ODM_INFO("z_index: %s" % z_index)
    log.ODM_INFO("classification_index: %s" % classification_index)
    log.ODM_INFO("red_index: %s" % red_index)
    log.ODM_INFO("green_index: %s" % green_index)
    log.ODM_INFO("blue_index: %s" % blue_index)

    pts = pipeline.arrays[0]
    log.ODM_INFO("pts: %s" % pts)

    x = (pt[x_index] for pt in pts)
    y = (pt[y_index] for pt in pts)
    z = (pt[z_index] for pt in pts)
    classification = (pt[classification_index] for pt in pts)
    red = (pt[red_index] for pt in pts)
    green = (pt[green_index] for pt in pts)
    blue = (pt[blue_index] for pt in pts)

    cloud = PointCloud.with_dimensions(x, y, z, classification, red, green, blue)

    # Return the result
    return pipeline.metadata, cloud

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