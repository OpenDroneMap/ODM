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
    pipeline.execute()

    metadata = pipeline.metadata
    arrays = pipeline.arrays

    # Extract point coordinates, classification, and RGB values
    x = arrays[0]["X"]
    y = arrays[0]["Y"]
    z = arrays[0]["Z"]
    classification = arrays[0]["Classification"].astype(np.uint8)
    red = arrays[0]["Red"]
    green = arrays[0]["Green"]
    blue = arrays[0]["Blue"]

    # Create PointCloud object
    cloud = PointCloud.with_dimensions(x, y, z, classification, red, green, blue)

    # Return the result
    return metadata, cloud

def write_cloud(metadata, point_cloud, output_point_cloud_path, write_extra_dimensions=False):

    # Create PDAL pipeline to write point cloud
    pipeline = pdal.Pipeline('[{"type": "writers.las","filename": "%s","compression": "laszip","extra_dims": %s}]' %
                             (output_point_cloud_path, str(write_extra_dimensions).lower()))

    # Adapt points to scale and offset
    [x, y] = np.hsplit(point_cloud.xy, 2)
    z = point_cloud.z

    # Set color
    [red, green, blue] = np.hsplit(point_cloud.rgb, 3)
    # Set classification
    classification = point_cloud.classification.astype(np.uint8)

    # Write point cloud with PDAL
    pipeline.execute(np.column_stack((x, y, z, red, green, blue, classification)))
