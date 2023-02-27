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

    arrays = pipeline.arrays[0]

    # print arrays shape
    log.ODM_INFO(str(arrays.shape))
    log.ODM_INFO(str(arrays.dtype))
    log.ODM_INFO(str(arrays))
    log.ODM_INFO(str(arrays[0]))

    # Extract point coordinates, classification, and RGB values
    x = arrays["X"]
    y = arrays["Y"]
    z = arrays["Z"]
    classification = arrays["Classification"].astype(np.uint8)
    red = arrays["Red"]
    green = arrays["Green"]
    blue = arrays["Blue"]

    # Create PointCloud object
    cloud = PointCloud.with_dimensions(x, y, z, classification, red, green, blue)

    # Return the result
    return pipeline.metadata, cloud


def write_cloud(metadata, point_cloud, output_point_cloud_path, write_extra_dimensions=False):

    # Adapt points to scale and offset
    x, y = np.hsplit(point_cloud.xy, 2)
    z = point_cloud.z

    # Set color
    red, green, blue = np.hsplit(point_cloud.rgb, 3)

    # Set classification
    classification = point_cloud.classification.astype(np.uint8)

    # Print array dimensions
    x = x.ravel()
    y = y.ravel()
    classification = classification.ravel()
    red = red.astype(np.uint8).ravel()
    green = green.astype(np.uint8).ravel()
    blue = blue.astype(np.uint8).ravel()

    arrays = np.zeros(len(x),
                      dtype=[('X', '<f8'),
                             ('Y', '<f8'),
                             ('Z', '<f8'),
                             ('Intensity', '<u2'),
                             ('ReturnNumber', 'u1'),
                             ('NumberOfReturns', 'u1'),
                             ('ScanDirectionFlag', 'u1'),
                             ('EdgeOfFlightLine', 'u1'),
                             ('Classification', 'u1'),
                             ('ScanAngleRank', '<f4'),
                             ('UserData', 'u1'),
                             ('PointSourceId', '<u2'),
                             ('GpsTime', '<f8'),
                             ('Red', '<u2'),
                             ('Green', '<u2'),
                             ('Blue', '<u2')])
    arrays['X'] = x
    arrays['Y'] = y
    arrays['Z'] = z
    arrays['Classification'] = classification
    arrays['Red'] = red
    arrays['Green'] = green
    arrays['Blue'] = blue

    #test_data = np.array(
    #        [(x, y, z) for x, y, z in zip(x_vals, y_vals, z_vals)],
    #        dtype=[("X", float), ("Y", float), ("Z", float)],
    #    )

    log.ODM_INFO("arrays: %s" % str(arrays.shape))
    log.ODM_INFO("arrays: %s" % arrays)
    log.ODM_INFO("arrays: %s" % arrays[0])
    log.ODM_INFO("Write extra dimensions: %s" % write_extra_dimensions)

    # Create PDAL pipeline to write point cloud
    #pipeline = pdal.Pipeline('[{"type": "writers.las","filename": "%s","compression": "laszip", "extra_dims": %s}]' %
    #                         (output_point_cloud_path, str(write_extra_dimensions).lower()), arrays=[arrays])

    pipeline = pdal.Pipeline('[{"type": "writers.las","filename": "%s","compression": "laszip"}]' % output_point_cloud_path, arrays=[arrays])

    log.ODM_INFO("Dest path: %s" % output_point_cloud_path)

    # Write point cloud with PDAL
    pipeline.execute()
