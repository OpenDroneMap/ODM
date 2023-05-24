import time
from opendm.dem.ground_rectification.extra_dimensions.userdata_dimension import UserDataDimension
import pdal
import numpy as np
from opendm import log
from ..point_cloud import PointCloud
import pdb
import json

def read_cloud(point_cloud_path):
    pipeline = pdal.Pipeline('[{"type":"readers.las","filename":"%s"}]' % point_cloud_path)
    pipeline.execute()

    arrays = pipeline.arrays[0]

    # Extract point coordinates, classification, and RGB values
    x = arrays["X"]
    y = arrays["Y"]
    z = arrays["Z"]
    classification = arrays["Classification"].astype(np.uint8)
    red = arrays["Red"]
    green = arrays["Green"]
    blue = arrays["Blue"]

    cloud = PointCloud.with_dimensions(x, y, z, classification, red, green, blue)

    if "UserData" in arrays.dtype.names:
        cloud.add_dimension(UserDataDimension(), arrays["UserData"])

    return pipeline.metadata["metadata"]["readers.las"], cloud


def safe_add_metadata(pipeline, metadata, key, sourcekey=None):
    k = key if sourcekey is None else sourcekey
    if k in metadata:
        pipeline["pipeline"][0][key] = metadata[k]


def write_cloud(metadata, point_cloud, output_point_cloud_path):

    # Adapt points to scale and offset
    x, y = np.hsplit(point_cloud.xy, 2)

    red, green, blue = np.hsplit(point_cloud.rgb, 3)

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
    arrays['X'] = x.ravel()
    arrays['Y'] = y.ravel()
    arrays['Z'] = point_cloud.z
    arrays['Classification'] = point_cloud.classification.astype(np.uint8).ravel()
    arrays['Red'] = red.astype(np.uint8).ravel()
    arrays['Green'] = green.astype(np.uint8).ravel()
    arrays['Blue'] = blue.astype(np.uint8).ravel()

    if "UserData" in point_cloud.extra_dimensions:
        arrays['UserData'] = point_cloud.extra_dimensions["UserData"].ravel()

    writer_pipeline = {
        "pipeline": [
            {
                "type": "writers.las",
                "filename": output_point_cloud_path,
                "compression": "lazperf",
                "extra_dims": "all"
            }
        ]
    }

    safe_add_metadata(writer_pipeline, metadata, "scale_x")
    safe_add_metadata(writer_pipeline, metadata, "scale_y")
    safe_add_metadata(writer_pipeline, metadata, "scale_z")
    safe_add_metadata(writer_pipeline, metadata, "offset_x")
    safe_add_metadata(writer_pipeline, metadata, "offset_y")
    safe_add_metadata(writer_pipeline, metadata, "offset_z")
    safe_add_metadata(writer_pipeline, metadata, "a_srs", "spatialreference")
    safe_add_metadata(writer_pipeline, metadata, "dataformat_id")
    safe_add_metadata(writer_pipeline, metadata, "system_id")
    safe_add_metadata(writer_pipeline, metadata, "software_id")
    safe_add_metadata(writer_pipeline, metadata, "creation_doy")
    safe_add_metadata(writer_pipeline, metadata, "creation_year")
    safe_add_metadata(writer_pipeline, metadata, "minor_version")
    safe_add_metadata(writer_pipeline, metadata, "major_version")
    safe_add_metadata(writer_pipeline, metadata, "file_source_id")
    safe_add_metadata(writer_pipeline, metadata, "global_encoding")

    # The metadata object contains the VLRs as fields called "vlr_N" where N is the index of the VLR
    # We have to copy them over to the writer pipeline as a list of dictionaries in the "vlrs" field
    writer_pipeline["pipeline"][0]["vlrs"] = []

    i = 0
    while True:
        vlr_field = "vlr_%d" % i
        if vlr_field in metadata:
            vlr = metadata[vlr_field]
            writer_pipeline["pipeline"][0]["vlrs"].append({
                "record_id": vlr["record_id"],
                "user_id": vlr["user_id"],
                "description": vlr["description"],
                "data": vlr["data"]
            })
            i += 1
        else:
            break

    pipeline = pdal.Pipeline(json.dumps(writer_pipeline), arrays=[arrays])
    pipeline.execute()
