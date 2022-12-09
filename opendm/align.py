import os
import shutil
import json
import codem
import dataclasses
import pdal
import numpy as np
from rasterio.crs import CRS
from opendm import log
from opendm import io

def get_point_cloud_crs(file):
    pipeline = pdal.Pipeline(json.dumps([ file ]))
    metadata = pipeline.quickinfo

    reader_metadata = [val for key, val in metadata.items() if "readers" in key]
    crs = CRS.from_string(reader_metadata[0]["srs"]["horizontal"])
    return str(crs)

def reproject_point_cloud(file, out_srs):
    log.ODM_INFO("Reprojecting %s to %s" % (file, out_srs))
    out_file = io.related_file_path(file, postfix="_reprojected")
    pipeline = pdal.Pipeline(json.dumps([ file, {
        "type": "filters.reprojection",
        "out_srs": out_srs
    }, out_file]))
    pipeline.execute()
    return out_file

def compute_alignment_matrix(input_laz, align_file, stats_dir):
    if os.path.exists(stats_dir):
        shutil.rmtree(stats_dir)
    os.mkdir(stats_dir)

    # Check if we need to reproject align file
    input_crs = get_point_cloud_crs(input_laz)
    log.ODM_INFO("Input CRS: %s" % input_crs)

    _, ext = os.path.splitext(align_file)
    if ext.lower() in [".tif"]:
        pass #TODO
    elif ext.lower() in [".las", ".laz"]:
        align_crs = get_point_cloud_crs(align_file)
        log.ODM_INFO("Align CRS: %s" % align_crs)
        if input_crs != get_point_cloud_crs(align_file):
            # Reprojection needed
            align_file = reproject_point_cloud(align_file, input_crs)

    conf = dataclasses.asdict(codem.CodemRunConfig(align_file, input_laz, OUTPUT_DIR=stats_dir))
    fnd_obj, aoi_obj = codem.preprocess(conf)
    fnd_obj.prep()
    aoi_obj.prep()
    log.ODM_INFO("Aligning reconstruction to %s" % align_file)
    log.ODM_INFO("Coarse registration...")
    dsm_reg = codem.coarse_registration(fnd_obj, aoi_obj, conf)
    log.ODM_INFO("Fine registration...")
    icp_reg = codem.fine_registration(fnd_obj, aoi_obj, dsm_reg, conf)

    app_reg = codem.registration.ApplyRegistration(
        fnd_obj,
        aoi_obj,
        icp_reg.registration_parameters,
        icp_reg.residual_vectors,
        icp_reg.residual_origins,
        conf,
        None,
    )

    reg = app_reg.get_registration_transformation()
    # print(dsm_reg.registration_parameters)
    # print(icp_reg.registration_parameters)
    matrix = np.fromstring(reg['matrix'], dtype=float, sep=' ').reshape((4, 4))
    return matrix

def transform_point_cloud(input_laz, a_matrix, output_laz):
    pipe = [
        input_laz,
        {
            'type': 'filters.transformation',
            'matrix': " ".join(list(map(str, a_matrix.flatten()))),
        },
        output_laz,
    ]
    p = pdal.Pipeline(json.dumps(pipe))
    p.execute()

def transform_obj(input_obj, a_matrix, geo_offset, output_obj):
    g_off = np.array([geo_offset[0], geo_offset[1], 0, 0])

    with open(input_obj, 'r') as fin:
        with open(output_obj, 'w') as fout:
            lines = fin.readlines()
            for line in lines:
                if line.startswith("v "):
                    v = np.fromstring(line.strip()[2:] + " 1",  sep=' ', dtype=float)
                    vt = (a_matrix.dot((v + g_off)) - g_off)[:3]
                    fout.write("v " + " ".join(map(str, list(vt))) + '\n')
                else:
                    fout.write(line)