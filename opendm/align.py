import os
import shutil
import json
import codem
import dataclasses
import pdal
import numpy as np
from opendm import log

def compute_alignment_matrix(input_laz, align_file, stats_dir):
    if os.path.exists(stats_dir):
        shutil.rmtree(stats_dir)
    os.mkdir(stats_dir)

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

def transform_obj(input_obj, a_matrix, output_obj):
    with open(input_obj, 'r') as fin:
        with open(output_obj, 'w') as fout:
            lines = fin.readlines()
            for line in lines:
                if line.startswith("v "):
                    v = np.fromstring(line.strip()[2:] + " 1",  sep=' ', dtype=float)
                    vt = v.dot(a_matrix)[:3]
                    fout.write("v " + " ".join(map(str, list(vt))) + '\n')
                else:
                    fout.write(line)