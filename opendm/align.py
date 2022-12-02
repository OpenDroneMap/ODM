import os
import shutil
import codem
import dataclasses
from opendm import log

def compute_alignment_matrix(input_laz, align_file, tmp_dir):
    if os.path.exists(tmp_dir):
        shutil.rmtree(tmp_dir)
    os.mkdir(tmp_dir)

    conf = dataclasses.asdict(codem.CodemRunConfig(align_file, input_laz, MIN_RESOLUTION=0.2, OUTPUT_DIR=tmp_dir)) # TODO: how to compute this
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

    return app_reg.get_registration_transformation()
    # if os.path.exists(tmp_dir):
    #     shutil.rmtree(tmp_dir)