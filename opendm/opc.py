import os
from opendm.ai import get_model
from opendm import log
from opendm.system import run
from opendm import io

def classify(point_cloud, max_threads=8):
    tmp_output = io.related_file_path(point_cloud, postfix=".classified")
    if os.path.isfile(tmp_output):
        os.remove(tmp_output)

    try:
        model = get_model("openpointclass", 
            "https://github.com/uav4geo/OpenPointClass/releases/download/v1.1.3/vehicles-vegetation-buildings.zip", 
            "v1.0.0",
            name="model.bin")

        if model is not None:
            run('pcclassify "%s" "%s" "%s" -u -s 2,64' % (point_cloud, tmp_output, model), env_vars={'OMP_NUM_THREADS': max_threads})
            
            if os.path.isfile(tmp_output):
                os.remove(point_cloud)
                os.rename(tmp_output, point_cloud)
            else:
                log.ODM_WARNING("Cannot classify using OpenPointClass (no output generated)")
        else:
            log.ODM_WARNING("Cannot download/access model from %s" % (model_url))

    except Exception as e:
        log.ODM_WARNING("Cannot classify using OpenPointClass: %s" % str(e))

