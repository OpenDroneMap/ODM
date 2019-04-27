import os
import shutil
from opendm import log
from opendm import io
from opendm import concurrency 


def compute_cutline(orthophoto_file, crop_area_file, destination, max_concurrency=1):
    if io.file_exists(orthophoto_file) and io.file_exists(crop_area_file):
        from opendm.grass_engine import grass
        log.ODM_DEBUG("Computing cutline")

        gctx = grass.create_context({'auto_cleanup' : False})
        gctx.add_param('orthophoto_file', orthophoto_file)
        gctx.add_param('crop_area_file', crop_area_file)
        gctx.add_param('max_concurrency', max_concurrency)
        gctx.add_param('memory', int(concurrency.get_max_memory_mb(300)))
        gctx.set_location(orthophoto_file)

        cutline_file = gctx.execute(os.path.join("opendm", "grass", "compute_cutline.grass"))
        if cutline_file != 'error':
            if io.file_exists(cutline_file):
                shutil.move(cutline_file, destination)
                log.ODM_INFO("Generated cutline file: %s --> %s" % (cutline_file, destination))
                # gctx.cleanup()
                return destination
            else:
                log.ODM_WARNING("Unexpected script result: %s. No cutline file has been generated." % cutline_file)
        else:
            log.ODM_WARNING("Could not generate orthophoto cutline. An error occured when running GRASS. No orthophoto will be generated.")
    else:
        log.ODM_WARNING("We've been asked to compute cutline, but either %s or %s is missing. Skipping..." % (orthophoto_file, crop_area_file))
