import os
import shutil
from opendm import log
from opendm import io
from opendm import concurrency 
from opendm import get_image_size
from opendm import system
import math

def compute_cutline(orthophoto_file, crop_area_file, destination, max_concurrency=1, tmpdir=None, scale=1):
    if io.file_exists(orthophoto_file) and io.file_exists(crop_area_file):
        from opendm.grass_engine import grass
        log.ODM_INFO("Computing cutline")

        if tmpdir and not io.dir_exists(tmpdir):
            system.mkdir_p(tmpdir)

        scale = max(0.0001, min(1, scale))
        scaled_orthophoto = None

        if scale < 1:
            log.ODM_INFO("Scaling orthophoto to %s%% to compute cutline" % (scale * 100))

            scaled_orthophoto = os.path.join(tmpdir, os.path.basename(io.related_file_path(orthophoto_file, postfix=".scaled")))
            # Scale orthophoto before computing cutline
            system.run("gdal_translate -outsize {}% 0 "
                "-co NUM_THREADS={} "
                "--config GDAL_CACHEMAX {}% "
                "{} {}".format(
                scale * 100,
                max_concurrency,
                concurrency.get_max_memory(),
                orthophoto_file,
                scaled_orthophoto
            ))
            orthophoto_file = scaled_orthophoto

        try:
            ortho_width,ortho_height = get_image_size.get_image_size(orthophoto_file, fallback_on_error=False)
            log.ODM_INFO("Orthophoto dimensions are %sx%s" % (ortho_width, ortho_height))
            number_lines = int(max(8, math.ceil(min(ortho_width, ortho_height) / 256.0)))
        except:
            log.ODM_INFO("Cannot compute orthophoto dimensions, setting arbitrary number of lines.")
            number_lines = 32
        
        log.ODM_INFO("Number of lines: %s" % number_lines)

        gctx = grass.create_context({'auto_cleanup' : False, 'tmpdir': tmpdir})
        gctx.add_param('orthophoto_file', orthophoto_file)
        gctx.add_param('crop_area_file', crop_area_file)
        gctx.add_param('number_lines', number_lines)
        gctx.add_param('max_concurrency', max_concurrency)
        gctx.add_param('memory', int(concurrency.get_max_memory_mb(300)))
        gctx.set_location(orthophoto_file)

        cutline_file = gctx.execute(os.path.join("opendm", "grass", "compute_cutline.grass"))
        if cutline_file != 'error':
            if io.file_exists(cutline_file):
                shutil.move(cutline_file, destination)
                log.ODM_INFO("Generated cutline file: %s --> %s" % (cutline_file, destination))
                gctx.cleanup()
                return destination
            else:
                log.ODM_WARNING("Unexpected script result: %s. No cutline file has been generated." % cutline_file)
        else:
            log.ODM_WARNING("Could not generate orthophoto cutline. An error occured when running GRASS. No orthophoto will be generated.")
    else:
        log.ODM_WARNING("We've been asked to compute cutline, but either %s or %s is missing. Skipping..." % (orthophoto_file, crop_area_file))
