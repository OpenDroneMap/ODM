import os
import shutil
import rasterio
import numpy as np
import math
from opendm import log
from opendm import io
from opendm import concurrency 
from opendm import get_image_size
from opendm import system

from skimage.feature import canny
from skimage.draw import line
from skimage.graph import route_through_array

def write_raster(data, file):
    profile = {
        'driver': 'GTiff',
        'width': data.shape[1],
        'height': data.shape[0],
        'count': 1,
        'dtype': 'float32',
        'transform': None,
        'nodata': None,
        'crs': None
    }

    with rasterio.open(file, 'w', **profile) as wout:
        wout.write(data, 1)

def compute_cutline(orthophoto_file, crop_area_file, destination, max_concurrency=1, tmpdir=None, scale=1):
    if io.file_exists(orthophoto_file) and io.file_exists(crop_area_file):
        log.ODM_INFO("Computing cutline")
        # if tmpdir and not io.dir_exists(tmpdir):
        #     system.mkdir_p(tmpdir)
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
        
        # open raster
        with rasterio.open(orthophoto_file) as f:
            rast = f.read()
            _, height, width = rast.shape
            number_lines = int(max(8, math.ceil(min(width, height) / 256.0)))
            print("num lines: %s" % number_lines)
            line_hor_offset = int(width / number_lines)
            line_ver_offset = int(height / number_lines)

            if line_hor_offset <= 2 or line_ver_offset <= 2:
                log.ODM_WARNING("Cannot compute cutline, orthophoto is too small (%sx%spx)" % (width, height))
                return

            # Compute canny edges on first band
            edges = canny(rast[0])

            # Initialize cost map
            cost_map = np.full((height, width), 1, dtype=np.float32)

            # Write edges to cost map
            cost_map[edges==True] = 0 # Low cost

            # Write vertical "barrier, floor is lava" costs
            vertical_lines = [((i, 0), (i, height - 1)) for i in range(line_hor_offset, width - line_hor_offset, line_hor_offset)]
            vertical_points = []
            pad_x = int(line_hor_offset / 2.0)

            for i in range(0, len(vertical_lines)):
                a,b = vertical_lines[i]
                vertical_points.append(((a[0] - pad_x , a[1]), (b[0] - pad_x, b[1])))
            a,b = vertical_lines[-1]
            vertical_points.append(((a[0] + pad_x , a[1]), (b[0] + pad_x, b[1])))
                    
            for a, b in vertical_lines:
                rr,cc = line(*a, *b)
                cost_map[cc, rr] = 9999 # Lava
            
            # Calculate route
            for a, b in vertical_points:
                line_coords, cost = route_through_array(cost_map, (a[1], a[0]), (b[1], b[0]), fully_connected=True, geometric=True)

                print(a, b)
                print("Cost: %s" % cost)
                for p in line_coords:
                    cost_map[p[0], p[1]] = 5000

            write_raster(cost_map, '/datasets/brighton2/cc_cost_map.tif')
            
    else:
        log.ODM_WARNING("We've been asked to compute cutline, but either %s or %s is missing. Skipping..." % (orthophoto_file, crop_area_file))
# def compute_cutline(orthophoto_file, crop_area_file, destination, max_concurrency=1, tmpdir=None, scale=1):
#     if io.file_exists(orthophoto_file) and io.file_exists(crop_area_file):
#         from opendm.grass_engine import grass
#         log.ODM_INFO("Computing cutline")

#         if tmpdir and not io.dir_exists(tmpdir):
#             system.mkdir_p(tmpdir)

#         scale = max(0.0001, min(1, scale))
#         scaled_orthophoto = None

#         if scale < 1:
#             log.ODM_INFO("Scaling orthophoto to %s%% to compute cutline" % (scale * 100))

#             scaled_orthophoto = os.path.join(tmpdir, os.path.basename(io.related_file_path(orthophoto_file, postfix=".scaled")))
#             # Scale orthophoto before computing cutline
#             system.run("gdal_translate -outsize {}% 0 "
#                 "-co NUM_THREADS={} "
#                 "--config GDAL_CACHEMAX {}% "
#                 "{} {}".format(
#                 scale * 100,
#                 max_concurrency,
#                 concurrency.get_max_memory(),
#                 orthophoto_file,
#                 scaled_orthophoto
#             ))
#             orthophoto_file = scaled_orthophoto

#         try:
#             ortho_width,ortho_height = get_image_size.get_image_size(orthophoto_file, fallback_on_error=False)
#             log.ODM_INFO("Orthophoto dimensions are %sx%s" % (ortho_width, ortho_height))
#             number_lines = int(max(8, math.ceil(min(ortho_width, ortho_height) / 256.0)))
#         except:
#             log.ODM_INFO("Cannot compute orthophoto dimensions, setting arbitrary number of lines.")
#             number_lines = 32
        
#         log.ODM_INFO("Number of lines: %s" % number_lines)

#         gctx = grass.create_context({'auto_cleanup' : False, 'tmpdir': tmpdir})
#         gctx.add_param('orthophoto_file', orthophoto_file)
#         gctx.add_param('crop_area_file', crop_area_file)
#         gctx.add_param('number_lines', number_lines)
#         gctx.add_param('max_concurrency', max_concurrency)
#         gctx.add_param('memory', int(concurrency.get_max_memory_mb(300)))
#         gctx.set_location(orthophoto_file)

#         cutline_file = gctx.execute(os.path.join("opendm", "grass", "compute_cutline.grass"))
#         if cutline_file != 'error':
#             if io.file_exists(cutline_file):
#                 shutil.move(cutline_file, destination)
#                 log.ODM_INFO("Generated cutline file: %s --> %s" % (cutline_file, destination))
#                 gctx.cleanup()
#                 return destination
#             else:
#                 log.ODM_WARNING("Unexpected script result: %s. No cutline file has been generated." % cutline_file)
#         else:
#             log.ODM_WARNING("Could not generate orthophoto cutline. An error occured when running GRASS. No orthophoto will be generated.")
#     else:
#         log.ODM_WARNING("We've been asked to compute cutline, but either %s or %s is missing. Skipping..." % (orthophoto_file, crop_area_file))
