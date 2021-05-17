import os
import shutil
import rasterio
import fiona
import numpy as np
import math
import sys
from opendm import log
from opendm import io
from opendm import concurrency 
from opendm import get_image_size
from opendm import system

from skimage.feature import canny
from skimage.draw import line
from skimage.graph import route_through_array
import shapely
from shapely.geometry import LineString, mapping, shape
from shapely.ops import polygonize, unary_union

if sys.platform == 'win32':
    # Temporary fix for: ValueError: GEOSGeom_createLinearRing_r returned a NULL pointer  
    # https://github.com/Toblerity/Shapely/issues/1005
    shapely.speedups.disable()

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

def compute_cutline(orthophoto_file, crop_area_file, destination, max_concurrency=1, scale=1):
    if io.file_exists(orthophoto_file) and io.file_exists(crop_area_file):
        log.ODM_INFO("Computing cutline")

        scale = max(0.0001, min(1, scale))
        scaled_orthophoto = None
        if scale < 1:
            log.ODM_INFO("Scaling orthophoto to %s%% to compute cutline" % (scale * 100))

            scaled_orthophoto = io.related_file_path(orthophoto_file, postfix=".scaled")
            # Scale orthophoto before computing cutline
            system.run("gdal_translate -outsize {}% 0 "
                "-co NUM_THREADS={} "
                "--config GDAL_CACHEMAX {}% "
                '"{}" "{}"'.format(
                scale * 100,
                max_concurrency,
                concurrency.get_max_memory(),
                orthophoto_file,
                scaled_orthophoto
            ))
            
            orthophoto_file = scaled_orthophoto
        
        # open raster
        f =  rasterio.open(orthophoto_file)
        rast = f.read(1) # First band only
        height, width = rast.shape
        number_lines = int(max(8, math.ceil(min(width, height) / 256.0)))
        line_hor_offset = int(width / number_lines)
        line_ver_offset = int(height / number_lines)

        if line_hor_offset <= 2 or line_ver_offset <= 2:
            log.ODM_WARNING("Cannot compute cutline, orthophoto is too small (%sx%spx)" % (width, height))
            return

        crop_f = fiona.open(crop_area_file, 'r')
        if len(crop_f) == 0:
            log.ODM_WARNING("Crop area is empty, cannot compute cutline")
            return

        crop_poly = shape(crop_f[1]['geometry'])
        crop_f.close()

        linestrings = []

        # Compute canny edges on first band
        edges = canny(rast)

        def compute_linestrings(direction):
            log.ODM_INFO("Computing %s cutlines" % direction)
            # Initialize cost map
            cost_map = np.full((height, width), 1, dtype=np.float32)

            # Write edges to cost map
            cost_map[edges==True] = 0 # Low cost

            # Write "barrier, floor is lava" costs
            if direction == 'vertical':
                lines = [((i, 0), (i, height - 1)) for i in range(line_hor_offset, width - line_hor_offset, line_hor_offset)]
                points = []
                pad_x = int(line_hor_offset / 2.0)
                for i in range(0, len(lines)):
                    a,b = lines[i]
                    points.append(((a[0] - pad_x , a[1]), (b[0] - pad_x, b[1])))
                a,b = lines[-1]
                points.append(((a[0] + pad_x , a[1]), (b[0] + pad_x, b[1])))
            else:
                lines = [((0, j), (width - 1, j)) for j in range(line_ver_offset, height - line_ver_offset, line_ver_offset)]
                points = []
                pad_y = int(line_ver_offset / 2.0)
                for i in range(0, len(lines)):
                    a,b = lines[i]
                    points.append(((a[0] , a[1] - pad_y), (b[0], b[1] - pad_y)))
                a,b = lines[-1]
                points.append(((a[0] , a[1] + pad_y), (b[0], b[1] + pad_y)))
                    
            for a, b in lines:
                rr,cc = line(*a, *b)
                cost_map[cc, rr] = 9999 # Lava
            
            # Calculate route
            for a, b in points:
                line_coords, cost = route_through_array(cost_map, (a[1], a[0]), (b[1], b[0]), fully_connected=True, geometric=True)

                # Convert to geographic
                geo_line_coords = [f.xy(*c) for c in line_coords]

                # Simplify
                ls = LineString(geo_line_coords)
                linestrings.append(ls.simplify(0.05, preserve_topology=False))
                
        compute_linestrings('vertical')
        compute_linestrings('horizontal')

                    
        # Generate polygons and keep only those inside the crop area
        log.ODM_INFO("Generating polygons... this could take a bit.")
        polygons = []
        for p in polygonize(unary_union(linestrings)):
            if crop_poly.contains(p):
                polygons.append(p)

        # This should never happen
        if len(polygons) == 0:
            log.ODM_WARNING("No polygons, cannot compute cutline")
            return
        
        log.ODM_INFO("Merging polygons")
        cutline_polygons = unary_union(polygons)
        largest_cutline = cutline_polygons[0]
        max_area = largest_cutline.area
        for p in cutline_polygons:
            if p.area > max_area:
                max_area = p.area
                largest_cutline = p
        
        log.ODM_INFO("Largest cutline found: %s m^2" % max_area)

        meta = {
            'crs': {'init': str(f.crs).lower() },
            'driver': 'GPKG',
            'schema': {
                'properties': {},
                'geometry': 'Polygon'
            }
        }

        # Remove previous
        if os.path.exists(destination):
            os.remove(destination)
        
        with fiona.open(destination, 'w', **meta) as sink:
            sink.write({
                'geometry': mapping(largest_cutline),
                'properties': {}
            })
        f.close()
        log.ODM_INFO("Wrote %s" % destination)

        # Cleanup
        if scaled_orthophoto is not None and os.path.exists(scaled_orthophoto):
            os.remove(scaled_orthophoto)
    else:
        log.ODM_WARNING("We've been asked to compute cutline, but either %s or %s is missing. Skipping..." % (orthophoto_file, crop_area_file))
