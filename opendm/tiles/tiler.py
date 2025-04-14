import os
import sys
import math
from opendm import log
from opendm import system
from opendm import io

def generate_tiles(geotiff, output_dir, max_concurrency, resolution):
    circumference_earth_cm = 2*math.pi*637_813_700
    px_per_tile = 256
    resolution_equator_cm = circumference_earth_cm/px_per_tile
    zoom = math.ceil(math.log(resolution_equator_cm/resolution, 2))

    min_zoom = 5  # 4.89 km/px
    max_zoom = min(zoom, 23)  # No deeper zoom than 23 (1.86 cm/px at equator)

    gdal2tiles = os.path.join(os.path.dirname(__file__), "gdal2tiles.py")
    system.run('%s "%s" --processes %s -z %s-%s -n -w none "%s" "%s"' % (sys.executable, gdal2tiles, max_concurrency, min_zoom, max_zoom, geotiff, output_dir))

def generate_orthophoto_tiles(geotiff, output_dir, max_concurrency, resolution):
    try:
        generate_tiles(geotiff, output_dir, max_concurrency, resolution)
    except Exception as e:
        log.ODM_WARNING("Cannot generate orthophoto tiles: %s" % str(e))

def generate_colored_hillshade(geotiff):
    relief_file = os.path.join(os.path.dirname(__file__), "color_relief.txt")
    hsv_merge_script = os.path.join(os.path.dirname(__file__), "hsv_merge.py")
    colored_dem = io.related_file_path(geotiff, postfix="color")
    hillshade_dem = io.related_file_path(geotiff, postfix="hillshade")
    colored_hillshade_dem = io.related_file_path(geotiff, postfix="colored_hillshade")
    try:
        outputs = [colored_dem, hillshade_dem, colored_hillshade_dem]

        # Cleanup previous
        for f in outputs:
            if os.path.isfile(f):
                os.remove(f)

        system.run('gdaldem color-relief "%s" "%s" "%s" -alpha -co ALPHA=YES' % (geotiff, relief_file, colored_dem))
        system.run('gdaldem hillshade "%s" "%s" -z 1.0 -s 1.0 -az 315.0 -alt 45.0' % (geotiff, hillshade_dem))
        system.run('"%s" "%s" "%s" "%s" "%s"' % (sys.executable, hsv_merge_script, colored_dem, hillshade_dem, colored_hillshade_dem))
        
        return outputs
    except Exception as e:
        log.ODM_WARNING("Cannot generate colored hillshade: %s" % str(e))
        return (None, None, None)

def generate_dem_tiles(geotiff, output_dir, max_concurrency, resolution):
    try:
        colored_dem, hillshade_dem, colored_hillshade_dem = generate_colored_hillshade(geotiff)
        generate_tiles(colored_hillshade_dem, output_dir, max_concurrency, resolution)

        # Cleanup
        for f in [colored_dem, hillshade_dem, colored_hillshade_dem]:
            if os.path.isfile(f):
                os.remove(f)
    except Exception as e:
        log.ODM_WARNING("Cannot generate DEM tiles: %s" % str(e))
