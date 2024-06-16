import os
import sys
import rasterio
import numpy
import math
import time
import shutil
import glob
import re
from joblib import delayed, Parallel
from opendm.system import run
from opendm import point_cloud
from opendm import io
from opendm import system
from opendm.concurrency import get_max_memory, parallel_map, get_total_memory
from datetime import datetime
from opendm.vendor.gdal_fillnodata import main as gdal_fillnodata
from opendm import log

from .ground_rectification.rectify import run_rectification
from . import pdal

try:
    # GDAL >= 3.3
    from osgeo_utils.gdal_proximity import main as gdal_proximity
except ModuleNotFoundError:
    # GDAL <= 3.2
    try:
        from osgeo.utils.gdal_proximity import main as gdal_proximity
    except:
        pass

def classify(lasFile, scalar, slope, threshold, window):
    start = datetime.now()

    try:
        pdal.run_pdaltranslate_smrf(lasFile, lasFile, scalar, slope, threshold, window)
    except:
        log.ODM_WARNING("Error creating classified file %s" % lasFile)

    log.ODM_INFO('Created %s in %s' % (lasFile, datetime.now() - start))
    return lasFile

def rectify(lasFile, reclassify_threshold=5, min_area=750, min_points=500):
    start = datetime.now()

    try:

        log.ODM_INFO("Rectifying {} using with [reclassify threshold: {}, min area: {}, min points: {}]".format(lasFile, reclassify_threshold, min_area, min_points))
        run_rectification(
            input=lasFile, output=lasFile, \
            reclassify_plan='median', reclassify_threshold=reclassify_threshold, \
            extend_plan='surrounding', extend_grid_distance=5, \
            min_area=min_area, min_points=min_points)

        log.ODM_INFO('Created %s in %s' % (lasFile, datetime.now() - start))
    except Exception as e:
        log.ODM_WARNING("Error rectifying ground in file %s: %s" % (lasFile, str(e)))

    return lasFile

error = None

def create_dem(input_point_cloud, dem_type, output_type='max', radiuses=['0.56'], gapfill=True,
                outdir='', resolution=0.1, max_workers=1, max_tile_size=4096,
                decimation=None, with_euclidean_map=False,
                apply_smoothing=True, max_tiles=None):
    """ Create DEM from multiple radii, and optionally gapfill """
    
    start = datetime.now()
    kwargs = {
        'input': input_point_cloud,
        'outdir': outdir,
        'outputType': output_type,
        'radiuses': ",".join(map(str, radiuses)),
        'resolution': resolution,
        'maxTiles': 0 if max_tiles is None else max_tiles,
        'decimation': 1 if decimation is None else decimation,
        'classification': 2 if dem_type == 'dtm' else -1,
        'tileSize': max_tile_size
    }
    system.run('renderdem "{input}" '
                '--outdir "{outdir}" '
                '--output-type {outputType} '
                '--radiuses {radiuses} '
                '--resolution {resolution} '
                '--max-tiles {maxTiles} '
                '--decimation {decimation} '
                '--classification {classification} '
                '--tile-size {tileSize} '
                '--force '.format(**kwargs), env_vars={'OMP_NUM_THREADS': max_workers})

    output_file = "%s.tif" % dem_type
    output_path = os.path.abspath(os.path.join(outdir, output_file))

    # Fetch tiles
    tiles = []
    for p in glob.glob(os.path.join(os.path.abspath(outdir), "*.tif")):
        filename = os.path.basename(p)
        m = re.match("^r([\d\.]+)_x\d+_y\d+\.tif", filename)
        if m is not None:
            tiles.append({'filename': p, 'radius': float(m.group(1))})

    if len(tiles) == 0:
        raise system.ExitException("No DEM tiles were generated, something went wrong")

    log.ODM_INFO("Generated %s tiles" % len(tiles))

    # Sort tiles by decreasing radius
    tiles.sort(key=lambda t: float(t['radius']), reverse=True)

    # Create virtual raster
    tiles_vrt_path = os.path.abspath(os.path.join(outdir, "tiles.vrt"))
    tiles_file_list = os.path.abspath(os.path.join(outdir, "tiles_list.txt"))
    with open(tiles_file_list, 'w') as f:
        for t in tiles:
            f.write(t['filename'] + '\n')

    run('gdalbuildvrt -input_file_list "%s" "%s" ' % (tiles_file_list, tiles_vrt_path))

    merged_vrt_path = os.path.abspath(os.path.join(outdir, "merged.vrt"))
    geotiff_small_path = os.path.abspath(os.path.join(outdir, 'tiles.small.tif'))
    geotiff_small_filled_path = os.path.abspath(os.path.join(outdir, 'tiles.small_filled.tif'))
    geotiff_path = os.path.abspath(os.path.join(outdir, 'tiles.tif'))

    # Build GeoTIFF
    kwargs = {
        'max_memory': get_max_memory(),
        'threads': max_workers if max_workers else 'ALL_CPUS',
        'tiles_vrt': tiles_vrt_path,
        'merged_vrt': merged_vrt_path,
        'geotiff': geotiff_path,
        'geotiff_small': geotiff_small_path,
        'geotiff_small_filled': geotiff_small_filled_path
    }

    if gapfill:
        # Sometimes, for some reason gdal_fillnodata.py
        # behaves strangely when reading data directly from a .VRT
        # so we need to convert to GeoTIFF first.
        # Scale to 10% size
        run('gdal_translate '
                '-co NUM_THREADS={threads} '
                '-co BIGTIFF=IF_SAFER '
                '-co COMPRESS=DEFLATE '
                '--config GDAL_CACHEMAX {max_memory}% '
                '-outsize 10% 0 '
                '"{tiles_vrt}" "{geotiff_small}"'.format(**kwargs))

        # Fill scaled
        gdal_fillnodata(['.', 
                        '-co', 'NUM_THREADS=%s' % kwargs['threads'], 
                        '-co', 'BIGTIFF=IF_SAFER',
                        '-co', 'COMPRESS=DEFLATE',
                        '--config', 'GDAL_CACHE_MAX', str(kwargs['max_memory']) + '%',
                        '-b', '1',
                        '-of', 'GTiff',
                        kwargs['geotiff_small'], kwargs['geotiff_small_filled']])
        
        # Merge filled scaled DEM with unfilled DEM using bilinear interpolation
        run('gdalbuildvrt -resolution highest -r bilinear "%s" "%s" "%s"' % (merged_vrt_path, geotiff_small_filled_path, tiles_vrt_path))
        run('gdal_translate '
            '-co NUM_THREADS={threads} '
            '-co TILED=YES '
            '-co BIGTIFF=IF_SAFER '
            '-co COMPRESS=DEFLATE '
            '--config GDAL_CACHEMAX {max_memory}% '
            '"{merged_vrt}" "{geotiff}"'.format(**kwargs))
    else:
        run('gdal_translate '
                '-co NUM_THREADS={threads} '
                '-co TILED=YES '
                '-co BIGTIFF=IF_SAFER '
                '-co COMPRESS=DEFLATE '
                '--config GDAL_CACHEMAX {max_memory}% '
                '"{tiles_vrt}" "{geotiff}"'.format(**kwargs))

    if apply_smoothing:
        median_smoothing(geotiff_path, output_path, num_workers=max_workers)
        os.remove(geotiff_path)
    else:
        os.replace(geotiff_path, output_path)

    if os.path.exists(tiles_vrt_path):
        if with_euclidean_map:
            emap_path = io.related_file_path(output_path, postfix=".euclideand")
            compute_euclidean_map(tiles_vrt_path, emap_path, overwrite=True)
    
    for cleanup_file in [tiles_vrt_path, tiles_file_list, merged_vrt_path, geotiff_small_path, geotiff_small_filled_path]:
        if os.path.exists(cleanup_file): os.remove(cleanup_file)

    for t in tiles:
        if os.path.exists(t['filename']): os.remove(t['filename'])

    log.ODM_INFO('Completed %s in %s' % (output_file, datetime.now() - start))


def compute_euclidean_map(geotiff_path, output_path, overwrite=False):
    if not os.path.exists(geotiff_path):
        log.ODM_WARNING("Cannot compute euclidean map (file does not exist: %s)" % geotiff_path)
        return

    nodata = -9999
    with rasterio.open(geotiff_path) as f:
        nodata = f.nodatavals[0]

    if not os.path.isfile(output_path) or overwrite:
        if os.path.isfile(output_path):
            os.remove(output_path)

        log.ODM_INFO("Computing euclidean distance: %s" % output_path)

        if gdal_proximity is not None:
            try:
                gdal_proximity(['gdal_proximity.py', 
                                geotiff_path, output_path, '-values', str(nodata),
                                '-co', 'TILED=YES',
                                '-co', 'BIGTIFF=IF_SAFER',
                                '-co', 'COMPRESS=DEFLATE',
                            ])
            except Exception as e:
                log.ODM_WARNING("Cannot compute euclidean distance: %s" % str(e))

            if os.path.exists(output_path):
                return output_path
            else:
                log.ODM_WARNING("Cannot compute euclidean distance file: %s" % output_path)
        else:
            log.ODM_WARNING("Cannot compute euclidean map, gdal_proximity is missing")
            
    else:
        log.ODM_INFO("Found a euclidean distance map: %s" % output_path)
        return output_path


def median_smoothing(geotiff_path, output_path, window_size=512, num_workers=1, radius=4):
    """ Apply median smoothing """
    start = datetime.now()

    if not os.path.exists(geotiff_path):
        raise Exception('File %s does not exist!' % geotiff_path)

    kwargs = {
        'input': geotiff_path,
        'output': output_path,
        'window': window_size,
        'radius': radius,
    }
    system.run('fastrasterfilter "{input}" '
                '--output "{output}" '
                '--window-size {window} '
                '--radius {radius} '
                '--co TILED=YES '
                '--co BIGTIFF=IF_SAFER '
                '--co COMPRESS=DEFLATE '.format(**kwargs), env_vars={'OMP_NUM_THREADS': num_workers})

    log.ODM_INFO('Completed smoothing to create %s in %s' % (output_path, datetime.now() - start))
    return output_path


def get_dem_radius_steps(stats_file, steps, resolution, multiplier = 1.0):
    radius_steps = [point_cloud.get_spacing(stats_file, resolution) * multiplier]
    for _ in range(steps - 1):
        radius_steps.append(radius_steps[-1] * math.sqrt(2))
    
    return radius_steps