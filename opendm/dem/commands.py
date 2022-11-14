import os
import sys
import rasterio
import numpy
import math
import time
import shutil
import functools
from joblib import delayed, Parallel
from opendm.system import run
from opendm import point_cloud
from opendm import io
from opendm import system
from opendm.concurrency import get_max_memory, parallel_map
from scipy import ndimage
from datetime import datetime
from opendm.vendor.gdal_fillnodata import main as gdal_fillnodata
from opendm import log
try:
    import Queue as queue
except:
    import queue
import threading

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

def rectify(lasFile, debug=False, reclassify_threshold=5, min_area=750, min_points=500):
    start = datetime.now()

    try:
        # Currently, no Python 2 lib that supports reading and writing LAZ, so we will do it manually until ODM is migrated to Python 3
        # When migration is done, we can move to pylas and avoid using PDAL for conversion
        tempLasFile = os.path.join(os.path.dirname(lasFile), 'tmp.las')

        # Convert LAZ to LAS
        cmd = [
            'pdal',
            'translate',
            '-i %s' % lasFile,
            '-o %s' % tempLasFile
        ]
        system.run(' '.join(cmd))

        log.ODM_INFO("Rectifying {} using with [reclassify threshold: {}, min area: {}, min points: {}]".format(lasFile, reclassify_threshold, min_area, min_points))
        run_rectification(
            input=tempLasFile, output=tempLasFile, debug=debug, \
            reclassify_plan='median', reclassify_threshold=reclassify_threshold, \
            extend_plan='surrounding', extend_grid_distance=5, \
            min_area=min_area, min_points=min_points)

        # Convert LAS to LAZ
        cmd = [
            'pdal',
            'translate',
            '-i %s' % tempLasFile,
            '-o %s' % lasFile
        ]
        system.run(' '.join(cmd))
        os.remove(tempLasFile)

    except Exception as e:
        raise Exception("Error rectifying ground in file %s: %s" % (lasFile, str(e)))

    log.ODM_INFO('Created %s in %s' % (lasFile, datetime.now() - start))
    return lasFile

error = None

def create_dem(input_point_cloud, dem_type, output_type='max', radiuses=['0.56'], gapfill=True,
                outdir='', resolution=0.1, max_workers=1, max_tile_size=4096,
                decimation=None, keep_unfilled_copy=False,
                apply_smoothing=True):
    """ Create DEM from multiple radii, and optionally gapfill """
    
    global error
    error = None

    start = datetime.now()

    if not os.path.exists(outdir):
        log.ODM_INFO("Creating %s" % outdir)
        os.mkdir(outdir)

    extent = point_cloud.get_extent(input_point_cloud)
    log.ODM_INFO("Point cloud bounds are [minx: %s, maxx: %s] [miny: %s, maxy: %s]" % (extent['minx'], extent['maxx'], extent['miny'], extent['maxy']))
    ext_width = extent['maxx'] - extent['minx']
    ext_height = extent['maxy'] - extent['miny']

    w, h = (int(math.ceil(ext_width / float(resolution))),
            int(math.ceil(ext_height / float(resolution))))

    # Set a floor, no matter the resolution parameter
    # (sometimes a wrongly estimated scale of the model can cause the resolution
    # to be set unrealistically low, causing errors)
    RES_FLOOR = 64
    if w < RES_FLOOR and h < RES_FLOOR:
        prev_w, prev_h = w, h
        
        if w >= h:
            w, h = (RES_FLOOR, int(math.ceil(ext_height / ext_width * RES_FLOOR)))
        else:
            w, h = (int(math.ceil(ext_width / ext_height * RES_FLOOR)), RES_FLOOR)
        
        floor_ratio = prev_w / float(w)
        resolution *= floor_ratio
        radiuses = [str(float(r) * floor_ratio) for r in radiuses]

        log.ODM_WARNING("Really low resolution DEM requested %s will set floor at %s pixels. Resolution changed to %s. The scale of this reconstruction might be off." % ((prev_w, prev_h), RES_FLOOR, resolution))
        
    final_dem_pixels = w * h

    num_splits = int(max(1, math.ceil(math.log(math.ceil(final_dem_pixels / float(max_tile_size * max_tile_size)))/math.log(2))))
    num_tiles = num_splits * num_splits
    log.ODM_INFO("DEM resolution is %s, max tile size is %s, will split DEM generation into %s tiles" % ((h, w), max_tile_size, num_tiles))

    tile_bounds_width = ext_width / float(num_splits)
    tile_bounds_height = ext_height / float(num_splits)

    tiles = []

    for r in radiuses:
        minx = extent['minx']

        for x in range(num_splits):
            miny = extent['miny']
            if x == num_splits - 1:
                maxx = extent['maxx']
            else:
                maxx = minx + tile_bounds_width

            for y in range(num_splits):
                if y == num_splits - 1:
                    maxy = extent['maxy']
                else:
                    maxy = miny + tile_bounds_height

                filename = os.path.join(os.path.abspath(outdir), '%s_r%s_x%s_y%s.tif' % (dem_type, r, x, y))

                tiles.append({
                    'radius': r,
                    'bounds': {
                        'minx': minx,
                        'maxx': maxx,
                        'miny': miny,
                        'maxy': maxy 
                    },
                    'filename': filename
                })

                miny = maxy
            minx = maxx

    # Sort tiles by increasing radius
    tiles.sort(key=lambda t: float(t['radius']), reverse=True)

    def process_tile(q):
        log.ODM_INFO("Generating %s (%s, radius: %s, resolution: %s)" % (q['filename'], output_type, q['radius'], resolution))
        
        d = pdal.json_gdal_base(q['filename'], output_type, q['radius'], resolution, q['bounds'])

        if dem_type == 'dtm':
            d = pdal.json_add_classification_filter(d, 2)

        if decimation is not None:
            d = pdal.json_add_decimation_filter(d, decimation)

        pdal.json_add_readers(d, [input_point_cloud])
        pdal.run_pipeline(d)

    parallel_map(process_tile, tiles, max_workers)

    output_file = "%s.tif" % dem_type
    output_path = os.path.abspath(os.path.join(outdir, output_file))

    # Verify tile results
    for t in tiles: 
        if not os.path.exists(t['filename']):
            raise Exception("Error creating %s, %s failed to be created" % (output_file, t['filename']))

    # Create virtual raster
    tiles_vrt_path = os.path.abspath(os.path.join(outdir, "tiles.vrt"))
    tiles_file_list = os.path.abspath(os.path.join(outdir, "tiles_list.txt"))
    with open(tiles_file_list, 'w') as f:
        for t in tiles:
            f.write(t['filename'] + '\n')

    run('gdalbuildvrt -input_file_list "%s" "%s" ' % (tiles_file_list, tiles_vrt_path))

    merged_vrt_path = os.path.abspath(os.path.join(outdir, "merged.vrt"))
    geotiff_tmp_path = os.path.abspath(os.path.join(outdir, 'tiles.tmp.tif'))
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
        'geotiff_tmp': geotiff_tmp_path,
        'geotiff_small': geotiff_small_path,
        'geotiff_small_filled': geotiff_small_filled_path
    }

    if gapfill:
        # Sometimes, for some reason gdal_fillnodata.py
        # behaves strangely when reading data directly from a .VRT
        # so we need to convert to GeoTIFF first.
        run('gdal_translate '
                '-co NUM_THREADS={threads} '
                '-co BIGTIFF=IF_SAFER '
                '--config GDAL_CACHEMAX {max_memory}% '
                '"{tiles_vrt}" "{geotiff_tmp}"'.format(**kwargs))

        # Scale to 10% size
        run('gdal_translate '
            '-co NUM_THREADS={threads} '
            '-co BIGTIFF=IF_SAFER '
            '--config GDAL_CACHEMAX {max_memory}% '
            '-outsize 10% 0 '
            '"{geotiff_tmp}" "{geotiff_small}"'.format(**kwargs))

        # Fill scaled
        gdal_fillnodata(['.', 
                        '-co', 'NUM_THREADS=%s' % kwargs['threads'], 
                        '-co', 'BIGTIFF=IF_SAFER',
                        '--config', 'GDAL_CACHE_MAX', str(kwargs['max_memory']) + '%',
                        '-b', '1',
                        '-of', 'GTiff',
                        kwargs['geotiff_small'], kwargs['geotiff_small_filled']])
        
        # Merge filled scaled DEM with unfilled DEM using bilinear interpolation
        run('gdalbuildvrt -resolution highest -r bilinear "%s" "%s" "%s"' % (merged_vrt_path, geotiff_small_filled_path, geotiff_tmp_path))
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

    if os.path.exists(geotiff_tmp_path):
        if not keep_unfilled_copy: 
            os.remove(geotiff_tmp_path)
        else:
            os.replace(geotiff_tmp_path, io.related_file_path(output_path, postfix=".unfilled"))
    
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

    if not os.path.exists(output_path) or overwrite:
        log.ODM_INFO("Computing euclidean distance: %s" % output_path)

        if gdal_proximity is not None:
            try:
                gdal_proximity(['gdal_proximity.py', geotiff_path, output_path, '-values', str(nodata)])
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


def median_smoothing(geotiff_path, output_path, smoothing_iterations=1, window_size=512, num_workers=1):
    """ Apply median smoothing """
    start = datetime.now()

    if not os.path.exists(geotiff_path):
        raise Exception('File %s does not exist!' % geotiff_path)

    log.ODM_INFO('Starting smoothing...')

    with rasterio.open(geotiff_path) as img:
        nodata = img.nodatavals[0]
        dtype = img.dtypes[0]
        shape = img.shape
        arr = img.read()[0]
        for i in range(smoothing_iterations):
            log.ODM_INFO("Smoothing iteration %s" % str(i + 1))
            rows, cols = numpy.meshgrid(numpy.arange(0, shape[0], window_size), numpy.arange(0, shape[1], window_size))
            rows = rows.flatten()
            cols = cols.flatten()
            rows_end = numpy.minimum(rows + window_size, shape[0])
            cols_end= numpy.minimum(cols + window_size, shape[1])
            windows = numpy.dstack((rows, cols, rows_end, cols_end)).reshape(-1, 4)

            filter = functools.partial(ndimage.median_filter, size=9, output=dtype, mode='nearest')

            # threading backend and GIL released filter are important for memory efficiency and multi-core performance
            window_arrays = Parallel(n_jobs=num_workers, backend='threading')(delayed(window_filter_2d)(arr, nodata , window, 9, filter) for window in windows)

            for window, win_arr in zip(windows, window_arrays):
                arr[window[0]:window[2], window[1]:window[3]] = win_arr
        log.ODM_INFO("Smoothing completed in %s" % str(datetime.now() - start))
        # write output
        with rasterio.open(output_path, 'w', BIGTIFF="IF_SAFER", **img.profile) as imgout:
            imgout.write(arr, 1)

    log.ODM_INFO('Completed smoothing to create %s in %s' % (output_path, datetime.now() - start))
    return output_path


def window_filter_2d(arr, nodata, window, kernel_size, filter):
    """
    Apply a filter to dem within a window, expects to work with kernal based filters

    :param geotiff_path: path to the geotiff to filter
    :param window: the window to apply the filter, should be a list contains row start, col_start, row_end, col_end
    :param kernel_size: the size of the kernel for the filter, works with odd numbers, need to test if it works with even numbers
    :param filter: the filter function which takes a 2d array as input and filter results as output.
    """
    shape = arr.shape[:2]
    if window[0] < 0 or window[1] < 0 or window[2] > shape[0] or window[3] > shape[1]:
        raise Exception('Window is out of bounds')
    expanded_window = [ max(0, window[0] - kernel_size // 2), max(0, window[1] - kernel_size // 2), min(shape[0], window[2] + kernel_size // 2), min(shape[1], window[3] + kernel_size // 2) ]
    win_arr = arr[expanded_window[0]:expanded_window[2], expanded_window[1]:expanded_window[3]]
    # Should have a better way to handle nodata, similar to the way the filter algorithms handle the border (reflection, nearest, interpolation, etc).
    # For now will follow the old approach to guarantee identical outputs
    nodata_locs = win_arr == nodata
    win_arr = filter(win_arr)
    win_arr[nodata_locs] = nodata
    win_arr = win_arr[window[0] - expanded_window[0] : window[2] - expanded_window[0], window[1] - expanded_window[1] : window[3] - expanded_window[1]]
    return win_arr
