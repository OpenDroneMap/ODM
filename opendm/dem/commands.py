import os
import sys
import gippy
import numpy
import math
import time
from opendm.system import run
from opendm import point_cloud
from opendm.concurrency import get_max_memory
from scipy import ndimage, signal
from datetime import datetime
from opendm import log
try:
    import Queue as queue
except:
    import queue
import threading

from . import pdal

def classify(lasFile, scalar, slope, threshold, window, verbose=False):
    start = datetime.now()

    try:
        pdal.run_pdaltranslate_smrf(lasFile, lasFile, scalar, slope, threshold, window, verbose)
    except:
        raise Exception("Error creating classified file %s" % fout)

    log.ODM_INFO('Created %s in %s' % (os.path.relpath(lasFile), datetime.now() - start))
    return lasFile

error = None

def create_dem(input_point_cloud, dem_type, output_type='max', radiuses=['0.56'], gapfill=True,
                outdir='', resolution=0.1, max_workers=1, max_tile_size=2048,
                verbose=False, decimation=None):
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

    final_dem_resolution = (int(math.ceil(ext_width / float(resolution))),
                            int(math.ceil(ext_height / float(resolution))))
    final_dem_pixels = final_dem_resolution[0] * final_dem_resolution[1]

    num_splits = int(max(1, math.ceil(math.log(math.ceil(final_dem_pixels / float(max_tile_size * max_tile_size)))/math.log(2))))
    num_tiles = num_splits * num_splits
    log.ODM_INFO("DEM resolution is %s, max tile size is %s, will split DEM generation into %s tiles" % (final_dem_resolution, max_tile_size, num_tiles))

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

    def process_one(q):
        log.ODM_INFO("Generating %s (%s, radius: %s, resolution: %s)" % (q['filename'], output_type, q['radius'], resolution))
        
        d = pdal.json_gdal_base(q['filename'], output_type, q['radius'], resolution, q['bounds'])

        if dem_type == 'dsm':
            d = pdal.json_add_classification_filter(d, 2, equality='max')
        elif dem_type == 'dtm':
            d = pdal.json_add_classification_filter(d, 2)

        if decimation is not None:
            d = pdal.json_add_decimation_filter(d, decimation)

        pdal.json_add_readers(d, [input_point_cloud])
        pdal.run_pipeline(d, verbose=verbose)

    def worker():
        global error

        while True:
            (num, q) = pq.get()
            if q is None or error is not None:
                pq.task_done()
                break

            try:
                process_one(q)
            except Exception as e:
                error = e
            finally:
                pq.task_done()

    if max_workers > 1:
        use_single_thread = False
        pq = queue.PriorityQueue()
        threads = []
        for i in range(max_workers):
            t = threading.Thread(target=worker)
            t.start()
            threads.append(t)

        for t in tiles:
            pq.put((i, t.copy()))

        def stop_workers():
            for i in range(len(threads)):
                pq.put((-1, None))
            for t in threads:
                t.join()

        # block until all tasks are done
        try:
            while pq.unfinished_tasks > 0:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("CTRL+C terminating...")
            stop_workers()
            sys.exit(1)

        stop_workers()

        if error is not None:
            # Try to reprocess using a single thread
            # in case this was a memory error
            log.ODM_WARNING("DEM processing failed with multiple threads, let's retry with a single thread...")
            use_single_thread = True
    else:
        use_single_thread = True

    if use_single_thread:
        # Boring, single thread processing
        for q in tiles:
            process_one(q)

    output_file = "%s.tif" % dem_type
    output_path = os.path.abspath(os.path.join(outdir, output_file))

    # Verify tile results
    for t in tiles: 
        if not os.path.exists(t['filename']):
            raise Exception("Error creating %s, %s failed to be created" % (output_file, t['filename']))
    
    # Create virtual raster
    vrt_path = os.path.abspath(os.path.join(outdir, "merged.vrt"))
    run('gdalbuildvrt "%s" "%s"' % (vrt_path, '" "'.join(map(lambda t: t['filename'], tiles))))

    geotiff_tmp_path = os.path.abspath(os.path.join(outdir, 'merged.tmp.tif'))
    geotiff_path = os.path.abspath(os.path.join(outdir, 'merged.tif'))

    # Build GeoTIFF
    kwargs = {
        'max_memory': get_max_memory(),
        'threads': max_workers if max_workers else 'ALL_CPUS',
        'vrt': vrt_path,
        'geotiff': geotiff_path,
        'geotiff_tmp': geotiff_tmp_path
    }

    if gapfill:
        # Sometimes, for some reason gdal_fillnodata.py
        # behaves strangely when reading data directly from a .VRT
        # so we need to convert to GeoTIFF first.
        run('gdal_translate '
                '-co NUM_THREADS={threads} '
                '--config GDAL_CACHEMAX {max_memory}% '
                '{vrt} {geotiff_tmp}'.format(**kwargs))

        run('gdal_fillnodata.py '
            '-co NUM_THREADS={threads} '
            '--config GDAL_CACHEMAX {max_memory}% '
            '-b 1 '
            '-of GTiff '
            '{geotiff_tmp} {geotiff}'.format(**kwargs))
    else:
        run('gdal_translate '
                '-co NUM_THREADS={threads} '
                '--config GDAL_CACHEMAX {max_memory}% '
                '{vrt} {geotiff}'.format(**kwargs))

    post_process(geotiff_path, output_path)
    os.remove(geotiff_path)

    if os.path.exists(geotiff_tmp_path): os.remove(geotiff_tmp_path)
    if os.path.exists(vrt_path): os.remove(vrt_path)
    for t in tiles:
        if os.path.exists(t['filename']): os.remove(t['filename'])
    
    log.ODM_INFO('Completed %s in %s' % (output_file, datetime.now() - start))


def post_process(geotiff_path, output_path, smoothing_iterations=1):
    """ Apply median smoothing """
    start = datetime.now()

    if not os.path.exists(geotiff_path):
        raise Exception('File %s does not exist!' % geotiff_path)

    log.ODM_INFO('Starting post processing (smoothing)...')

    img = gippy.GeoImage(geotiff_path)
    nodata = img[0].nodata()
    arr = img[0].read()

    # Median filter (careful, changing the value 5 might require tweaking)
    # the lines below. There's another numpy function that takes care of 
    # these edge cases, but it's slower.
    for i in range(smoothing_iterations):
        log.ODM_INFO("Smoothing iteration %s" % str(i + 1))
        arr = signal.medfilt(arr, 5)
    
    # Fill corner points with nearest value
    if arr.shape >= (4, 4):
        arr[0][:2] = arr[1][0] = arr[1][1]
        arr[0][-2:] = arr[1][-1] = arr[2][-1]
        arr[-1][:2] = arr[-2][0] = arr[-2][1]
        arr[-1][-2:] = arr[-2][-1] = arr[-2][-2]

    # Median filter leaves a bunch of zeros in nodata areas
    locs = numpy.where(arr == 0.0)
    arr[locs] = nodata

    # write output
    imgout = gippy.GeoImage.create_from(img, output_path)
    imgout.set_nodata(nodata)
    imgout[0].write(arr)
    output_path = imgout.filename()
    imgout = None

    log.ODM_INFO('Completed post processing to create %s in %s' % (os.path.relpath(output_path), datetime.now() - start))

    return output_path