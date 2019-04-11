import os, glob
import gippy
import numpy
import math
from opendm.system import run
from opendm import point_cloud
from opendm.concurrency import get_max_memory
import pprint

from scipy import ndimage
from datetime import datetime
from opendm import log
from loky import get_reusable_executor
from functools import partial

from . import pdal

def classify(lasFile, slope=0.15, cellsize=1, maxWindowSize=18, verbose=False):
    start = datetime.now()

    try:
        pdal.run_pdaltranslate_smrf(lasFile, lasFile, slope, cellsize, maxWindowSize, verbose)
    except:
        raise Exception("Error creating classified file %s" % fout)

    log.ODM_INFO('Created %s in %s' % (os.path.relpath(lasFile), datetime.now() - start))
    return lasFile


def create_dem(input_point_cloud, dem_type, output_type='max', radiuses=['0.56'], gapfill=True,
                outdir='', resolution=0.1, max_workers=None, max_tile_size=4096, 
                verbose=False, decimation=None):
    """ Create DEM from multiple radii, and optionally gapfill """
    start = datetime.now()

    if not os.path.exists(outdir):
        log.ODM_INFO("Creating %s" % outdir)
        os.mkdir(outdir)

    extent = point_cloud.get_extent(input_point_cloud)
    log.ODM_INFO("Point cloud bounds are [minx: %s, maxx: %s] [miny: %s, maxy: %s]" % (extent['minx'], extent['maxx'], extent['miny'], extent['maxy']))
    # extent = {
    #     'maxx': 100,
    #     'minx': 0,
    #     'maxy': 100,
    #     'miny': 0
    # }
    ext_width = extent['maxx'] - extent['minx']
    ext_height = extent['maxy'] - extent['miny']

    final_dem_resolution = (int(math.ceil(ext_width / float(resolution))),
                            int(math.ceil(ext_height / float(resolution))))
    num_splits = int(math.ceil(max(final_dem_resolution) / float(max_tile_size)))
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

    # pp = pprint.PrettyPrinter(indent=4)
    # pp.pprint(queue)
    # TODO: parallel queue
    queue = tiles[:]

    for q in queue:
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
    

    output_file = "%s.tif" % dem_type
    output_path = os.path.abspath(os.path.join(outdir, output_file))

    # Verify tile results
    for t in tiles: 
        if not os.path.exists(t['filename']):
            raise Exception("Error creating %s, %s failed to be created" % (output_file, t['filename']))
    
    # Create virtual raster
    vrt_path = os.path.abspath(os.path.join(outdir, "merged.vrt"))
    run('gdalbuildvrt "%s" "%s"' % (vrt_path, '" "'.join(map(lambda t: t['filename'], tiles))))

    geotiff_path = os.path.abspath(os.path.join(outdir, 'merged.tiff'))

    # Build GeoTIFF
    kwargs = {
        'max_memory': get_max_memory(),
        'threads': max_workers if max_workers else 'ALL_CPUS',
        'vrt': vrt_path,
        'geotiff': geotiff_path
    }

    run('gdal_translate '
            '-co NUM_THREADS={threads} '
            '--config GDAL_CACHEMAX {max_memory}% '
            '{vrt} {geotiff}'.format(**kwargs))

    if gapfill:
        gapfill_and_smooth(geotiff_path, output_path)
        os.remove(geotiff_path)
    else:
        log.ODM_INFO("Skipping gap-fill interpolation")
        os.rename(geotiff_path, output_path)

    # TODO cleanup
    
    log.ODM_INFO('Completed %s in %s' % (output_file, datetime.now() - start))



def gapfill_and_smooth(geotiff_path, output_path):
    """ Gap fill with nearest neighbor interpolation and apply median smoothing """
    start = datetime.now()

    if not os.path.exists(geotiff_path):
        raise Exception('File %s does not exist!' % geotiff_path)

    log.ODM_INFO('Starting gap-filling with nearest interpolation...')

    img = gippy.GeoImage(geotiff_path)
    nodata = img[0].nodata()
    arr = img[0].read()

    # Nearest neighbor interpolation at bad points
    indices = ndimage.distance_transform_edt(arr == nodata, 
                                    return_distances=False, 
                                    return_indices=True)
    arr = arr[tuple(indices)]

    # Median filter (careful, changing the value 5 might require tweaking)
    # the lines below. There's another numpy function that takes care of 
    # these edge cases, but it's slower.
    from scipy import signal
    arr = signal.medfilt(arr, 5)
    
    # Fill corner points with nearest value
    if arr.shape >= (4, 4):
        arr[0][:2] = arr[1][0] = arr[1][1]
        arr[0][-2:] = arr[1][-1] = arr[2][-1]
        arr[-1][:2] = arr[-2][0] = arr[-2][1]
        arr[-1][-2:] = arr[-2][-1] = arr[-2][-2]

    # write output
    imgout = gippy.GeoImage.create_from(img, output_path)
    imgout.set_nodata(nodata)
    imgout[0].write(arr)
    output_path = imgout.filename()
    imgout = None

    log.ODM_INFO('Completed gap-filling to create %s in %s' % (os.path.relpath(output_path), datetime.now() - start))

    return output_path