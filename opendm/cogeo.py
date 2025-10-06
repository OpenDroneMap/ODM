import os
import shutil
from opendm import system
from opendm.concurrency import get_max_memory
from opendm import io
from opendm import log

def convert_to_cogeo(src_path, blocksize=256, max_workers=1, compression="DEFLATE", compact_overviews=False):
    """
    Guarantee that the .tif passed as an argument is a Cloud Optimized GeoTIFF (cogeo)
    The file is destructively converted into a cogeo.
    If the file cannot be converted, the function does not change the file
    :param src_path: path to GeoTIFF
    :return: True on success
    """

    if not os.path.isfile(src_path):
        log.ODM_WARNING("Cannot convert to cogeo: %s (file does not exist)" % src_path)
        return False

    log.ODM_INFO("Optimizing %s as Cloud Optimized GeoTIFF" % src_path)

    tmpfile = io.related_file_path(src_path, postfix='_cogeo')
    swapfile = io.related_file_path(src_path, postfix='_cogeo_swap')

    # Configuration params
    threads = max_workers if max_workers else 'ALL_CPUS'
    predictor = '2' if compression in ['LZW', 'DEFLATE'] else '1'
    max_memory = get_max_memory()

    # Build gdal_translate command
    cmd = [
        'gdal_translate',
        '-of', 'COG',
        '-co', f'NUM_THREADS={threads}',
        '-co', f'BLOCKSIZE={blocksize}',
        '-co', f'COMPRESS={compression}',
        '-co', f'PREDICTOR={predictor}',
        '-co', 'BIGTIFF=IF_SAFER',
        '-co', 'RESAMPLING=NEAREST',
    ]

    if compact_overviews:
        cmd.extend([
            '-co', 'PHOTOMETRIC_OVERVIEW=YCBCR',
            '-co', 'INTERLEAVE_OVERVIEW=PIXEL',
        ])

    cmd.extend([
        '--config', 'GDAL_CACHEMAX', f'{max_memory}%',
        '--config', 'GDAL_NUM_THREADS', str(threads),
        src_path,
        tmpfile,
    ])

    try:
        system.run(' '.join(cmd))
    except Exception as e:
        log.ODM_WARNING("Cannot create Cloud Optimized GeoTIFF: %s" % str(e))
        return False

    if os.path.isfile(tmpfile):
        shutil.move(src_path, swapfile) # Move to swap location

        try:
            shutil.move(tmpfile, src_path)
        except IOError as e:
            log.ODM_WARNING("Cannot move %s to %s: %s" % (tmpfile, src_path, str(e)))
            shutil.move(swapfile, src_path) # Attempt to restore
            return False

        if os.path.isfile(swapfile):
            os.remove(swapfile)

        return True
    else:
        return False
