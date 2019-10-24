import os
from opendm import log
from opendm import system
from opendm.cropper import Cropper
from opendm.concurrency import get_max_memory

def get_orthophoto_vars(args):
    return {
        'TILED': 'NO' if args.orthophoto_no_tiled else 'YES',
        'COMPRESS': args.orthophoto_compression,
        'PREDICTOR': '2' if args.orthophoto_compression in ['LZW', 'DEFLATE'] else '1',
        'BIGTIFF': 'IF_SAFER',
        'BLOCKXSIZE': 512,
        'BLOCKYSIZE': 512,
        'NUM_THREADS': args.max_concurrency
    }

def build_overviews(orthophoto_file):
    log.ODM_INFO("Building Overviews")
    kwargs = {'orthophoto': orthophoto_file}
    
    # Run gdaladdo
    system.run('gdaladdo -ro -r average '
                '--config BIGTIFF_OVERVIEW IF_SAFER '
                '--config COMPRESS_OVERVIEW JPEG '
                '{orthophoto} 2 4 8 16'.format(**kwargs))

def generate_png(orthophoto_file):
    log.ODM_INFO("Generating PNG")
    base, ext = os.path.splitext(orthophoto_file)
    orthophoto_png = base + '.png'

    system.run('gdal_translate -of png "%s" "%s" '
               '--config GDAL_CACHEMAX %s%% ' % (orthophoto_file, orthophoto_png, get_max_memory()))


def post_orthophoto_steps(args, bounds_file_path, orthophoto_file):
    if args.crop > 0:
        Cropper.crop(bounds_file_path, orthophoto_file, get_orthophoto_vars(args), warp_options=['-dstalpha'])

    if args.build_overviews:
        build_overviews(orthophoto_file)

    if args.orthophoto_png:
        generate_png(orthophoto_file)