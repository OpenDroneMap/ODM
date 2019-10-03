from opendm import log
from opendm import system

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