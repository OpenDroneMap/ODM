
def get_dem_vars(args):
    return {
        'TILED': 'YES',
        'COMPRESS': 'DEFLATE',
        'BLOCKXSIZE': 512,
        'BLOCKYSIZE': 512,
        'BIGTIFF': 'IF_SAFER',
        'NUM_THREADS': args.max_concurrency,
    }
