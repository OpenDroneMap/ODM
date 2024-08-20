import rasterio

def get_dem_vars(args):
    return {
        'TILED': 'YES',
        'COMPRESS': 'DEFLATE',
        'BLOCKXSIZE': 512,
        'BLOCKYSIZE': 512,
        'BIGTIFF': 'IF_SAFER',
        'NUM_THREADS': args.max_concurrency,
    }

def update_tags(dem_file):

    # - Update the geotiff tags in place using rasterio
    with rasterio.open(dem_file, 'r+') as rst:
        rst.update_tags(TIFFTAG_DATETIME='2024:01:01 00:00+00:00')
        rst.update_tags(TIFFTAG_SOFTWARE='OpenDroneMap')
