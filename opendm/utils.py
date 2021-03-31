from opendm import log
from opendm.photo import find_largest_photo_dim
from osgeo import gdal

def get_depthmap_resolution(args, photos):
    if 'depthmap_resolution_is_set' in args:
        # Legacy
        log.ODM_WARNING("Legacy option --depthmap-resolution (this might be removed in a future version). Use --pc-quality instead.")
        return int(args.depthmap_resolution)
    else:
        max_dim = find_largest_photo_dim(photos)
        min_dim = 320 # Never go lower than this

        pc_quality_scale = {
            'ultra': 0.5,
            'high': 0.25,
            'medium': 0.125,
            'low': 0.0675,
            'lowest': 0.03375
        }

        if max_dim > 0:
            return max(min_dim, int(max_dim * pc_quality_scale[args.pc_quality]))
        else:
            log.ODM_WARNING("Cannot compute max image dimensions, going with default depthmap_resolution of 640")
            return 640 # Sensible default

def get_raster_stats(geotiff):
    stats = []
    gtif = gdal.Open(geotiff)
    for b in range(gtif.RasterCount):
        srcband = gtif.GetRasterBand(b + 1)
        s = srcband.GetStatistics(True, True)

        stats.append({
            'min': s[0],
            'max': s[1],
            'mean': s[2],
            'stddev': s[3]
        })
            
    return stats