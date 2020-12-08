from opendm import log
from opendm.photo import find_largest_photo_dim

def get_depthmap_resolution(args, photos):
    if 'depthmap_resolution_is_set' in args:
        # Legacy
        log.ODM_WARNING("Legacy option --depthmap-resolution (this might be removed in a future version). Use --pc-quality instead.")
        return int(args.depthmap_resolution)
    else:
        max_dim = find_largest_photo_dim(photos)
        min_dim = 320 # Never go lower than this

        pc_quality_scale = {
            'ultra': 1,
            'high': 0.5,
            'medium': 0.25,
            'low': 0.125,
            'lowest': 0.0675
        }

        if max_dim > 0:
            return max(min_dim, int(max_dim * pc_quality_scale[args.pc_quality]))
        else:
            log.ODM_WARNING("Cannot compute max image dimensions, going with default depthmap_resolution of 640")
            return 640 # Sensible default
