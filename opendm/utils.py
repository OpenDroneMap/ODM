from opendm import log
from opendm.photo import find_largest_photo_dim

def get_depthmap_resolution(args, photos):
    if 'depthmap_resolution_is_set' in args:
        # Legacy
        log.ODM_WARNING("Legacy option --depthmap-resolution (this might be removed in a future version). Use --pc-quality instead.")
        return int(args.depthmap_resolution)
    else:
        max_dim = find_largest_photo_dim(photos)

        pc_quality_scale = {
            'double': 2.00,
            'original': 1.00,
            'half': 0.50,
            'quarter': 0.25,
            'eighth': 0.125,
        }

        if max_dim > 0:
            return int(max_dim * pc_quality_scale[args.pc_quality])
        else:
            log.ODM_WARNING("Cannot compute max image dimensions, going with default depthmap_resolution of 640")
            return 640 # Sensible default
