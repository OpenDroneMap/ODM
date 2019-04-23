"""
OpenSfM related utils
"""

from opendm import io
from opendm import log
from opendm import system
from opendm import context

def run(command, opensfm_project_path):
    system.run('%s/bin/opensfm %s %s' %
                           (context.opensfm_path, command, opensfm_project_path))

def setup(args, params, images_path, opensfm_path, photos, gcp_path=None, append_config = []):
    """
    Setup a OpenSfM project
    """
    if not io.dir_exists(opensfm_path):
        system.mkdir_p(opensfm_path)

    # create file list
    list_path = io.join_paths(opensfm_path, 'image_list.txt')
    has_alt = True
    with open(list_path, 'w') as fout:
        for photo in photos:
            if not photo.altitude:
                has_alt = False
            fout.write('%s\n' % io.join_paths(images_path, photo.filename))

    # create config file for OpenSfM
    config = [
        "use_exif_size: %s" % ('no' if not params.get('use_exif_size') else 'yes'),
        "feature_process_size: %s" % params.get('feature_process_size'),
        "feature_min_frames: %s" % params.get('feature_min_frames'),
        "processes: %s" % params.get('processes'),
        "matching_gps_neighbors: %s" % params.get('matching_gps_neighbors'),
        "depthmap_method: %s" % args.opensfm_depthmap_method,
        "depthmap_resolution: %s" % args.depthmap_resolution,
        "depthmap_min_patch_sd: %s" % args.opensfm_depthmap_min_patch_sd,
        "depthmap_min_consistent_views: %s" % args.opensfm_depthmap_min_consistent_views,
        "optimize_camera_parameters: %s" % ('no' if params.get('fixed_camera_params') else 'yes')
    ]

    if has_alt:
        log.ODM_DEBUG("Altitude data detected, enabling it for GPS alignment")
        config.append("use_altitude_tag: yes")
        config.append("align_method: naive")
    else:
        config.append("align_method: orientation_prior")
        config.append("align_orientation_prior: vertical")

    if args.use_hybrid_bundle_adjustment:
        log.ODM_DEBUG("Enabling hybrid bundle adjustment")
        config.append("bundle_interval: 100")          # Bundle after adding 'bundle_interval' cameras
        config.append("bundle_new_points_ratio: 1.2")  # Bundle when (new points) / (bundled points) > bundle_new_points_ratio
        config.append("local_bundle_radius: 1")        # Max image graph distance for images to be included in local bundle adjustment

    if args.matcher_distance > 0:
        config.append("matching_gps_distance: %s" % params.get('matching_gps_distance'))

    if gcp_path:
        config.append("bundle_use_gcp: yes")
        io.copy(gcp_path, opensfm_path)
    
    config = config + append_config

    # write config file
    log.ODM_DEBUG(config)
    config_filename = io.join_paths(opensfm_path, 'config.yaml')
    with open(config_filename, 'w') as fout:
        fout.write("\n".join(config))