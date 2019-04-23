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


def export_bundler(opensfm_project_path, destination_bundle_file, rerun=False):
    if not io.file_exists(destination_bundle_file) or self.rerun():
            # convert back to bundler's format
            system.run('%s/bin/export_bundler %s' %
                    (context.opensfm_path, opensfm_project_path))
    else:
        log.ODM_WARNING('Found a valid Bundler file in: %s' % destination_bundle_file)


def setup(args, images_path, opensfm_path, photos, gcp_path=None, append_config = []):
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

            # TODO: does this need to be a relative path?

    # create config file for OpenSfM
    config = [
        "use_exif_size: no",
        "feature_process_size: %s" % args.resize_to,
        "feature_min_frames: %s" % args.min_num_features,
        "processes: %s" % args.max_concurrency,
        "matching_gps_neighbors: %s" % args.matcher_neighbors,
        "depthmap_method: %s" % args.opensfm_depthmap_method,
        "depthmap_resolution: %s" % args.depthmap_resolution,
        "depthmap_min_patch_sd: %s" % args.opensfm_depthmap_min_patch_sd,
        "depthmap_min_consistent_views: %s" % args.opensfm_depthmap_min_consistent_views,
        "optimize_camera_parameters: %s" % ('no' if args.use_fixed_camera_params else 'yes')
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
        config.append("matching_gps_distance: %s" % args.matcher_distance)

    if gcp_path:
        config.append("bundle_use_gcp: yes")
        io.copy(gcp_path, opensfm_path)
    
    config = config + append_config

    # write config file
    log.ODM_DEBUG(config)
    config_filename = io.join_paths(opensfm_path, 'config.yaml')
    with open(config_filename, 'w') as fout:
        fout.write("\n".join(config))

def run_feature_matching(opensfm_project_path, rerun=False):
    matched_done_file = io.join_paths(opensfm_project_path, 'matching_done.txt')
    if not io.file_exists(matched_done_file) or rerun:
        run('extract_metadata', opensfm_project_path)

        # TODO: distributed workflow should do these two steps independently
        run('detect_features', opensfm_project_path)
        run('match_features', opensfm_project_path)

        with open(matched_done_file, 'w') as fout:
            fout.write("Matching done!\n")
    else:
        log.ODM_WARNING('Found a feature matching done progress file in: %s' %
                        matched_done_file)