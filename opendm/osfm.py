"""
OpenSfM related utils
"""

import os, shutil, sys
from opendm import io
from opendm import log
from opendm import system
from opendm import context

def run(command, opensfm_project_path):
    system.run('%s/bin/opensfm %s %s' %
                (context.opensfm_path, command, opensfm_project_path))


def export_bundler(opensfm_project_path, destination_bundle_file, rerun=False):
    if not io.file_exists(destination_bundle_file) or rerun:
            # convert back to bundler's format
            system.run('%s/bin/export_bundler %s' %
                    (context.opensfm_path, opensfm_project_path))
    else:
        log.ODM_WARNING('Found a valid Bundler file in: %s' % destination_bundle_file)


def reconstruct(opensfm_project_path, rerun=False):
    tracks_file = os.path.join(opensfm_project_path, 'tracks.csv')
    reconstruction_file = os.path.join(opensfm_project_path, 'reconstruction.json')

    if not io.file_exists(tracks_file) or rerun:
        run('create_tracks', opensfm_project_path)
    else:
        log.ODM_WARNING('Found a valid OpenSfM tracks file in: %s' % tracks_file)

    if not io.file_exists(reconstruction_file) or rerun:
        run('reconstruct', opensfm_project_path)
    else:
        log.ODM_WARNING('Found a valid OpenSfM reconstruction file in: %s' % reconstruction_file)

    # Check that a reconstruction file has been created
    if not io.file_exists(reconstruction_file):
        log.ODM_ERROR("The program could not process this dataset using the current settings. "
                        "Check that the images have enough overlap, "
                        "that there are enough recognizable features "
                        "and that the images are in focus. "
                        "You could also try to increase the --min-num-features parameter."
                        "The program will now exit.")
        raise Exception("Reconstruction could not be generated")


def setup(args, images_path, opensfm_path, photos, gcp_path=None, append_config = [], rerun=False):
    """
    Setup a OpenSfM project
    """
    if rerun and io.dir_exists(opensfm_path):
        shutil.rmtree(opensfm_path)

    if not io.dir_exists(opensfm_path):
        system.mkdir_p(opensfm_path)

    # create file list
    list_path = io.join_paths(opensfm_path, 'image_list.txt')
    if not io.file_exists(list_path) or rerun:
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
    else:
        log.ODM_WARNING("%s already exists, not rerunning OpenSfM setup" % list_path)

def feature_matching(opensfm_project_path, rerun=False):
    if not feature_matching_done(opensfm_project_path) or rerun:
        run('extract_metadata', opensfm_project_path)

        # TODO: distributed workflow should do these two steps independently
        run('detect_features', opensfm_project_path)
        run('match_features', opensfm_project_path)

        mark_feature_matching_done(opensfm_project_path)
    else:
        log.ODM_WARNING('Found a feature matching done progress file in: %s' %
                        feature_matching_done_file(opensfm_project_path))

def feature_matching_done_file(opensfm_project_path):
    return io.join_paths(opensfm_project_path, 'matching_done.txt')

def mark_feature_matching_done(opensfm_project_path):
    with open(feature_matching_done_file(opensfm_project_path), 'w') as fout:
        fout.write("Matching done!\n")

def feature_matching_done(opensfm_project_path):
    return io.file_exists(feature_matching_done_file(opensfm_project_path))

def get_submodel_argv(args, submodels_path, submodel_name):
    """
    :return the same as argv, but removing references to --split, 
        setting/replacing --project-path and name
        setting/replacing --crop to 0 (never crop on submodels)
    """
    argv = sys.argv

    result = [argv[0]]
    i = 1
    project_path_found = False
    project_name_added = False
    crop_found = True

    # TODO: what about GCP paths?

    while i < len(argv):
        arg = argv[i]
        
        # Last?
        if i == len(argv) - 1:
            # Project name?
            if arg == args.name:
                result.append(submodel_name)
                project_name_added = True
            else:
                result.append(arg)
            i += 1
        elif arg == '--project-path':
            result.append(arg)
            result.append(submodels_path)
            project_path_found = True
            i += 2
        elif arg == '--crop':
            result.append(arg)
            result.append('0')
            crop_found = True
            i += 2
        elif arg == '--split':
            i += 2
        else:
            result.append(arg)
            i += 1
    
    if not project_path_found:
        result.append('--project-path')
        result.append(submodel_project_path)
    
    if not crop_found:
        result.append('--crop')
        result.append('0')

    if not project_name_added:
        result.append(submodel_name)
    
    return result


def get_submodel_paths(submodels_path, *paths):
    """
    :return Existing paths for all submodels
    """
    result = []
    for f in os.listdir(submodels_path):
        if f.startswith('submodel'):
            p = os.path.join(submodels_path, f, *paths) 
            if os.path.exists(p):
                result.append(p)
            else:
                log.ODM_WARNING("Missing %s from submodel %s" % (p, f))

    return result