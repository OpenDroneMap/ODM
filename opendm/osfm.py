"""
OpenSfM related utils
"""

import os, shutil, sys, json, argparse
import yaml
from opendm import io
from opendm import log
from opendm import system
from opendm import context
from opendm import camera
from opensfm.large import metadataset
from opensfm.large import tools
from opensfm.commands import undistort

class OSFMContext:
    def __init__(self, opensfm_project_path):
        self.opensfm_project_path = opensfm_project_path
    
    def run(self, command):
        # Use Python 2.x by default, otherwise OpenSfM uses Python 3.x
        system.run('/usr/bin/env python2 %s/bin/opensfm %s "%s"' %
                    (context.opensfm_path, command, self.opensfm_project_path))

    def is_reconstruction_done(self):
        tracks_file = os.path.join(self.opensfm_project_path, 'tracks.csv')
        reconstruction_file = os.path.join(self.opensfm_project_path, 'reconstruction.json')

        return io.file_exists(tracks_file) and io.file_exists(reconstruction_file)

    def reconstruct(self, rerun=False):
        tracks_file = os.path.join(self.opensfm_project_path, 'tracks.csv')
        reconstruction_file = os.path.join(self.opensfm_project_path, 'reconstruction.json')

        if not io.file_exists(tracks_file) or rerun:
            self.run('create_tracks')
        else:
            log.ODM_WARNING('Found a valid OpenSfM tracks file in: %s' % tracks_file)

        if not io.file_exists(reconstruction_file) or rerun:
            self.run('reconstruct')
        else:
            log.ODM_WARNING('Found a valid OpenSfM reconstruction file in: %s' % reconstruction_file)

        # Check that a reconstruction file has been created
        if not self.reconstructed():
            log.ODM_ERROR("The program could not process this dataset using the current settings. "
                            "Check that the images have enough overlap, "
                            "that there are enough recognizable features "
                            "and that the images are in focus. "
                            "You could also try to increase the --min-num-features parameter."
                            "The program will now exit.")
            exit(1)


    def setup(self, args, images_path, photos, reconstruction, append_config = [], rerun=False):
        """
        Setup a OpenSfM project
        """
        if rerun and io.dir_exists(self.opensfm_project_path):
            shutil.rmtree(self.opensfm_project_path)

        if not io.dir_exists(self.opensfm_project_path):
            system.mkdir_p(self.opensfm_project_path)

        list_path = io.join_paths(self.opensfm_project_path, 'image_list.txt')
        if not io.file_exists(list_path) or rerun:
            
            # create file list
            has_alt = True
            has_gps = False
            with open(list_path, 'w') as fout:
                for photo in photos:
                    if not photo.altitude:
                        has_alt = False
                    if photo.latitude is not None and photo.longitude is not None:
                        has_gps = True
                    fout.write('%s\n' % io.join_paths(images_path, photo.filename))
            
            # check for image_groups.txt (split-merge)
            image_groups_file = os.path.join(args.project_path, "image_groups.txt")
            if io.file_exists(image_groups_file):
                log.ODM_INFO("Copied image_groups.txt to OpenSfM directory")
                io.copy(image_groups_file, os.path.join(self.opensfm_project_path, "image_groups.txt"))
        
            # check for cameras
            if args.cameras:
                try:
                    camera_overrides = camera.get_opensfm_camera_models(args.cameras)
                    with open(os.path.join(self.opensfm_project_path, "camera_models_overrides.json"), 'w') as f:
                        f.write(json.dumps(camera_overrides))
                    log.ODM_INFO("Wrote camera_models_overrides.json to OpenSfM directory")
                except Exception as e:
                    log.ODM_WARNING("Cannot set camera_models_overrides.json: %s" % str(e))

            use_bow = False
            feature_type = "SIFT"

            matcher_neighbors = args.matcher_neighbors
            if matcher_neighbors != 0 and reconstruction.multi_camera is not None:
                matcher_neighbors *= len(reconstruction.multi_camera)
                log.ODM_INFO("Increasing matcher neighbors to %s to accomodate multi-camera setup" % matcher_neighbors)
                log.ODM_INFO("Multi-camera setup, using BOW matching")
                use_bow = True

            # GPSDOP override if we have GPS accuracy information (such as RTK)
            override_gps_dop = 'gps_accuracy_is_set' in args
            for p in photos:
                if p.get_gps_dop() is not None:
                    override_gps_dop = True
                    break
            
            if override_gps_dop:
                if 'gps_accuracy_is_set' in args:
                    log.ODM_INFO("Forcing GPS DOP to %s for all images" % args.gps_accuracy)
                else:
                    log.ODM_INFO("Looks like we have RTK accuracy info for some photos. Good! We'll use it.")

                exif_overrides = {}
                for p in photos:
                    dop = args.gps_accuracy if 'gps_accuracy_is_set' in args else p.get_gps_dop()
                    if dop is not None and p.latitude is not None and p.longitude is not None:
                        exif_overrides[p.filename] = {
                            'gps': {
                                'latitude': p.latitude,
                                'longitude': p.longitude,
                                'altitude': p.altitude if p.altitude is not None else 0,
                                'dop': dop,
                            }
                        }

                with open(os.path.join(self.opensfm_project_path, "exif_overrides.json"), 'w') as f:
                    f.write(json.dumps(exif_overrides))

            # create config file for OpenSfM
            config = [
                "use_exif_size: no",
                "flann_algorithm: KDTREE", # more stable, faster than KMEANS
                "feature_process_size: %s" % args.resize_to,
                "feature_min_frames: %s" % args.min_num_features,
                "processes: %s" % args.max_concurrency,
                "matching_gps_neighbors: %s" % matcher_neighbors,
                "matching_gps_distance: %s" % args.matcher_distance,
                "depthmap_method: %s" % args.opensfm_depthmap_method,
                "depthmap_resolution: %s" % args.depthmap_resolution,
                "depthmap_min_patch_sd: %s" % args.opensfm_depthmap_min_patch_sd,
                "depthmap_min_consistent_views: %s" % args.opensfm_depthmap_min_consistent_views,
                "optimize_camera_parameters: %s" % ('no' if args.use_fixed_camera_params or args.cameras else 'yes'),
                "undistorted_image_format: tif",
                "bundle_outlier_filtering_type: AUTO",
                "align_orientation_prior: vertical",
                "triangulation_type: ROBUST",
                "bundle_common_position_constraints: %s" % ('no' if reconstruction.multi_camera is None else 'yes'),
            ]

            if args.camera_lens != 'auto':
                config.append("camera_projection_type: %s" % args.camera_lens.upper())

            if not has_gps:
                log.ODM_INFO("No GPS information, using BOW matching")
                use_bow = True

            feature_type = args.feature_type.upper()

            if use_bow:
                config.append("matcher_type: WORDS")

                # Cannot use SIFT with BOW
                if feature_type == "SIFT":
                    log.ODM_WARNING("Using BOW matching, will use HAHOG feature type, not SIFT")
                    feature_type = "HAHOG"
            
            config.append("feature_type: %s" % feature_type)

            if has_alt:
                log.ODM_INFO("Altitude data detected, enabling it for GPS alignment")
                config.append("use_altitude_tag: yes")

            gcp_path = reconstruction.gcp.gcp_path
            if has_alt or gcp_path:
                config.append("align_method: auto")
            else:
                config.append("align_method: orientation_prior")
            
            if args.use_hybrid_bundle_adjustment:
                log.ODM_INFO("Enabling hybrid bundle adjustment")
                config.append("bundle_interval: 100")          # Bundle after adding 'bundle_interval' cameras
                config.append("bundle_new_points_ratio: 1.2")  # Bundle when (new points) / (bundled points) > bundle_new_points_ratio
                config.append("local_bundle_radius: 1")        # Max image graph distance for images to be included in local bundle adjustment
            else:
                config.append("local_bundle_radius: 0")
                
            if gcp_path:
                config.append("bundle_use_gcp: yes")
                if not args.force_gps:
                    config.append("bundle_use_gps: no")
                io.copy(gcp_path, self.path("gcp_list.txt"))
            
            config = config + append_config

            # write config file
            log.ODM_INFO(config)
            config_filename = self.get_config_file_path()
            with open(config_filename, 'w') as fout:
                fout.write("\n".join(config))
        else:
            log.ODM_WARNING("%s already exists, not rerunning OpenSfM setup" % list_path)

    def get_config_file_path(self):
        return io.join_paths(self.opensfm_project_path, 'config.yaml')

    def reconstructed(self):
        if not io.file_exists(self.path("reconstruction.json")):
            return False
        
        with open(self.path("reconstruction.json"), 'r') as f:
            return f.readline().strip() != "[]"

    def extract_metadata(self, rerun=False):
        metadata_dir = self.path("exif")
        if not io.dir_exists(metadata_dir) or rerun:
            self.run('extract_metadata')

    def is_feature_matching_done(self):
        features_dir = self.path("features")
        matches_dir = self.path("matches")

        return io.dir_exists(features_dir) and io.dir_exists(matches_dir)

    def feature_matching(self, rerun=False):
        features_dir = self.path("features")
        matches_dir = self.path("matches")
        
        if not io.dir_exists(features_dir) or rerun:
            self.run('detect_features')
        else:
            log.ODM_WARNING('Detect features already done: %s exists' % features_dir)

        if not io.dir_exists(matches_dir) or rerun:
            self.run('match_features')
        else:
            log.ODM_WARNING('Match features already done: %s exists' % matches_dir)

    def align_reconstructions(self, rerun):
        alignment_file = self.path('alignment_done.txt')
        if not io.file_exists(alignment_file) or rerun:
            log.ODM_INFO("Aligning submodels...")
            meta_data = metadataset.MetaDataSet(self.opensfm_project_path)
            reconstruction_shots = tools.load_reconstruction_shots(meta_data)
            transformations = tools.align_reconstructions(reconstruction_shots,
                                            tools.partial_reconstruction_name,
                                            True)
            tools.apply_transformations(transformations)

            self.touch(alignment_file)
        else:
            log.ODM_WARNING('Found a alignment done progress file in: %s' % alignment_file)

    def touch(self, file):
        with open(file, 'w') as fout:
            fout.write("Done!\n")

    def path(self, *paths):
        return os.path.join(self.opensfm_project_path, *paths)

    def extract_cameras(self, output, rerun=False):
        if not os.path.exists(output) or rerun:
            try:
                reconstruction_file = self.path("reconstruction.json")
                with open(output, 'w') as fout:
                    fout.write(json.dumps(camera.get_cameras_from_opensfm(reconstruction_file), indent=4))
            except Exception as e:
                log.ODM_WARNING("Cannot export cameras to %s. %s." % (output, str(e)))
        else:
            log.ODM_INFO("Already extracted cameras")
    
    def convert_and_undistort(self, rerun=False, imageFilter=None):
        log.ODM_INFO("Undistorting %s ..." % self.opensfm_project_path)
        undistorted_images_path = self.path("undistorted", "images")

        if not io.dir_exists(undistorted_images_path) or rerun:
            cmd = undistort.Command(imageFilter)
            parser = argparse.ArgumentParser()
            cmd.add_arguments(parser)
            cmd.run(parser.parse_args([self.opensfm_project_path]))
        else:
            log.ODM_WARNING("Found an undistorted directory in %s" % undistorted_images_path)


    def update_config(self, cfg_dict):
        cfg_file = self.get_config_file_path()
        log.ODM_INFO("Updating %s" % cfg_file)
        if os.path.exists(cfg_file):
            try:
                with open(cfg_file) as fin:
                    cfg = yaml.safe_load(fin)
                for k, v in cfg_dict.items():
                    cfg[k] = v
                    log.ODM_INFO("%s: %s" % (k, v))
                with open(cfg_file, 'w') as fout:
                    fout.write(yaml.dump(cfg, default_flow_style=False))
            except Exception as e:
                log.ODM_WARNING("Cannot update configuration file %s: %s" % (cfg_file, str(e)))
        else:
            log.ODM_WARNING("Tried to update configuration, but %s does not exist." % cfg_file)

    def name(self):
        return os.path.basename(os.path.abspath(self.path("..")))

def get_submodel_argv(args, submodels_path = None, submodel_name = None):
    """
    Gets argv for a submodel starting from the args passed to the application startup.
    Additionally, if project_name, submodels_path and submodel_name are passed, the function
    handles the <project name> value and --project-path detection / override.
    When all arguments are set to None, --project-path and project name are always removed.

    :return the same as argv, but removing references to --split, 
        setting/replacing --project-path and name
        removing --rerun-from, --rerun, --rerun-all, --sm-cluster
        removing --pc-las, --pc-csv, --pc-ept flags (processing these is wasteful)
        adding --orthophoto-cutline
        adding --dem-euclidean-map
        adding --skip-3dmodel (split-merge does not support 3D model merging)
        tweaking --crop if necessary (DEM merging makes assumption about the area of DEMs and their euclidean maps that require cropping. If cropping is skipped, this leads to errors.)
        removing --gcp (the GCP path if specified is always "gcp_list.txt")
        reading the contents of --cameras
    """
    assure_always = ['orthophoto_cutline', 'dem_euclidean_map', 'skip_3dmodel']
    remove_always = ['split', 'split_overlap', 'rerun_from', 'rerun', 'gcp', 'end_with', 'sm_cluster', 'rerun_all', 'pc_csv', 'pc_las', 'pc_ept']
    read_json_always = ['cameras']

    argv = sys.argv
    result = [argv[0]] # Startup script (/path/to/run.py)

    args_dict = vars(args).copy()
    set_keys = [k[:-len("_is_set")] for k in args_dict.keys() if k.endswith("_is_set")]

    # Handle project name and project path (special case)
    if "name" in set_keys:
        del args_dict["name"]
        set_keys.remove("name")

    if "project_path" in set_keys:
        del args_dict["project_path"]
        set_keys.remove("project_path")

    # Remove parameters
    set_keys = [k for k in set_keys if k not in remove_always]

    # Assure parameters
    for k in assure_always:
        if not k in set_keys:
            set_keys.append(k)
            args_dict[k] = True
    
    # Read JSON always
    for k in read_json_always:
        if k in set_keys:
            try:
                if isinstance(args_dict[k], str):
                    args_dict[k] = io.path_or_json_string_to_dict(args_dict[k])
                if isinstance(args_dict[k], dict):
                    args_dict[k] = json.dumps(args_dict[k])
            except ValueError as e:
                log.ODM_WARNING("Cannot parse/read JSON: {}".format(str(e)))

    # Handle crop (cannot be zero for split/merge)
    if "crop" in set_keys:
        crop_value = float(args_dict["crop"])
        if crop_value == 0:
            crop_value = 0.015625
        args_dict["crop"] = crop_value

    # Populate result
    for k in set_keys:
        result.append("--%s" % k.replace("_", "-"))
        
        # No second value for booleans
        if isinstance(args_dict[k], bool) and args_dict[k] == True:
            continue
        
        result.append(str(args_dict[k]))
    
    if submodels_path:
        result.append("--project-path")
        result.append(submodels_path)

    if submodel_name:
        result.append(submodel_name)

    return result

def get_submodel_args_dict(args):
    submodel_argv = get_submodel_argv(args)
    result = {}

    i = 0
    while i < len(submodel_argv):
        arg = submodel_argv[i]
        next_arg = None if i == len(submodel_argv) - 1 else submodel_argv[i + 1]

        if next_arg and arg.startswith("--"):
            if next_arg.startswith("--"):
                result[arg[2:]] = True
            else:
                result[arg[2:]] = next_arg
                i += 1
        elif arg.startswith("--"):
            result[arg[2:]] = True
        i += 1

    return result


def get_submodel_paths(submodels_path, *paths):
    """
    :return Existing paths for all submodels
    """
    result = []
    if not os.path.exists(submodels_path):
        return result

    for f in os.listdir(submodels_path):
        if f.startswith('submodel'):
            p = os.path.join(submodels_path, f, *paths) 
            if os.path.exists(p):
                result.append(p)
            else:
                log.ODM_WARNING("Missing %s from submodel %s" % (p, f))

    return result

def get_all_submodel_paths(submodels_path, *all_paths):
    """
    :return Existing, multiple paths for all submodels as a nested list (all or nothing for each submodel)
        if a single file is missing from the submodule, no files are returned for that submodel.

        (i.e. get_multi_submodel_paths("path/", "odm_orthophoto.tif", "dem.tif")) -->
                [["path/submodel_0000/odm_orthophoto.tif", "path/submodel_0000/dem.tif"],
                 ["path/submodel_0001/odm_orthophoto.tif", "path/submodel_0001/dem.tif"]]
    """
    result = []
    if not os.path.exists(submodels_path):
        return result

    for f in os.listdir(submodels_path):
        if f.startswith('submodel'):
            all_found = True

            for ap in all_paths:
                p = os.path.join(submodels_path, f, ap) 
                if not os.path.exists(p):
                    log.ODM_WARNING("Missing %s from submodel %s" % (p, f))
                    all_found = False

            if all_found:
                result.append([os.path.join(submodels_path, f, ap) for ap in all_paths])

    return result