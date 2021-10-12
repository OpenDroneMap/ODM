"""
OpenSfM related utils
"""

import os, shutil, sys, json, argparse
import yaml
import numpy as np
import pyproj
from pyproj import CRS
from opendm import io
from opendm import log
from opendm import system
from opendm import context
from opendm import camera
from opendm import location
from opendm.utils import get_depthmap_resolution
from opendm.photo import find_largest_photo_dim
from opensfm.large import metadataset
from opensfm.large import tools
from opensfm.actions import undistort
from opensfm.dataset import DataSet
from opensfm import report
from opendm.multispectral import get_photos_by_band
from opendm.gpu import has_gpus
from opensfm import multiview
from opensfm.actions.export_geocoords import _transform

class OSFMContext:
    def __init__(self, opensfm_project_path):
        self.opensfm_project_path = opensfm_project_path
    
    def run(self, command):
        osfm_bin = os.path.join(context.opensfm_path, 'bin', 'opensfm')
        system.run('%s %s "%s"' %
                    (osfm_bin, command, self.opensfm_project_path))

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
            raise system.ExitException("The program could not process this dataset using the current settings. "
                            "Check that the images have enough overlap, "
                            "that there are enough recognizable features "
                            "and that the images are in focus. "
                            "You could also try to increase the --min-num-features parameter."
                            "The program will now exit.")


    def setup(self, args, images_path, reconstruction, append_config = [], rerun=False):
        """
        Setup a OpenSfM project
        """
        if rerun and io.dir_exists(self.opensfm_project_path):
            shutil.rmtree(self.opensfm_project_path)

        if not io.dir_exists(self.opensfm_project_path):
            system.mkdir_p(self.opensfm_project_path)

        list_path = os.path.join(self.opensfm_project_path, 'image_list.txt')
        if not io.file_exists(list_path) or rerun:

            if reconstruction.multi_camera:
                photos = get_photos_by_band(reconstruction.multi_camera, args.primary_band)
                if len(photos) < 1:
                    raise Exception("Not enough images in selected band %s" % args.primary_band.lower())
                log.ODM_INFO("Reconstruction will use %s images from %s band" % (len(photos), args.primary_band.lower()))
            else:
                photos = reconstruction.photos

            # create file list
            has_alt = True
            has_gps = False
            with open(list_path, 'w') as fout:
                for photo in photos:
                    if not photo.altitude:
                        has_alt = False
                    if photo.latitude is not None and photo.longitude is not None:
                        has_gps = True

                    fout.write('%s\n' % os.path.join(images_path, photo.filename))
            
            # check for image_groups.txt (split-merge)
            image_groups_file = os.path.join(args.project_path, "image_groups.txt")
            if 'split_image_groups_is_set' in args:
                image_groups_file = os.path.abspath(args.split_image_groups)

            if io.file_exists(image_groups_file):
                dst_groups_file = os.path.join(self.opensfm_project_path, "image_groups.txt")
                io.copy(image_groups_file, dst_groups_file)
                log.ODM_INFO("Copied %s to %s" % (image_groups_file, dst_groups_file))
        
            # check for cameras
            if args.cameras:
                try:
                    camera_overrides = camera.get_opensfm_camera_models(args.cameras)
                    with open(os.path.join(self.opensfm_project_path, "camera_models_overrides.json"), 'w') as f:
                        f.write(json.dumps(camera_overrides))
                    log.ODM_INFO("Wrote camera_models_overrides.json to OpenSfM directory")
                except Exception as e:
                    log.ODM_WARNING("Cannot set camera_models_overrides.json: %s" % str(e))

            use_bow = args.matcher_type == "bow"
            feature_type = "SIFT"

            # GPSDOP override if we have GPS accuracy information (such as RTK)
            if 'gps_accuracy_is_set' in args:
                log.ODM_INFO("Forcing GPS DOP to %s for all images" % args.gps_accuracy)
            
            log.ODM_INFO("Writing exif overrides")

            exif_overrides = {}
            for p in photos:
                if 'gps_accuracy_is_set' in args:
                    dop = args.gps_accuracy
                elif p.get_gps_dop() is not None:
                    dop = p.get_gps_dop()
                else:
                    dop = args.gps_accuracy # default value

                if p.latitude is not None and p.longitude is not None:
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

            # Check image masks
            masks = []
            for p in photos:
                if p.mask is not None:
                    masks.append((p.filename, os.path.join(images_path, p.mask)))
            
            if masks:
                log.ODM_INFO("Found %s image masks" % len(masks))
                with open(os.path.join(self.opensfm_project_path, "mask_list.txt"), 'w') as f:
                    for fname, mask in masks:
                        f.write("{} {}\n".format(fname, mask))
            
            # Compute feature_process_size
            feature_process_size = 2048 # default

            if ('resize_to_is_set' in args) and args.resize_to > 0:
                # Legacy
                log.ODM_WARNING("Legacy option --resize-to (this might be removed in a future version). Use --feature-quality instead.")
                feature_process_size = int(args.resize_to)
            else:
                feature_quality_scale = {
                    'ultra': 1,
                    'high': 0.5,
                    'medium': 0.25,
                    'low': 0.125,
                    'lowest': 0.0675,
                }

                max_dim = find_largest_photo_dim(photos)

                if max_dim > 0:
                    log.ODM_INFO("Maximum photo dimensions: %spx" % str(max_dim))
                    feature_process_size = int(max_dim * feature_quality_scale[args.feature_quality])
                    log.ODM_INFO("Photo dimensions for feature extraction: %ipx" % feature_process_size)
                else:
                    log.ODM_WARNING("Cannot compute max image dimensions, going with defaults")

            depthmap_resolution = get_depthmap_resolution(args, photos)

            # create config file for OpenSfM
            config = [
                "use_exif_size: no",
                "flann_algorithm: KDTREE", # more stable, faster than KMEANS
                "feature_process_size: %s" % feature_process_size,
                "feature_min_frames: %s" % args.min_num_features,
                "processes: %s" % args.max_concurrency,
                "matching_gps_neighbors: %s" % args.matcher_neighbors,
                "matching_gps_distance: %s" % args.matcher_distance,
                "optimize_camera_parameters: %s" % ('no' if args.use_fixed_camera_params or args.cameras else 'yes'),
                "undistorted_image_format: tif",
                "bundle_outlier_filtering_type: AUTO",
                "sift_peak_threshold: 0.066",
                "align_orientation_prior: vertical",
                "triangulation_type: ROBUST",
                "retriangulation_ratio: 2",
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
            
            # GPU acceleration?
            if has_gpus() and feature_type == "SIFT":
                log.ODM_INFO("Using GPU for extracting SIFT features")
                log.ODM_INFO("--min-num-features will be ignored")
                feature_type = "SIFT_GPU"
            
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
            
            # We impose our own reference_lla
            if reconstruction.is_georeferenced():
                self.write_reference_lla(reconstruction.georef.utm_east_offset, reconstruction.georef.utm_north_offset, reconstruction.georef.proj4())
        else:
            log.ODM_WARNING("%s already exists, not rerunning OpenSfM setup" % list_path)

    def get_config_file_path(self):
        return os.path.join(self.opensfm_project_path, 'config.yaml')

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
                                            False)
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
    
    def convert_and_undistort(self, rerun=False, imageFilter=None, image_list=None, runId="nominal"):
        log.ODM_INFO("Undistorting %s ..." % self.opensfm_project_path)
        done_flag_file = self.path("undistorted", "%s_done.txt" % runId)

        if not io.file_exists(done_flag_file) or rerun:
            ds = DataSet(self.opensfm_project_path)

            if image_list is not None:
                ds._set_image_list(image_list)

            undistort.run_dataset(ds, "reconstruction.json", 
                                  0, None, "undistorted", imageFilter)
            
            self.touch(done_flag_file)
        else:
            log.ODM_WARNING("Already undistorted (%s)" % runId)

    def restore_reconstruction_backup(self):
        if os.path.exists(self.recon_backup_file()):
            # This time export the actual reconstruction.json
            # (containing only the primary band)
            if os.path.exists(self.recon_file()):
                os.remove(self.recon_file())
            os.replace(self.recon_backup_file(), self.recon_file())
            log.ODM_INFO("Restored reconstruction.json")

    def backup_reconstruction(self):
        if os.path.exists(self.recon_backup_file()):
            os.remove(self.recon_backup_file())
            
        log.ODM_INFO("Backing up reconstruction")
        shutil.copyfile(self.recon_file(), self.recon_backup_file())

    def recon_backup_file(self):
        return self.path("reconstruction.backup.json")
    
    def recon_file(self):
        return self.path("reconstruction.json")

    def add_shots_to_reconstruction(self, p2s):
        with open(self.recon_file()) as f:
            reconstruction = json.loads(f.read())

        # Augment reconstruction.json
        for recon in reconstruction:
            shots = recon['shots']
            sids = list(shots)
            
            for shot_id in sids:
                secondary_photos = p2s.get(shot_id)
                if secondary_photos is None:
                    log.ODM_WARNING("Cannot find secondary photos for %s" % shot_id)
                    continue

                for p in secondary_photos:
                    shots[p.filename] = shots[shot_id]

        with open(self.recon_file(), 'w') as f:
            f.write(json.dumps(reconstruction))


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

    def export_stats(self, rerun=False):
        log.ODM_INFO("Export reconstruction stats")
        stats_path = self.path("stats", "stats.json")
        if not os.path.exists(stats_path) or rerun:
            self.run("compute_statistics --diagram_max_points 100000")
        else:
            log.ODM_WARNING("Found existing reconstruction stats %s" % stats_path)

    def export_report(self, report_path, odm_stats, rerun=False):
        log.ODM_INFO("Exporting report to %s" % report_path)

        osfm_report_path = self.path("stats", "report.pdf")
        if not os.path.exists(report_path) or rerun:
            data = DataSet(self.opensfm_project_path)
            pdf_report = report.Report(data, odm_stats)
            pdf_report.generate_report()
            pdf_report.save_report("report.pdf")
            
            if os.path.exists(osfm_report_path):
                shutil.move(osfm_report_path, report_path)
            else:
                log.ODM_WARNING("Report could not be generated")
        else:
            log.ODM_WARNING("Report %s already exported" % report_path)
    
    def write_reference_lla(self, offset_x, offset_y, proj4):
        reference_lla = self.path("reference_lla.json")

        longlat = CRS.from_epsg("4326")
        lon, lat = location.transform2(CRS.from_proj4(proj4), longlat, offset_x, offset_y)

        with open(reference_lla, 'w') as f:
            f.write(json.dumps({
                'latitude': lat,
                'longitude': lon,
                'altitude': 0.0
            }, indent=4))
        
        log.ODM_INFO("Wrote reference_lla.json")

    def ground_control_points(self, proj4):
        """
        Load ground control point information.
        """
        gcp_stats_file = self.path("stats", "ground_control_points.json")

        if not io.file_exists(gcp_stats_file):
            return []
        
        gcps_stats = {}
        try:
            with open(gcp_stats_file) as f:
                gcps_stats = json.loads(f.read())
        except:
            log.ODM_INFO("Cannot parse %s" % gcp_stats_file)

        if not gcps_stats:
            return []
        
        ds = DataSet(self.opensfm_project_path)
        reference = ds.load_reference()
        projection = pyproj.Proj(proj4)

        result = []
        for gcp in gcps_stats:
            geocoords = _transform(gcp['coordinates'], reference, projection)
            result.append({
                'id': gcp['id'],
                'observations': gcp['observations'],
                'coordinates': geocoords,
                'error': gcp['error']
            })

        return result
    

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
        removing --pc-las, --pc-csv, --pc-ept, --tiles flags (processing these is wasteful)
        adding --orthophoto-cutline
        adding --dem-euclidean-map
        adding --skip-3dmodel (split-merge does not support 3D model merging)
        tweaking --crop if necessary (DEM merging makes assumption about the area of DEMs and their euclidean maps that require cropping. If cropping is skipped, this leads to errors.)
        removing --gcp (the GCP path if specified is always "gcp_list.txt")
        reading the contents of --cameras
        reading the contents of --boundary
    """
    assure_always = ['orthophoto_cutline', 'dem_euclidean_map', 'skip_3dmodel', 'skip_report']
    remove_always = ['split', 'split_overlap', 'rerun_from', 'rerun', 'gcp', 'end_with', 'sm_cluster', 'rerun_all', 'pc_csv', 'pc_las', 'pc_ept', 'tiles', 'copy-to', 'cog']
    read_json_always = ['cameras', 'boundary']

    argv = sys.argv

    # Startup script (/path/to/run.py)
    startup_script = argv[0]

    # On Windows, make sure we always invoke the "run.bat" file
    if sys.platform == 'win32':
        startup_script_dir = os.path.dirname(startup_script)
        startup_script = os.path.join(startup_script_dir, "run")

    result = [startup_script] 

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
