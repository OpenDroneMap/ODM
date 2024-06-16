import argparse
import json
from opendm import context
from opendm import io
from opendm import log
from appsettings import SettingsParser
from pyodm import Node, exceptions
import os
import sys

# parse arguments
processopts = ['dataset', 'split', 'merge', 'opensfm', 'openmvs', 'odm_filterpoints',
               'odm_meshing', 'mvs_texturing', 'odm_georeferencing',
               'odm_dem', 'odm_orthophoto', 'odm_report', 'odm_postprocess']

rerun_stages = {
    '3d_tiles': 'odm_postprocess',
    'align': 'odm_georeferencing',
    'auto_boundary': 'odm_filterpoints',
    'auto_boundary_distance': 'odm_filterpoints',
    'bg_removal': 'dataset',
    'boundary': 'odm_filterpoints',
    'build_overviews': 'odm_orthophoto',
    'camera_lens': 'dataset',
    'cameras': 'dataset',
    'cog': 'odm_dem',
    'copy_to': 'odm_postprocess',
    'crop': 'odm_georeferencing',
    'dem_decimation': 'odm_dem',
    'dem_euclidean_map': 'odm_dem',
    'dem_gapfill_steps': 'odm_dem',
    'dem_resolution': 'odm_dem',
    'dsm': 'odm_dem',
    'dtm': 'odm_dem',
    'end_with': None,
    'fast_orthophoto': 'odm_filterpoints',
    'feature_quality': 'opensfm',
    'feature_type': 'opensfm',
    'force_gps': 'opensfm',
    'gcp': 'dataset',
    'geo': 'dataset',
    'gltf': 'mvs_texturing',
    'gps_accuracy': 'dataset',
    'help': None,
    'ignore_gsd': 'opensfm',
    'matcher_neighbors': 'opensfm',
    'matcher_order': 'opensfm',
    'matcher_type': 'opensfm',
    'max_concurrency': None,
    'merge': 'Merge',
    'mesh_octree_depth': 'odm_meshing',
    'mesh_size': 'odm_meshing',
    'min_num_features': 'opensfm',
    'name': None,
    'no_gpu': None,
    'optimize_disk_space': None,
    'orthophoto_compression': 'odm_orthophoto',
    'orthophoto_cutline': 'odm_orthophoto',
    'orthophoto_kmz': 'odm_orthophoto',
    'orthophoto_no_tiled': 'odm_orthophoto',
    'orthophoto_png': 'odm_orthophoto',
    'orthophoto_resolution': 'odm_orthophoto',
    'pc_classify': 'odm_georeferencing',
    'pc_copc': 'odm_georeferencing',
    'pc_csv': 'odm_georeferencing',
    'pc_ept': 'odm_georeferencing',
    'pc_filter': 'openmvs',
    'pc_las': 'odm_georeferencing',
    'pc_quality': 'opensfm',
    'pc_rectify': 'odm_georeferencing',
    'pc_sample': 'odm_filterpoints',
    'pc_skip_geometric': 'openmvs',
    'primary_band': 'dataset',
    'project_path': None,
    'radiometric_calibration': 'opensfm',
    'rerun': None,
    'rerun_all': None,
    'rerun_from': None,
    'rolling_shutter': 'opensfm',
    'rolling_shutter_readout': 'opensfm',
    'sfm_algorithm': 'opensfm',
    'sfm_no_partial': 'opensfm',
    'skip_3dmodel': 'odm_meshing',
    'skip_band_alignment': 'opensfm',
    'skip_orthophoto': 'odm_orthophoto',
    'skip_report': 'odm_report',
    'sky_removal': 'dataset',
    'sm_cluster': 'split',
    'sm_no_align': 'split',
    'smrf_scalar': 'odm_dem',
    'smrf_slope': 'odm_dem',
    'smrf_threshold': 'odm_dem',
    'smrf_window': 'odm_dem',
    'split': 'split',
    'split_image_groups': 'split',
    'split_overlap': 'split',
    'texturing_keep_unseen_faces': 'mvs_texturing',
    'texturing_single_material': 'mvs_texturing',
    'texturing_skip_global_seam_leveling': 'mvs_texturing',
    'tiles': 'odm_dem',
    'use_3dmesh': 'mvs_texturing',
    'use_exif': 'dataset',
    'use_fixed_camera_params': 'opensfm',
    'use_hybrid_bundle_adjustment': 'opensfm',
    'version': None,
    'video_limit': 'dataset',
    'video_resolution': 'dataset',
}

with open(os.path.join(context.root_path, 'VERSION')) as version_file:
    __version__ = version_file.read().strip()


def path_or_json_string(string):
    try:
        return io.path_or_json_string_to_dict(string)
    except ValueError as e:
        raise argparse.ArgumentTypeError("{0}".format(str(e)))

# Django URL validation regex
def url_string(string):
    import re
    regex = re.compile(
        r'^(?:http|ftp)s?://' # http:// or https://
        r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.?)+(?:[A-Z]{2,6}\.?|[A-Z0-9-]{2,}\.?)|' #domain...
        r'localhost|' #localhost...
        r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})' # ...or ip
        r'(?::\d+)?' # optional port
        r'(?:/?|[/?]\S+)$', re.IGNORECASE)
        
    if re.match(regex, string) is None:
        raise argparse.ArgumentTypeError("%s is not a valid URL. The URL must be in the format: http(s)://host[:port]/[?token=]" % string)
    return string

class RerunFrom(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        setattr(namespace, self.dest, processopts[processopts.index(values):])
        setattr(namespace, self.dest + '_is_set', True)

class StoreTrue(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        setattr(namespace, self.dest, True)
        setattr(namespace, self.dest + '_is_set', True)

class StoreValue(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        setattr(namespace, self.dest, values)
        setattr(namespace, self.dest + '_is_set', True)

args = None

def config(argv=None, parser=None):
    global args

    if args is not None and argv is None:
        return args

    if sys.platform == 'win32':
        usage_bin = 'run'
    else:
        usage_bin = 'run.sh'

    if parser is None:
        parser = SettingsParser(description='ODM is a command line toolkit to generate maps, point clouds, 3D models and DEMs from drone, balloon or kite images.',
                            usage='%s [options] <dataset name>' % usage_bin,
                            yaml_file=open(context.settings_path))
    
    parser.add_argument('--project-path',
                        metavar='<path>',
                        action=StoreValue,
                        help='Path to the project folder. Your project folder should contain subfolders for each dataset. Each dataset should have an "images" folder.')
    parser.add_argument('name',
                        metavar='<dataset name>',
                        action=StoreValue,
                        type=str,
                        default='code',
                        nargs='?',
                        help='Name of dataset (i.e subfolder name within project folder). Default: %(default)s')

    parser.add_argument('--end-with', '-e',
                        metavar='<string>',
                        action=StoreValue,
                        default='odm_postprocess',
                        choices=processopts,
                        help='End processing at this stage. Can be one of: %(choices)s. Default: %(default)s')

    rerun = parser.add_mutually_exclusive_group()

    rerun.add_argument('--rerun', '-r',
                       metavar='<string>',
                       action=StoreValue,
                       choices=processopts,
                       help=('Rerun this stage only and stop. Can be one of: %(choices)s. Default: %(default)s'))

    rerun.add_argument('--rerun-all',
                       action=StoreTrue,
                       nargs=0,
                       default=False,
                       help='Permanently delete all previous results and rerun the processing pipeline.')

    rerun.add_argument('--rerun-from',
                       action=RerunFrom,
                       metavar='<string>',
                       choices=processopts,
                       help=('Rerun processing from this stage. Can be one of: %(choices)s. Default: %(default)s'))

    parser.add_argument('--min-num-features',
                        metavar='<integer>',
                        action=StoreValue,
                        default=10000,
                        type=int,
                        help=('Minimum number of features to extract per image. '
                              'More features can be useful for finding more matches between images, '
                              'potentially allowing the reconstruction of areas with little overlap or insufficient features. '
                              'More features also slow down processing. Default: %(default)s'))
    
    parser.add_argument('--feature-type',
                        metavar='<string>',
                        action=StoreValue,
                        default='dspsift',
                        choices=['akaze', 'dspsift', 'hahog', 'orb', 'sift'],
                        help=('Choose the algorithm for extracting keypoints and computing descriptors. '
                            'Can be one of: %(choices)s. Default: '
                            '%(default)s'))
    
    parser.add_argument('--feature-quality',
                        metavar='<string>',
                        action=StoreValue,
                        default='high',
                        choices=['ultra', 'high', 'medium', 'low', 'lowest'],
                        help=('Set feature extraction quality. Higher quality generates better features, but requires more memory and takes longer. '
                            'Can be one of: %(choices)s. Default: '
                            '%(default)s'))
    
    parser.add_argument('--matcher-type',
                        metavar='<string>',
                        action=StoreValue,
                        default='flann',
                        choices=['bow', 'bruteforce', 'flann'],
                        help=('Matcher algorithm, Fast Library for Approximate Nearest Neighbors or Bag of Words. FLANN is slower, but more stable. BOW is faster, but can sometimes miss valid matches. BRUTEFORCE is very slow but robust.'
                            'Can be one of: %(choices)s. Default: '
                            '%(default)s'))

    parser.add_argument('--matcher-neighbors',
                        metavar='<positive integer>',
                        action=StoreValue,
                        default=0,
                        type=int,
                        help='Perform image matching with the nearest images based on GPS exif data. Set to 0 to match by triangulation. Default: %(default)s')
    
    parser.add_argument('--matcher-order',
                        metavar='<positive integer>',
                        action=StoreValue,
                        default=0,
                        type=int,
                        help='Perform image matching with the nearest N images based on image filename order. Can speed up processing of sequential images, such as those extracted from video. It is applied only on non-georeferenced datasets. Set to 0 to disable. Default: %(default)s')

    parser.add_argument('--use-fixed-camera-params',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Turn off camera parameter optimization during bundle adjustment. This can be sometimes useful for improving results that exhibit doming/bowling or when images are taken with a rolling shutter camera. Default: %(default)s')

    parser.add_argument('--cameras',
                        default='',
                        metavar='<json>',
                        action=StoreValue,
                        type=path_or_json_string,
                        help='Use the camera parameters computed from '
                             'another dataset instead of calculating them. '
                             'Can be specified either as path to a cameras.json file or as a '
                             'JSON string representing the contents of a '
                             'cameras.json file. Default: %(default)s')

    parser.add_argument('--camera-lens',
            metavar='<string>',
            action=StoreValue,
            default='auto',
            choices=['auto', 'perspective', 'brown', 'fisheye', 'fisheye_opencv', 'spherical', 'equirectangular', 'dual'],
            help=('Set a camera projection type. Manually setting a value '
                'can help improve geometric undistortion. By default the application '
                'tries to determine a lens type from the images metadata. Can be one of: %(choices)s. Default: '
                '%(default)s'))

    parser.add_argument('--radiometric-calibration',
            metavar='<string>',
            action=StoreValue,
            default='none',
            choices=['none', 'camera', 'camera+sun'],
            help=('Set the radiometric calibration to perform on images. '
                'When processing multispectral and thermal images you should set this option '
                'to obtain reflectance/temperature values (otherwise you will get digital number values). '
                '[camera] applies black level, vignetting, row gradient gain/exposure compensation (if appropriate EXIF tags are found) and computes absolute temperature values. '
                '[camera+sun] is experimental, applies all the corrections of [camera], plus compensates for spectral radiance registered via a downwelling light sensor (DLS) taking in consideration the angle of the sun. '
                'Can be one of: %(choices)s. Default: '
                '%(default)s'))

    parser.add_argument('--max-concurrency',
                        metavar='<positive integer>',
                        action=StoreValue,
                        default=context.num_cores,
                        type=int,
                        help=('The maximum number of processes to use in various '
                              'processes. Peak memory requirement is ~1GB per '
                              'thread and 2 megapixel image resolution. Default: %(default)s'))

    parser.add_argument('--use-hybrid-bundle-adjustment',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Run local bundle adjustment for every image added to the reconstruction and a global '
                             'adjustment every 100 images. Speeds up reconstruction for very large datasets. Default: %(default)s')

    parser.add_argument('--sfm-algorithm',
                    metavar='<string>',
                    action=StoreValue,
                    default='incremental',
                    choices=['incremental', 'triangulation', 'planar'],
                    help=('Choose the structure from motion algorithm. For aerial datasets, if camera GPS positions and angles are available, triangulation can generate better results. For planar scenes captured at fixed altitude with nadir-only images, planar can be much faster. '
                        'Can be one of: %(choices)s. Default: '
                        '%(default)s'))

    parser.add_argument('--sfm-no-partial',
                action=StoreTrue,
                nargs=0,
                default=False,
                help='Do not attempt to merge partial reconstructions. This can happen when images do not have sufficient overlap or are isolated. Default: %(default)s')

    parser.add_argument('--sky-removal',
                action=StoreTrue,
                nargs=0,
                default=False,
                help='Automatically compute image masks using AI to remove the sky. Experimental. Default: %(default)s')
    
    parser.add_argument('--bg-removal',
                action=StoreTrue,
                nargs=0,
                default=False,
                help='Automatically compute image masks using AI to remove the background. Experimental. Default: %(default)s')

    parser.add_argument('--use-3dmesh',
                    action=StoreTrue,
                    nargs=0,
                    default=False,
                    help='Use a full 3D mesh to compute the orthophoto instead of a 2.5D mesh. This option is a bit faster and provides similar results in planar areas. Default: %(default)s')

    parser.add_argument('--skip-3dmodel',
                    action=StoreTrue,
                    nargs=0,
                    default=False,
                    help='Skip generation of a full 3D model. This can save time if you only need 2D results such as orthophotos and DEMs. Default: %(default)s')
    
    parser.add_argument('--skip-report',
                    action=StoreTrue,
                    nargs=0,
                    default=False,
                    help='Skip generation of PDF report. This can save time if you don\'t need a report. Default: %(default)s')
    
    parser.add_argument('--skip-orthophoto',
                    action=StoreTrue,
                    nargs=0,
                    default=False,
                    help='Skip generation of the orthophoto. This can save time if you only need 3D results or DEMs. Default: %(default)s')
    
    parser.add_argument('--ignore-gsd',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Ignore Ground Sampling Distance (GSD).'
                        'A memory and processor hungry change relative to the default behavior if set to true. '
                        'Ordinarily, GSD estimates are used to cap the maximum resolution of image outputs and resizes images when necessary, resulting in faster processing and lower memory usage. '
                        'Since GSD is an estimate, sometimes ignoring it can result in slightly better image output quality. '
                        'Never set --ignore-gsd to true unless you are positive you need it, and even then: do not use it. Default: %(default)s')
    
    parser.add_argument('--no-gpu',
                    action=StoreTrue,
                    nargs=0,
                    default=False,
                    help='Do not use GPU acceleration, even if it\'s available. Default: %(default)s')
    
    parser.add_argument('--mesh-size',
                        metavar='<positive integer>',
                        action=StoreValue,
                        default=200000,
                        type=int,
                        help=('The maximum vertex count of the output mesh. '
                              'Default: %(default)s'))

    parser.add_argument('--mesh-octree-depth',
                        metavar='<integer: 1 <= x <= 14>',
                        action=StoreValue,
                        default=11,
                        type=int,
                        help=('Octree depth used in the mesh reconstruction, '
                              'increase to get more vertices, recommended '
                              'values are 8-12. Default: %(default)s'))

    parser.add_argument('--fast-orthophoto',
                action=StoreTrue,
                nargs=0,
                default=False,
                help='Skips dense reconstruction and 3D model generation. '
                'It generates an orthophoto directly from the sparse reconstruction. '
                'If you just need an orthophoto and do not need a full 3D model, turn on this option. Default: %(default)s')

    parser.add_argument('--crop',
                    metavar='<positive float>',
                    action=StoreValue,
                    default=3,
                    type=float,
                    help=('Automatically crop image outputs by creating a smooth buffer '
                          'around the dataset boundaries, shrunk by N meters. '
                          'Use 0 to disable cropping. '
                          'Default: %(default)s'))

    parser.add_argument('--boundary',
                    default='',
                    metavar='<json>',
                    action=StoreValue,
                    type=path_or_json_string,
                    help='GeoJSON polygon limiting the area of the reconstruction. '
                            'Can be specified either as path to a GeoJSON file or as a '
                            'JSON string representing the contents of a '
                            'GeoJSON file. Default: %(default)s')

    parser.add_argument('--auto-boundary',
                    action=StoreTrue,
                    nargs=0,
                    default=False,
                    help='Automatically set a boundary using camera shot locations to limit the area of the reconstruction. '
                    'This can help remove far away background artifacts (sky, background landscapes, etc.). See also --boundary. '
                    'Default: %(default)s')

    parser.add_argument('--auto-boundary-distance',
                    metavar='<positive float>',
                    action=StoreValue,
                    type=float,
                    default=0,
                    help='Specify the distance between camera shot locations and the outer edge of the boundary when computing the boundary with --auto-boundary. Set to 0 to automatically choose a value. '
                         'Default: %(default)s')

    parser.add_argument('--pc-quality',
                    metavar='<string>',
                    action=StoreValue,
                    default='medium',
                    choices=['ultra', 'high', 'medium', 'low', 'lowest'],
                    help=('Set point cloud quality. Higher quality generates better, denser point clouds, but requires more memory and takes longer. Each step up in quality increases processing time roughly by a factor of 4x.'
                        'Can be one of: %(choices)s. Default: '
                        '%(default)s'))

    parser.add_argument('--pc-classify',
            action=StoreTrue,
            nargs=0,
            default=False,
            help='Classify the point cloud outputs. '
            'You can control the behavior of this option by tweaking the --dem-* parameters. '
            'Default: '
            '%(default)s')

    parser.add_argument('--pc-csv',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Export the georeferenced point cloud in CSV format. Default: %(default)s')
    
    parser.add_argument('--pc-las',
                action=StoreTrue,
                nargs=0,
                default=False,
                help='Export the georeferenced point cloud in LAS format. Default: %(default)s')

    parser.add_argument('--pc-ept',
                action=StoreTrue,
                nargs=0,
                default=False,
                help='Export the georeferenced point cloud in Entwine Point Tile (EPT) format. Default: %(default)s')

    parser.add_argument('--pc-copc',
                action=StoreTrue,
                nargs=0,
                default=False,
                help='Save the georeferenced point cloud in Cloud Optimized Point Cloud (COPC) format. Default: %(default)s')

    parser.add_argument('--pc-filter',
                        metavar='<positive float>',
                        action=StoreValue,
                        type=float,
                        default=5,
                        help='Filters the point cloud by removing points that deviate more than N standard deviations from the local mean. Set to 0 to disable filtering. '
                             'Default: %(default)s')
    
    parser.add_argument('--pc-sample',
                        metavar='<positive float>',
                        action=StoreValue,
                        type=float,
                        default=0,
                        help='Filters the point cloud by keeping only a single point around a radius N (in meters). This can be useful to limit the output resolution of the point cloud and remove duplicate points. Set to 0 to disable sampling. '
                             'Default: %(default)s')

    parser.add_argument('--pc-skip-geometric',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Geometric estimates improve the accuracy of the point cloud by computing geometrically consistent depthmaps but may not be usable in larger datasets. This flag disables geometric estimates. '
                             'Default: %(default)s')

    parser.add_argument('--smrf-scalar',
                        metavar='<positive float>',
                        action=StoreValue,
                        type=float,
                        default=1.25,
                        help='Simple Morphological Filter elevation scalar parameter. '
                             'Default: %(default)s')

    parser.add_argument('--smrf-slope',
        metavar='<positive float>',
        action=StoreValue,
        type=float,
        default=0.15,
        help='Simple Morphological Filter slope parameter (rise over run). '
                'Default: %(default)s')
    
    parser.add_argument('--smrf-threshold',
        metavar='<positive float>',
        action=StoreValue,
        type=float,
        default=0.5,
        help='Simple Morphological Filter elevation threshold parameter (meters). '
                'Default: %(default)s')
    
    parser.add_argument('--smrf-window',
        metavar='<positive float>',
        action=StoreValue,
        type=float,
        default=18.0,
        help='Simple Morphological Filter window radius parameter (meters). '
                'Default: %(default)s')

    parser.add_argument('--texturing-skip-global-seam-leveling',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help=('Skip normalization of colors across all images. Useful when processing radiometric data. Default: %(default)s'))

    parser.add_argument('--texturing-keep-unseen-faces',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help=('Keep faces in the mesh that are not seen in any camera. '
                              'Default:  %(default)s'))

    parser.add_argument('--texturing-single-material',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help=('Generate OBJs that have a single material and a single texture file instead of multiple ones. '
                              'Default:  %(default)s'))

    parser.add_argument('--gltf',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help=('Generate single file Binary glTF (GLB) textured models. '
                              'Default:  %(default)s'))

    parser.add_argument('--gcp',
                        metavar='<path string>',
                        action=StoreValue,
                        default=None,
                        help=('Path to the file containing the ground control '
                              'points used for georeferencing. '
                              'The file needs to '
                              'use the following format: \n'
                              'EPSG:<code> or <+proj definition>\n'
                              'geo_x geo_y geo_z im_x im_y image_name [gcp_name] [extra1] [extra2]\n'
                              'Default: %(default)s'))

    parser.add_argument('--geo',
                        metavar='<path string>',
                        action=StoreValue,
                        default=None,
                        help=('Path to the image geolocation file containing the camera center coordinates used for georeferencing. '
                              'If you don\'t have values for yaw/pitch/roll you can set them to 0. '
                              'The file needs to '
                              'use the following format: \n'
                              'EPSG:<code> or <+proj definition>\n'
                              'image_name geo_x geo_y geo_z [yaw (degrees)] [pitch (degrees)] [roll (degrees)] [horz accuracy (meters)] [vert accuracy (meters)]\n'
                              'Default: %(default)s'))
    
    parser.add_argument('--align',
                    metavar='<path string>',
                    action=StoreValue,
                    default=None,
                    help=('Path to a GeoTIFF DEM or a LAS/LAZ point cloud '
                            'that the reconstruction outputs should be automatically aligned to. Experimental. '
                            'Default: %(default)s'))

    parser.add_argument('--use-exif',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help=('Use this tag if you have a GCP File but '
                              'want to use the EXIF information for georeferencing instead. Default: %(default)s'))

    parser.add_argument('--dtm',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Use this tag to build a DTM (Digital Terrain Model, ground only) using a simple '
                             'morphological filter. Check the --dem* and --smrf* parameters for finer tuning. Default: %(default)s')

    parser.add_argument('--dsm',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Use this tag to build a DSM (Digital Surface Model, ground + objects) using a progressive '
                             'morphological filter. Check the --dem* parameters for finer tuning. Default: %(default)s')

    parser.add_argument('--dem-gapfill-steps',
                        metavar='<positive integer>',
                        action=StoreValue,
                        default=3,
                        type=int,
                        help='Number of steps used to fill areas with gaps. Set to 0 to disable gap filling. '
                             'Starting with a radius equal to the output resolution, N different DEMs are generated with '
                             'progressively bigger radius using the inverse distance weighted (IDW) algorithm '
                             'and merged together. Remaining gaps are then merged using nearest neighbor interpolation. '
                             'Default: %(default)s')

    parser.add_argument('--dem-resolution',
                        metavar='<float>',
                        action=StoreValue,
                        type=float,
                        default=5,
                        help='DSM/DTM resolution in cm / pixel. Note that this value is capped by a ground sampling distance (GSD) estimate.'
                             ' Default: %(default)s')

    parser.add_argument('--dem-decimation',
                        metavar='<positive integer>',
                        action=StoreValue,
                        default=1,
                        type=int,
                        help='Decimate the points before generating the DEM. 1 is no decimation (full quality). '
                             '100 decimates ~99%% of the points. Useful for speeding up generation of DEM results in very large datasets. Default: %(default)s')
    
    parser.add_argument('--dem-euclidean-map',
            action=StoreTrue,
            nargs=0,
            default=False,
            help='Computes an euclidean raster map for each DEM. '
            'The map reports the distance from each cell to the nearest '
            'NODATA value (before any hole filling takes place). '
            'This can be useful to isolate the areas that have been filled. '
            'Default: '
            '%(default)s')

    parser.add_argument('--orthophoto-resolution',
                        metavar='<float > 0.0>',
                        action=StoreValue,
                        default=5,
                        type=float,
                        help=('Orthophoto resolution in cm / pixel. Note that this value is capped by a ground sampling distance (GSD) estimate.'
                              'Default: %(default)s'))

    parser.add_argument('--orthophoto-no-tiled',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Set this parameter if you want a striped GeoTIFF. '
                             'Default: %(default)s')

    parser.add_argument('--orthophoto-png',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Set this parameter if you want to generate a PNG rendering of the orthophoto. '
                             'Default: %(default)s')
    
    parser.add_argument('--orthophoto-kmz',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Set this parameter if you want to generate a Google Earth (KMZ) rendering of the orthophoto. '
                             'Default: %(default)s')    

    parser.add_argument('--orthophoto-compression',
                        metavar='<string>',
                        action=StoreValue,
                        type=str,
                        choices=['JPEG', 'LZW', 'PACKBITS', 'DEFLATE', 'LZMA', 'NONE'],
                        default='DEFLATE',
                        help='Set the compression to use for orthophotos. Can be one of: %(choices)s. Default: %(default)s')
    
    parser.add_argument('--orthophoto-cutline',
            action=StoreTrue,
            nargs=0,
            default=False,
            help='Generates a polygon around the cropping area '
            'that cuts the orthophoto around the edges of features. This polygon '
            'can be useful for stitching seamless mosaics with multiple overlapping orthophotos. '
            'Default: '
            '%(default)s')

    parser.add_argument('--tiles',
                    action=StoreTrue,
                    nargs=0,
                    default=False,
                    help='Generate static tiles for orthophotos and DEMs that are '
                         'suitable for viewers like Leaflet or OpenLayers. '
                         'Default: %(default)s')

    parser.add_argument('--3d-tiles',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Generate OGC 3D Tiles outputs. Default: %(default)s')

    parser.add_argument('--rolling-shutter',
                    action=StoreTrue,
                    nargs=0,
                    default=False,
                    help='Turn on rolling shutter correction. If the camera '
                         'has a rolling shutter and the images were taken in motion, you can turn on this option '
                         'to improve the accuracy of the results. See also --rolling-shutter-readout. '
                         'Default: %(default)s')

    parser.add_argument('--rolling-shutter-readout',
                        type=float,
                        action=StoreValue,
                        metavar='<positive integer>',
                        default=0,
                    help='Override the rolling shutter readout time for your camera sensor (in milliseconds), instead of using the rolling shutter readout database. ' 
                    'Note that not all cameras are present in the database. Set to 0 to use the database value. '
                    'Default: %(default)s')

    parser.add_argument('--build-overviews',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Build orthophoto overviews for faster display in programs such as QGIS. Default: %(default)s')

    parser.add_argument('--cog',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Create Cloud-Optimized GeoTIFFs instead of normal GeoTIFFs. Default: %(default)s')

    parser.add_argument('--copy-to',
                        metavar='<path>',
                        action=StoreValue,
                        help='Copy output results to this folder after processing.')

    parser.add_argument('--version',
                        action='version',
                        version='ODM {0}'.format(__version__),
                        help='Displays version number and exits. ')

    parser.add_argument('--video-limit',
                        type=int,
                        action=StoreValue,
                        default=500,
                        metavar='<positive integer>',
                        help='Maximum number of frames to extract from video files for processing. Set to 0 for no limit. Default: %(default)s')

    parser.add_argument('--video-resolution',
                        type=int,
                        action=StoreValue,
                        default=4000,
                        metavar='<positive integer>',
                        help='The maximum output resolution of extracted video frames in pixels. Default: %(default)s')
    
    parser.add_argument('--split',
                        type=int,
                        action=StoreValue,
                        default=999999,
                        metavar='<positive integer>',
                        help='Average number of images per submodel. When '
                                'splitting a large dataset into smaller '
                                'submodels, images are grouped into clusters. '
                                'This value regulates the number of images that '
                                'each cluster should have on average. Default: %(default)s')

    parser.add_argument('--split-overlap',
                        type=float,
                        action=StoreValue,
                        metavar='<positive integer>',
                        default=150,
                        help='Radius of the overlap between submodels. '
                        'After grouping images into clusters, images '
                        'that are closer than this radius to a cluster '
                        'are added to the cluster. This is done to ensure '
                        'that neighboring submodels overlap. Default: %(default)s')

    parser.add_argument('--split-image-groups',
                        metavar='<path string>',
                        action=StoreValue,
                        default=None,
                        help=('Path to the image groups file that controls how images should be split into groups. '
                              'The file needs to use the following format: \n'
                              'image_name group_name\n'
                              'Default: %(default)s'))

    parser.add_argument('--sm-no-align',
                    action=StoreTrue,
                    nargs=0,
                    default=False,
                    help='Skip alignment of submodels in split-merge. Useful if GPS is good enough on very large datasets. Default: %(default)s')

    parser.add_argument('--sm-cluster',
                        metavar='<string>',
                        action=StoreValue,
                        type=url_string,
                        default=None,
                        help='URL to a ClusterODM instance '
                            'for distributing a split-merge workflow on '
                            'multiple nodes in parallel. '
                            'Default: %(default)s')

    parser.add_argument('--merge',
                    metavar='<string>',
                    action=StoreValue,
                    default='all',
                    choices=['all', 'pointcloud', 'orthophoto', 'dem'],
                    help=('Choose what to merge in the merge step in a split dataset. '
                          'By default all available outputs are merged. '
                          'Options: %(choices)s. Default: '
                            '%(default)s'))

    parser.add_argument('--force-gps',
                    action=StoreTrue,
                    nargs=0,
                    default=False,
                    help=('Use images\' GPS exif data for reconstruction, even if there are GCPs present.'
                          'This flag is useful if you have high precision GPS measurements. '
                          'If there are no GCPs, this flag does nothing. Default: %(default)s'))
    
    parser.add_argument('--gps-accuracy',
                        type=float,
                        action=StoreValue,
                        metavar='<positive float>',
                        default=3,
                        help='Set a value in meters for the GPS Dilution of Precision (DOP) '
                        'information for all images. If your images are tagged '
                        'with high precision GPS information (RTK), this value will be automatically '
                        'set accordingly. You can use this option to manually set it in case the reconstruction '
                        'fails. Lowering this option can sometimes help control bowling-effects over large areas. Default: %(default)s')

    parser.add_argument('--optimize-disk-space',
                action=StoreTrue,
                nargs=0,
                default=False,
                help=('Delete heavy intermediate files to optimize disk space usage. This '
                      'affects the ability to restart the pipeline from an intermediate stage, '
                      'but allows datasets to be processed on machines that don\'t have sufficient '
                      'disk space available. Default: %(default)s'))

    parser.add_argument('--pc-rectify',
                    action=StoreTrue,
                    nargs=0,
                    default=False,
                    help=('Perform ground rectification on the point cloud. This means that wrongly classified ground '
                          'points will be re-classified and gaps will be filled. Useful for generating DTMs. '
                          'Default: %(default)s'))

    parser.add_argument('--primary-band',
                        metavar='<string>',
                        action=StoreValue,
                        default="auto",
                        type=str,
                        help=('When processing multispectral datasets, you can specify the name of the primary band that will be used for reconstruction. '
                              'It\'s recommended to choose a band which has sharp details and is in focus. ' 
                              'Default: %(default)s'))

    parser.add_argument('--skip-band-alignment',
                    action=StoreTrue,
                    nargs=0,
                    default=False,
                    help=('When processing multispectral datasets, ODM will automatically align the images for each band. '
                          'If the images have been postprocessed and are already aligned, use this option. '
                          'Default: %(default)s'))

    args, unknown = parser.parse_known_args(argv)
    DEPRECATED = ["--verbose", "--debug", "--time", "--resize-to", "--depthmap-resolution", "--pc-geometric", "--texturing-data-term", "--texturing-outlier-removal-type", "--texturing-tone-mapping", "--texturing-skip-local-seam-leveling"]
    unknown_e = [p for p in unknown if p not in DEPRECATED]
    if len(unknown_e) > 0:
        raise parser.error("unrecognized arguments: %s" % " ".join(unknown_e))

    for p in unknown:
        if p in DEPRECATED:
            log.ODM_WARNING("%s is no longer a valid argument and will be ignored!" % p)

    # check that the project path setting has been set properly
    if not args.project_path:
        log.ODM_ERROR('You need to set the project path in the '
                      'settings.yaml file before you can run ODM, '
                      'or use `--project-path <path>`. Run `python3 '
                      'run.py --help` for more information. ')
        sys.exit(1)

    if args.fast_orthophoto:
      log.ODM_INFO('Fast orthophoto is turned on, automatically setting --skip-3dmodel')
      args.skip_3dmodel = True

    if args.pc_rectify and not args.pc_classify:
      log.ODM_INFO("Ground rectify is turned on, automatically turning on point cloud classification")
      args.pc_classify = True

    if args.dtm and not args.pc_classify:
      log.ODM_INFO("DTM is turned on, automatically turning on point cloud classification")
      args.pc_classify = True

    if args.skip_3dmodel and args.use_3dmesh:
      log.ODM_WARNING('--skip-3dmodel is set, but so is --use-3dmesh. --skip-3dmodel will be ignored.')
      args.skip_3dmodel = False

    if args.orthophoto_cutline and not args.crop:
      log.ODM_WARNING("--orthophoto-cutline is set, but --crop is not. --crop will be set to 0.01")
      args.crop = 0.01

    if args.sm_cluster:
        try:
            Node.from_url(args.sm_cluster).info()
        except exceptions.NodeConnectionError as e:
            log.ODM_ERROR("Cluster node seems to be offline: %s"  % str(e))
            sys.exit(1)
    
    return args
