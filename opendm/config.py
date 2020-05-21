import argparse
import json
from opendm import context
from opendm import io
from opendm import log
from appsettings import SettingsParser
from pyodm import Node, exceptions

import sys

# parse arguments
processopts = ['dataset', 'split', 'merge', 'opensfm', 'mve', 'odm_filterpoints',
               'odm_meshing', 'mvs_texturing', 'odm_georeferencing',
               'odm_dem', 'odm_orthophoto', 'odm_report']

with open(io.join_paths(context.root_path, 'VERSION')) as version_file:
    __version__ = version_file.read().strip()


def alphanumeric_string(string):
    import re
    if re.match('^[a-zA-Z0-9_-]+$', string) is None:
        msg = '{0} is not a valid name. Must use alphanumeric characters.'.format(string)
        raise argparse.ArgumentTypeError(msg)
    return string

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

parser = SettingsParser(description='OpenDroneMap',
                        usage='%(prog)s [options] <project name>',
                        yaml_file=open(context.settings_path))
args = None

def config(argv=None):
    global args

    if args is not None and argv is None:
        return args
    
    parser.add_argument('--project-path',
                        metavar='<path>',
                        action=StoreValue,
                        help='Path to the project folder')

    parser.add_argument('name',
                        metavar='<project name>',
                        action=StoreValue,
                        type=alphanumeric_string,
                        default='code',
                        nargs='?',
                        help='Name of Project (i.e subdirectory of projects folder)')

    parser.add_argument('--resize-to',
                        metavar='<integer>',
                        action=StoreValue,
                        default=2048,
                        type=int,
                        help='Resizes images by the largest side for feature extraction purposes only. '
                             'Set to -1 to disable. This does not affect the final orthophoto '
                             ' resolution quality and will not resize the original images. Default:  %(default)s')

    parser.add_argument('--end-with', '-e',
                        metavar='<string>',
                        action=StoreValue,
                        default='odm_report',
                        choices=processopts,
                        help=('Can be one of:' + ' | '.join(processopts)))

    rerun = parser.add_mutually_exclusive_group()

    rerun.add_argument('--rerun', '-r',
                       metavar='<string>',
                       action=StoreValue,
                       choices=processopts,
                       help=('Can be one of:' + ' | '.join(processopts)))

    rerun.add_argument('--rerun-all',
                       action=StoreTrue,
                       nargs=0,
                       default=False,
                       help='force rerun of all tasks')

    rerun.add_argument('--rerun-from',
                       action=RerunFrom,
                       metavar='<string>',
                       choices=processopts,
                       help=('Can be one of:' + ' | '.join(processopts)))

    # parser.add_argument('--video',
    #                     metavar='<string>',
    #                     help='Path to the video file to process')

    # parser.add_argument('--slam-config',
    #                     metavar='<string>',
    #                     help='Path to config file for orb-slam')

    parser.add_argument('--min-num-features',
                        metavar='<integer>',
                        action=StoreValue,
                        default=8000,
                        type=int,
                        help=('Minimum number of features to extract per image. '
                              'More features leads to better results but slower '
                              'execution. Default: %(default)s'))
    
    parser.add_argument('--feature-type',
                        metavar='<string>',
                        action=StoreValue,
                        default='sift',
                        choices=['sift', 'hahog'],
                        help=('Choose the algorithm for extracting keypoints and computing descriptors. '
                            'Can be one of: [sift, hahog]. Default: '
                            '%(default)s'))

    parser.add_argument('--matcher-neighbors',
                        metavar='<integer>',
                        action=StoreValue,
                        default=8,
                        type=int,
                        help='Number of nearest images to pre-match based on GPS '
                             'exif data. Set to 0 to skip pre-matching. '
                             'Neighbors works together with Distance parameter, '
                             'set both to 0 to not use pre-matching. OpenSFM '
                             'uses both parameters at the same time, Bundler '
                             'uses only one which has value, prefering the '
                             'Neighbors parameter. Default: %(default)s')

    parser.add_argument('--matcher-distance',
                        metavar='<integer>',
                        action=StoreValue,
                        default=0,
                        type=int,
                        help='Distance threshold in meters to find pre-matching '
                             'images based on GPS exif data. Set both '
                             'matcher-neighbors and this to 0 to skip '
                             'pre-matching. Default: %(default)s')

    parser.add_argument('--use-fixed-camera-params',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Turn off camera parameter optimization during bundler')

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
            choices=['auto', 'perspective', 'brown', 'fisheye', 'spherical'],
            help=('Set a camera projection type. Manually setting a value '
                'can help improve geometric undistortion. By default the application '
                'tries to determine a lens type from the images metadata. Can be '
                'set to one of: [auto, perspective, brown, fisheye, spherical]. Default: '
                '%(default)s'))

    parser.add_argument('--radiometric-calibration',
            metavar='<string>',
            action=StoreValue,
            default='none',
            choices=['none', 'camera', 'camera+sun'],
            help=('Set the radiometric calibration to perform on images. '
                'When processing multispectral images you should set this option '
                'to obtain reflectance values (otherwise you will get digital number values). '
                '[camera] applies black level, vignetting, row gradient gain/exposure compensation (if appropriate EXIF tags are found). '
                '[camera+sun] is experimental, applies all the corrections of [camera], plus compensates for spectral radiance registered via a downwelling light sensor (DLS) taking in consideration the angle of the sun. '
                'Can be set to one of: [none, camera, camera+sun]. Default: '
                '%(default)s'))

    parser.add_argument('--max-concurrency',
                        metavar='<positive integer>',
                        action=StoreValue,
                        default=context.num_cores,
                        type=int,
                        help=('The maximum number of processes to use in various '
                              'processes. Peak memory requirement is ~1GB per '
                              'thread and 2 megapixel image resolution. Default: %(default)s'))

    parser.add_argument('--depthmap-resolution',
                        metavar='<positive float>',
                        action=StoreValue,
                        type=float,
                        default=640,
                        help=('Controls the density of the point cloud by setting the resolution of the depthmap images. Higher values take longer to compute '
                              'but produce denser point clouds. '
                              'Default: %(default)s'))

    parser.add_argument('--opensfm-depthmap-min-consistent-views',
                      metavar='<integer: 2 <= x <= 9>',
                      action=StoreValue,
                      type=int,
                      default=3,
                      help=('Minimum number of views that should reconstruct a point for it to be valid. Use lower values '
                            'if your images have less overlap. Lower values result in denser point clouds '
                            'but with more noise. '
                            'Default: %(default)s'))

    parser.add_argument('--opensfm-depthmap-method',
                      metavar='<string>',
                      action=StoreValue,
                      default='PATCH_MATCH',
                      choices=['PATCH_MATCH', 'BRUTE_FORCE', 'PATCH_MATCH_SAMPLE'],
                      help=('Raw depthmap computation algorithm. '
                            'PATCH_MATCH and PATCH_MATCH_SAMPLE are faster, but might miss some valid points. '
                            'BRUTE_FORCE takes longer but produces denser reconstructions. '
                            'Default: %(default)s'))

    parser.add_argument('--opensfm-depthmap-min-patch-sd',
                      metavar='<positive float>',
                      action=StoreValue,
                      type=float,
                      default=1,
                      help=('When using PATCH_MATCH or PATCH_MATCH_SAMPLE, controls the standard deviation threshold to include patches. '
                            'Patches with lower standard deviation are ignored. '
                            'Default: %(default)s'))

    parser.add_argument('--use-hybrid-bundle-adjustment',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Run local bundle adjustment for every image added to the reconstruction and a global '
                             'adjustment every 100 images. Speeds up reconstruction for very large datasets.')

    parser.add_argument('--mve-confidence',
                        metavar='<float: 0 <= x <= 1>',
                        action=StoreValue,
                        type=float,
                        default=0.60,
                        help=('Discard points that have less than a certain confidence threshold. '
                              'This only affects dense reconstructions performed with MVE. '
                              'Higher values discard more points. '
                              'Default: %(default)s'))

    parser.add_argument('--use-3dmesh',
                    action=StoreTrue,
                    nargs=0,
                    default=False,
                    help='Use a full 3D mesh to compute the orthophoto instead of a 2.5D mesh. This option is a bit faster and provides similar results in planar areas.')

    parser.add_argument('--skip-3dmodel',
                    action=StoreTrue,
                    nargs=0,
                    default=False,
                    help='Skip generation of a full 3D model. This can save time if you only need 2D results such as orthophotos and DEMs.')

    parser.add_argument('--use-opensfm-dense',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Use opensfm to compute dense point cloud alternatively')

    parser.add_argument('--ignore-gsd',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Ignore Ground Sampling Distance (GSD). GSD '
                        'caps the maximum resolution of image outputs and '
                        'resizes images when necessary, resulting in faster processing and '
                        'lower memory usage. Since GSD is an estimate, sometimes ignoring it can result in slightly better image output quality.')

    parser.add_argument('--mesh-size',
                        metavar='<positive integer>',
                        action=StoreValue,
                        default=200000,
                        type=int,
                        help=('The maximum vertex count of the output mesh. '
                              'Default: %(default)s'))

    parser.add_argument('--mesh-octree-depth',
                        metavar='<positive integer>',
                        action=StoreValue,
                        default=10,
                        type=int,
                        help=('Oct-tree depth used in the mesh reconstruction, '
                              'increase to get more vertices, recommended '
                              'values are 8-12. Default: %(default)s'))

    parser.add_argument('--mesh-samples',
                        metavar='<float >= 1.0>',
                        action=StoreValue,
                        default=1.0,
                        type=float,
                        help=('Number of points per octree node, recommended '
                              'and default value: %(default)s'))

    parser.add_argument('--mesh-point-weight',
                        metavar='<positive float>',
                        action=StoreValue,
                        default=4,
                        type=float,
                        help=('This floating point value specifies the importance'
                        ' that interpolation of the point samples is given in the '
                        'formulation of the screened Poisson equation. The results '
                        'of the original (unscreened) Poisson Reconstruction can '
                        'be obtained by setting this value to 0.'
                        'Default= %(default)s'))

    parser.add_argument('--fast-orthophoto',
                action=StoreTrue,
                nargs=0,
                default=False,
                help='Skips dense reconstruction and 3D model generation. '
                'It generates an orthophoto directly from the sparse reconstruction. '
                'If you just need an orthophoto and do not need a full 3D model, turn on this option.')

    parser.add_argument('--crop',
                    metavar='<positive float>',
                    action=StoreValue,
                    default=3,
                    type=float,
                    help=('Automatically crop image outputs by creating a smooth buffer '
                          'around the dataset boundaries, shrinked by N meters. '
                          'Use 0 to disable cropping. '
                          'Default: %(default)s'))

    parser.add_argument('--pc-classify',
            action=StoreTrue,
            nargs=0,
            default=False,
            help='Classify the point cloud outputs using a Simple Morphological Filter. '
            'You can control the behavior of this option by tweaking the --dem-* parameters. '
            'Default: '
            '%(default)s')

    parser.add_argument('--pc-csv',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Export the georeferenced point cloud in CSV format. Default:  %(default)s')
    
    parser.add_argument('--pc-las',
                action=StoreTrue,
                nargs=0,
                default=False,
                help='Export the georeferenced point cloud in LAS format. Default:  %(default)s')

    parser.add_argument('--pc-ept',
                action=StoreTrue,
                nargs=0,
                default=False,
                help='Export the georeferenced point cloud in Entwine Point Tile (EPT) format. Default:  %(default)s')

    parser.add_argument('--pc-filter',
                        metavar='<positive float>',
                        action=StoreValue,
                        type=float,
                        default=2.5,
                        help='Filters the point cloud by removing points that deviate more than N standard deviations from the local mean. Set to 0 to disable filtering.'
                             '\nDefault: '
                             '%(default)s')
    
    parser.add_argument('--pc-sample',
                        metavar='<positive float>',
                        action=StoreValue,
                        type=float,
                        default=0,
                        help='Filters the point cloud by keeping only a single point around a radius N (in meters). This can be useful to limit the output resolution of the point cloud. Set to 0 to disable sampling.'
                             '\nDefault: '
                             '%(default)s')

    parser.add_argument('--smrf-scalar',
                        metavar='<positive float>',
                        action=StoreValue,
                        type=float,
                        default=1.25,
                        help='Simple Morphological Filter elevation scalar parameter. '
                             '\nDefault: '
                             '%(default)s')

    parser.add_argument('--smrf-slope',
        metavar='<positive float>',
        action=StoreValue,
        type=float,
        default=0.15,
        help='Simple Morphological Filter slope parameter (rise over run). '
                '\nDefault: '
                '%(default)s')
    
    parser.add_argument('--smrf-threshold',
        metavar='<positive float>',
        action=StoreValue,
        type=float,
        default=0.5,
        help='Simple Morphological Filter elevation threshold parameter (meters). '
                '\nDefault: '
                '%(default)s')
    
    parser.add_argument('--smrf-window',
        metavar='<positive float>',
        action=StoreValue,
        type=float,
        default=18.0,
        help='Simple Morphological Filter window radius parameter (meters). '
                '\nDefault: '
                '%(default)s')

    parser.add_argument('--texturing-data-term',
                        metavar='<string>',
                        action=StoreValue,
                        default='gmi',
                        choices=['gmi', 'area'],
                        help=('Data term: [area, gmi]. Default: '
                              '%(default)s'))

    parser.add_argument('--texturing-nadir-weight',
                        metavar='<integer: 0 <= x <= 32>',
                        action=StoreValue,
                        default=16,
                        type=int,
                        help=('Affects orthophotos only. '
                              'Higher values result in sharper corners, but can affect color distribution and blurriness. '
                              'Use lower values for planar areas and higher values for urban areas. '
                              'The default value works well for most scenarios. Default: '
                              '%(default)s'))

    parser.add_argument('--texturing-outlier-removal-type',
                        metavar='<string>',
                        action=StoreValue,
                        default='gauss_clamping',
                        choices=['none', 'gauss_clamping', 'gauss_damping'],
                        help=('Type of photometric outlier removal method: '
                              '[none, gauss_damping, gauss_clamping]. Default: '
                              '%(default)s'))

    parser.add_argument('--texturing-skip-visibility-test',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help=('Skip geometric visibility test. Default: '
                              ' %(default)s'))

    parser.add_argument('--texturing-skip-global-seam-leveling',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help=('Skip global seam leveling. Useful for IR data.'
                              'Default: %(default)s'))

    parser.add_argument('--texturing-skip-local-seam-leveling',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Skip local seam blending. Default:  %(default)s')

    parser.add_argument('--texturing-skip-hole-filling',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help=('Skip filling of holes in the mesh. Default: '
                              ' %(default)s'))

    parser.add_argument('--texturing-keep-unseen-faces',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help=('Keep faces in the mesh that are not seen in any camera. '
                              'Default:  %(default)s'))

    parser.add_argument('--texturing-tone-mapping',
                        metavar='<string>',
                        action=StoreValue,
                        choices=['none', 'gamma'],
                        default='none',
                        help='Turn on gamma tone mapping or none for no tone '
                             'mapping. Choices are  \'gamma\' or \'none\'. '
                             'Default: %(default)s ')

    parser.add_argument('--gcp',
                        metavar='<path string>',
                        action=StoreValue,
                        default=None,
                        help=('path to the file containing the ground control '
                              'points used for georeferencing.  Default: '
                              '%(default)s. The file needs to '
                              'be on the following line format: \neasting '
                              'northing height pixelrow pixelcol imagename'))

    parser.add_argument('--use-exif',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help=('Use this tag if you have a gcp_list.txt but '
                              'want to use the exif geotags instead'))

    parser.add_argument('--dtm',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Use this tag to build a DTM (Digital Terrain Model, ground only) using a simple '
                             'morphological filter. Check the --dem* and --smrf* parameters for finer tuning.')

    parser.add_argument('--dsm',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Use this tag to build a DSM (Digital Surface Model, ground + objects) using a progressive '
                             'morphological filter. Check the --dem* parameters for finer tuning.')

    parser.add_argument('--dem-gapfill-steps',
                        metavar='<positive integer>',
                        action=StoreValue,
                        default=3,
                        type=int,
                        help='Number of steps used to fill areas with gaps. Set to 0 to disable gap filling. '
                             'Starting with a radius equal to the output resolution, N different DEMs are generated with '
                             'progressively bigger radius using the inverse distance weighted (IDW) algorithm '
                             'and merged together. Remaining gaps are then merged using nearest neighbor interpolation. '
                             '\nDefault=%(default)s')

    parser.add_argument('--dem-resolution',
                        metavar='<float>',
                        action=StoreValue,
                        type=float,
                        default=5,
                        help='DSM/DTM resolution in cm / pixel. Note that this value is capped by a ground sampling distance (GSD) estimate. To remove the cap, check --ignore-gsd also.'
                             '\nDefault: %(default)s')

    parser.add_argument('--dem-decimation',
                        metavar='<positive integer>',
                        action=StoreValue,
                        default=1,
                        type=int,
                        help='Decimate the points before generating the DEM. 1 is no decimation (full quality). '
                             '100 decimates ~99%% of the points. Useful for speeding up '
                             'generation.\nDefault=%(default)s')
    
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
                        help=('Orthophoto resolution in cm / pixel. Note that this value is capped by a ground sampling distance (GSD) estimate. To remove the cap, check --ignore-gsd also.\n'
                              'Default: %(default)s'))

    parser.add_argument('--orthophoto-no-tiled',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Set this parameter if you want a stripped geoTIFF.\n'
                             'Default: %(default)s')
    
    parser.add_argument('--orthophoto-png',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Set this parameter if you want to generate a PNG rendering of the orthophoto.\n'
                             'Default: %(default)s')

    parser.add_argument('--orthophoto-compression',
                        metavar='<string>',
                        action=StoreValue,
                        type=str,
                        choices=['JPEG', 'LZW', 'PACKBITS', 'DEFLATE', 'LZMA', 'NONE'],
                        default='DEFLATE',
                        help='Set the compression to use. Note that this could '
                             'break gdal_translate if you don\'t know what you '
                             'are doing. Options: %(choices)s.\nDefault: %(default)s')
    
    parser.add_argument('--orthophoto-cutline',
            action=StoreTrue,
            nargs=0,
            default=False,
            help='Generates a polygon around the cropping area '
            'that cuts the orthophoto around the edges of features. This polygon '
            'can be useful for stitching seamless mosaics with multiple overlapping orthophotos. '
            'Default: '
            '%(default)s')

    parser.add_argument('--build-overviews',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Build orthophoto overviews using gdaladdo.')

    parser.add_argument('--verbose', '-v',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Print additional messages to the console\n'
                             'Default: %(default)s')

    parser.add_argument('--time',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Generates a benchmark file with runtime info\n'
                             'Default: %(default)s')
    
    parser.add_argument('--debug',
                        action=StoreTrue,
                        nargs=0,
                        default=False,
                        help='Print debug messages\n'
                             'Default: %(default)s')

    parser.add_argument('--version',
                        action='version',
                        version='OpenDroneMap {0}'.format(__version__),
                        help='Displays version number and exits. ')

    parser.add_argument('--split',
                        type=int,
                        action=StoreValue,
                        default=999999,
                        metavar='<positive integer>',
                        help='Average number of images per submodel. When '
                                'splitting a large dataset into smaller '
                                'submodels, images are grouped into clusters. '
                                'This value regulates the number of images that '
                                'each cluster should have on average.')

    parser.add_argument('--split-overlap',
                        type=float,
                        action=StoreValue,
                        metavar='<positive integer>',
                        default=150,
                        help='Radius of the overlap between submodels. '
                        'After grouping images into clusters, images '
                        'that are closer than this radius to a cluster '
                        'are added to the cluster. This is done to ensure '
                        'that neighboring submodels overlap.')

    parser.add_argument('--split-multitracks',
                       action=StoreTrue,
                       nargs=0,
                       default=False,
                       help='Split multi-track reconstructions.')

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
                        default=15,
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

    args = parser.parse_args(argv)

    # check that the project path setting has been set properly
    if not args.project_path:
        log.ODM_ERROR('You need to set the project path in the '
                      'settings.yaml file before you can run ODM, '
                      'or use `--project-path <path>`. Run `python '
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
    
    # if args.radiometric_calibration != "none" and not args.texturing_skip_global_seam_leveling:
    #     log.ODM_WARNING("radiometric-calibration is turned on, automatically setting --texturing-skip-global-seam-leveling")
    #     args.texturing_skip_global_seam_leveling = True

    return args
