import argparse
from opendm import context
from opendm import io
import yaml

# parse arguments
processopts = ['resize', 'opensfm', 'slam', 'cmvs', 'pmvs',
               'odm_meshing', 'mvs_texturing', 'odm_georeferencing',
               'odm_orthophoto']

# Load global settings file
with open(context.settings_path) as stream:
    datamap = yaml.safe_load(stream)
    defaultSettings = datamap['settings']

with open(io.join_paths(context.root_path, 'VERSION')) as version_file:
    __version__ = version_file.read().strip()

def alphanumeric_string(string):
    import re
    if re.match('^[a-zA-Z0-9_-]+$', string) is None:
        msg = '{0} is not a valid name. Must use alphanumeric characters.'.format(string)
        raise argparse.ArgumentTypeError(msg)
    return string


class RerunFrom(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        setattr(namespace, self.dest, processopts[processopts.index(values):])


parser = argparse.ArgumentParser(description='OpenDroneMap')


def config():
    parser.add_argument('--images', '-i',
                        metavar='<path>',
                        help='Path to input images'),

    parser.add_argument('--project-path',
                        metavar='<path>',
                        help='Path to the project folder',
                        default=defaultSettings['project_path'])

    parser.add_argument('name',
                        metavar='<project name>',
                        type=alphanumeric_string,
                        help='Name of Project (i.e subdirectory of projects folder)')

    parser.add_argument('--resize-to',  # currently doesn't support 'orig'
                        metavar='<integer>',
                        default=defaultSettings['resize_to'],
                        type=int,
                        help='resizes images by the largest side')

    parser.add_argument('--start-with', '-s',
                        metavar='<string>',
                        default=defaultSettings['start_with'],
                        choices=processopts,
                        help=('Can be one of: ' + ' | '.join(processopts)))

    parser.add_argument('--end-with', '-e',
                        metavar='<string>',
                        default=defaultSettings['end_with'],
                        choices=processopts,
                        help=('Can be one of:' + ' | '.join(processopts)))

    rerun = parser.add_mutually_exclusive_group()

    rerun.add_argument('--rerun', '-r',
                       metavar='<string>',
                       choices=processopts,
                       help=('Can be one of:' + ' | '.join(processopts)))

    rerun.add_argument('--rerun-all',
                       action='store_true',
                       default=defaultSettings['rerun_all'],
                       help='force rerun of all tasks')

    rerun.add_argument('--rerun-from',
                       action=RerunFrom,
                       metavar='<string>',
                       choices=processopts,
                       help=('Can be one of:' + ' | '.join(processopts)))

    parser.add_argument('--video',
                        metavar='<string>',
                        help='Path to the video file to process')

    parser.add_argument('--slam-config',
                        metavar='<string>',
                        help='Path to config file for orb-slam')

    parser.add_argument('--force-focal',
                        metavar='<positive float>',
                        type=float,
                        help=('Override the focal length information for the '
                              'images'))

    parser.add_argument('--force-ccd',
                        metavar='<positive float>',
                        type=float,
                        help='Override the ccd width information for the images')

    parser.add_argument('--min-num-features',
                        metavar='<integer>',
                        default=defaultSettings['opensfm']['min_num_features'],
                        type=int,
                        help=('Minimum number of features to extract per image. '
                              'More features leads to better results but slower '
                              'execution. Default: %(default)s'))

    parser.add_argument('--matcher-threshold',
                        metavar='<percent>',
                        default=defaultSettings['opensfm']['matcher_threshold'],
                        type=float,
                        help=('Ignore matched keypoints if the two images share '
                              'less than <float> percent of keypoints. Default:'
                              ' %(default)s'))

    parser.add_argument('--matcher-ratio',
                        metavar='<float>',
                        default=defaultSettings['opensfm']['matcher_ratio'],
                        type=float,
                        help=('Ratio of the distance to the next best matched '
                              'keypoint. Default: %(default)s'))

    parser.add_argument('--matcher-neighbors',
                        type=int,
                        metavar='<integer>',
                        default=defaultSettings['opensfm']['matcher_neighbors'],
                        help='Number of nearest images to pre-match based on GPS '
                             'exif data. Set to 0 to skip pre-matching. '
                             'Neighbors works together with Distance parameter, '
                             'set both to 0 to not use pre-matching. OpenSFM '
                             'uses both parameters at the same time, Bundler '
                             'uses only one which has value, prefering the '
                             'Neighbors parameter. Default: %(default)s')

    parser.add_argument('--matcher-distance',
                        metavar='<integer>',
                        default=defaultSettings['opensfm']['matcher_distance'],
                        type=int,
                        help='Distance threshold in meters to find pre-matching '
                             'images based on GPS exif data. Set to 0 to skip '
                             'pre-matching. Default: %(default)s')

    parser.add_argument('--opensfm-processes',
                        metavar='<positive integer>',
                        default=defaultSettings['opensfm']['processes'],
                        type=int,
                        help=('The maximum number of processes to use in dense '
                              'reconstruction. Default: %(default)s'))

    parser.add_argument('--use-pmvs',
                        action='store_true',
                        default=defaultSettings['pmvs']['enabled'],
                        help='Use pmvs to compute point cloud alternatively')

    parser.add_argument('--cmvs-maxImages',
                        metavar='<integer>',
                        default=defaultSettings['pmvs']['cmvs_max_images'],
                        type=int,
                        help='The maximum number of images per cluster. '
                             'Default: %(default)s')

    parser.add_argument('--pmvs-level',
                        metavar='<positive integer>',
                        default=defaultSettings['pmvs']['level'],
                        type=int,
                        help=('The level in the image pyramid that is used '
                              'for the computation. see '
                              'http://www.di.ens.fr/pmvs/documentation.html for '
                              'more pmvs documentation. Default: %(default)s'))

    parser.add_argument('--pmvs-csize',
                        metavar='<positive integer>',
                        default=defaultSettings['pmvs']['cell_size'],
                        type=int,
                        help='Cell size controls the density of reconstructions'
                             'Default: %(default)s')

    parser.add_argument('--pmvs-threshold',
                        metavar='<float: -1.0 <= x <= 1.0>',
                        default=defaultSettings['pmvs']['threshold'],
                        type=float,
                        help=('A patch reconstruction is accepted as a success '
                              'and kept if its associated photometric consistency '
                              'measure is above this threshold. Default: %(default)s'))

    parser.add_argument('--pmvs-wsize',
                        metavar='<positive integer>',
                        default=defaultSettings['pmvs']['wsize'],
                        type=int,
                        help='pmvs samples wsize x wsize pixel colors from '
                             'each image to compute photometric consistency '
                             'score. For example, when wsize=7, 7x7=49 pixel '
                             'colors are sampled in each image. Increasing the '
                             'value leads to more stable reconstructions, but '
                             'the program becomes slower. Default: %(default)s')

    parser.add_argument('--pmvs-min-images',
                        metavar='<positive integer>',
                        default=defaultSettings['pmvs']['min_images'],
                        type=int,
                        help=('Each 3D point must be visible in at least '
                              'minImageNum images for being reconstructed. 3 is '
                              'suggested in general. Default: %(default)s'))

    parser.add_argument('--pmvs-num-cores',
                        metavar='<positive integer>',
                        default=defaultSettings['pmvs']['num_cores'],
                        type=int,
                        help=('The maximum number of cores to use in dense '
                              'reconstruction. Default: %(default)s'))

    parser.add_argument('--mesh-size',
                        metavar='<positive integer>',
                        default=defaultSettings['mesh']['size'],
                        type=int,
                        help=('The maximum vertex count of the output mesh '
                              'Default: %(default)s'))

    parser.add_argument('--mesh-octree-depth',
                        metavar='<positive integer>',
                        default=defaultSettings['mesh']['octree_depth'],
                        type=int,
                        help=('Oct-tree depth used in the mesh reconstruction, '
                              'increase to get more vertices, recommended '
                              'values are 8-12. Default: %(default)s'))

    parser.add_argument('--mesh-samples',
                        metavar='<float >= 1.0>',
                        default=defaultSettings['mesh']['samples'],
                        type=float,
                        help=('Number of points per octree node, recommended '
                              'and default value: %(default)s'))

    parser.add_argument('--mesh-solver-divide',
                        metavar='<positive integer>',
                        default=defaultSettings['mesh']['solver_divide'],
                        type=int,
                        help=('Oct-tree depth at which the Laplacian equation '
                              'is solved in the surface reconstruction step. '
                              'Increasing this value increases computation '
                              'times slightly but helps reduce memory usage. '
                              'Default: %(default)s'))

    parser.add_argument('--texturing-data-term',
                        metavar='<string>',
                        default=defaultSettings['texturing']['data_term'],
                        help=('Data term: [area, gmi]. Default: '
                              '%(default)s'))

    parser.add_argument('--texturing-outlier-removal-type',
                        metavar='<string>',
                        default=defaultSettings['texturing']['outlier_removal_type'],
                        help=('Type of photometric outlier removal method: ' 
                              '[none, gauss_damping, gauss_clamping]. Default: '  
                              '%(default)s'))

    parser.add_argument('--texturing-skip-visibility-test',
                        action='store_true',
                        default=defaultSettings['texturing']['skip_visibility_test'],
                        help=('Skip geometric visibility test. Default: '
                              ' %(default)s'))

    parser.add_argument('--texturing-skip-global-seam-leveling',
                        action='store_true',
                        default=defaultSettings['texturing']['skip_global_seam_leveling'],
                        help=('Skip global seam leveling. Useful for IR data.'
                              'Default: %(default)s'))

    parser.add_argument('--texturing-skip-local-seam-leveling',
                        action='store_true',
                        default=defaultSettings['texturing']['skip_local_seam_leveling'],
                        help='Skip local seam blending. Default:  %(default)s')

    parser.add_argument('--texturing-skip-hole-filling',
                        action='store_true',
                        default=defaultSettings['texturing']['skip_hole_filling'],
                        help=('Skip filling of holes in the mesh. Default: '
                              ' %(default)s'))

    parser.add_argument('--texturing-keep-unseen-faces',
                        action='store_true',
                        default=defaultSettings['texturing']['keep_unseen_faces'],
                        help=('Keep faces in the mesh that are not seen in any camera. ' 
                              'Default:  %(default)s'))

    parser.add_argument('--texturing-tone-mapping',
                        metavar='<string>',
                        choices=['none', 'gamma'],
                        default=defaultSettings['texturing']['tone_mapping'],
                        help='Turn on gamma tone mapping or none for no tone '
                             'mapping. Choices are  \'gamma\' or \'none\'. '
                             'Default: %(default)s ')

    parser.add_argument('--gcp',
                        metavar='<path string>',
                        default=defaultSettings['georeferencing']['gcp'],
                        help=('path to the file containing the ground control '
                              'points used for georeferencing.  Default: '
                              '%(default)s. The file needs to '
                              'be on the following line format: \neasting '
                              'northing height pixelrow pixelcol imagename'))

    parser.add_argument('--use-exif',
                        action='store_true',
                        default=defaultSettings['georeferencing']['use_exif'],
                        help=('Use this tag if you have a gcp_list.txt but '
                              'want to use the exif geotags instead'))

    parser.add_argument('--orthophoto-resolution',
                        metavar='<float > 0.0>',
                        default=defaultSettings['orthophoto']['resolution'],
                        type=float,
                        help=('Orthophoto ground resolution in pixels/meter'
                              'Default: %(default)s'))

    parser.add_argument('--orthophoto-target-srs',
                        metavar="<EPSG:XXXX>",
                        type=str,
                        default=None,
                        help='Target spatial reference for orthophoto creation. '
                             'Not implemented yet.\n'
                             'Default: %(default)s')

    parser.add_argument('--orthophoto-no-tiled',
                        action='store_true',
                        default=False,
                        help='Set this parameter if you want a stripped geoTIFF.\n'
                             'Default: %(default)s')

    parser.add_argument('--orthophoto-compression',
                        metavar='<STRING>',
                        type=str,
                        choices=['JPEG','LZW','PACKBITS','DEFLATE','LZMA','NONE'],
                        default='DEFLATE',
                        help='Set the compression to use. Note that this could '
                             'break gdal_translate if you don\'t know what you '
                             'are doing. Options: %(choices)s.\nDefault: %(default)s')

    parser.add_argument('--zip-results',
                        action='store_true',
                        default=defaultSettings['zip_results'],
                        help='compress the results using gunzip')

    parser.add_argument('--verbose', '-v',
                        action='store_true',
                        default=defaultSettings['verbose'],
                        help='Print additional messages to the console\n'
                             'Default: %(default)s')

    parser.add_argument('--time',
                        action='store_true',
                        default=defaultSettings['time'],
                        help='Generates a benchmark file with runtime info\n'
                             'Default: %(default)s')

    parser.add_argument('--version',
                        action='version',
                        version='OpenDroneMap {0}'.format(__version__),
                        help='Displays version number and exits. ')

    return parser.parse_args()
