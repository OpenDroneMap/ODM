import argparse
from opendm import context
from opendm import io
from opendm import log
from appsettings import SettingsParser

import sys

# parse arguments
processopts = ['dataset', 'opensfm', 'slam', 'cmvs', 'pmvs',
               'odm_meshing', 'odm_25dmeshing', 'mvs_texturing', 'odm_georeferencing',
               'odm_dem', 'odm_orthophoto']

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


parser = SettingsParser(description='OpenDroneMap',
                        usage='%(prog)s [options] <project name>',
                        yaml_file=open(context.settings_path))

def config():
    parser.add_argument('--images', '-i',
                        metavar='<path>',
                        help='Path to input images'),

    parser.add_argument('--project-path',
                        metavar='<path>',
                        help='Path to the project folder')

    parser.add_argument('name',
                        metavar='<project name>',
                        type=alphanumeric_string,
                        help='Name of Project (i.e subdirectory of projects folder)')

    parser.add_argument('--resize-to',
                        metavar='<integer>',
                        default=2048,
                        type=int,
                        help='resizes images by the largest side for opensfm. '
                             'Set to -1 to disable. Default:  %(default)s')

    parser.add_argument('--start-with', '-s',
                        metavar='<string>',
                        default='resize',
                        choices=processopts,
                        help=('Can be one of: ' + ' | '.join(processopts)))

    parser.add_argument('--end-with', '-e',
                        metavar='<string>',
                        default='odm_orthophoto',
                        choices=processopts,
                        help=('Can be one of:' + ' | '.join(processopts)))

    rerun = parser.add_mutually_exclusive_group()

    rerun.add_argument('--rerun', '-r',
                       metavar='<string>',
                       choices=processopts,
                       help=('Can be one of:' + ' | '.join(processopts)))

    rerun.add_argument('--rerun-all',
                       action='store_true',
                       default=False,
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

    parser.add_argument('--proj',
                        metavar='<PROJ4 string>',
                        help='Projection used to transform the model into geographic coordinates')

    parser.add_argument('--force-ccd',
                        metavar='<positive float>',
                        type=float,
                        help='Override the ccd width information for the images')

    parser.add_argument('--min-num-features',
                        metavar='<integer>',
                        default=8000,
                        type=int,
                        help=('Minimum number of features to extract per image. '
                              'More features leads to better results but slower '
                              'execution. Default: %(default)s'))

    parser.add_argument('--matcher-neighbors',
                        type=int,
                        metavar='<integer>',
                        default=8,
                        help='Number of nearest images to pre-match based on GPS '
                             'exif data. Set to 0 to skip pre-matching. '
                             'Neighbors works together with Distance parameter, '
                             'set both to 0 to not use pre-matching. OpenSFM '
                             'uses both parameters at the same time, Bundler '
                             'uses only one which has value, prefering the '
                             'Neighbors parameter. Default: %(default)s')

    parser.add_argument('--matcher-distance',
                        metavar='<integer>',
                        default=0,
                        type=int,
                        help='Distance threshold in meters to find pre-matching '
                             'images based on GPS exif data. Set both '
                             'matcher-neighbors and this to 0 to skip '
                             'pre-matching. Default: %(default)s')

    parser.add_argument('--use-fixed-camera-params',
                        action='store_true',
                        default=False,
                        help='Turn off camera parameter optimization during bundler')

    parser.add_argument('--opensfm-processes',
                        metavar='<positive integer>',
                        default=context.num_cores,
                        type=int,
                        help=('The maximum number of processes to use in dense '
                              'reconstruction. Default: %(default)s'))

    parser.add_argument('--use-hybrid-bundle-adjustment',
                        action='store_true',
                        default=False,
                        help='Run local bundle adjustment for every image added to the reconstruction and a global '
                             'adjustment every 100 images. Speeds up reconstruction for very large datasets.')

    parser.add_argument('--use-25dmesh',
                    action='store_true',
                    default=False,
                    help='Use a 2.5D mesh to compute the orthophoto. This option tends to provide better results for planar surfaces. Experimental.')

    parser.add_argument('--use-pmvs',
                        action='store_true',
                        default=False,
                        help='Use pmvs to compute point cloud alternatively')

    parser.add_argument('--cmvs-maxImages',
                        metavar='<integer>',
                        default=500,
                        type=int,
                        help='The maximum number of images per cluster. '
                             'Default: %(default)s')

    parser.add_argument('--pmvs-level',
                        metavar='<positive integer>',
                        default=1,
                        type=int,
                        help=('The level in the image pyramid that is used '
                              'for the computation. see '
                              'http://www.di.ens.fr/pmvs/documentation.html for '
                              'more pmvs documentation. Default: %(default)s'))

    parser.add_argument('--pmvs-csize',
                        metavar='<positive integer>',
                        default=2,
                        type=int,
                        help='Cell size controls the density of reconstructions'
                             'Default: %(default)s')

    parser.add_argument('--pmvs-threshold',
                        metavar='<float: -1.0 <= x <= 1.0>',
                        default=0.7,
                        type=float,
                        help=('A patch reconstruction is accepted as a success '
                              'and kept if its associated photometric consistency '
                              'measure is above this threshold. Default: %(default)s'))

    parser.add_argument('--pmvs-wsize',
                        metavar='<positive integer>',
                        default=7,
                        type=int,
                        help='pmvs samples wsize x wsize pixel colors from '
                             'each image to compute photometric consistency '
                             'score. For example, when wsize=7, 7x7=49 pixel '
                             'colors are sampled in each image. Increasing the '
                             'value leads to more stable reconstructions, but '
                             'the program becomes slower. Default: %(default)s')

    parser.add_argument('--pmvs-min-images',
                        metavar='<positive integer>',
                        default=3,
                        type=int,
                        help=('Each 3D point must be visible in at least '
                              'minImageNum images for being reconstructed. 3 is '
                              'suggested in general. Default: %(default)s'))

    parser.add_argument('--pmvs-num-cores',
                        metavar='<positive integer>',
                        default=context.num_cores,
                        type=int,
                        help=('The maximum number of cores to use in dense '
                              'reconstruction. Default: %(default)s'))

    parser.add_argument('--mesh-size',
                        metavar='<positive integer>',
                        default=100000,
                        type=int,
                        help=('The maximum vertex count of the output mesh '
                              'Default: %(default)s'))

    parser.add_argument('--mesh-octree-depth',
                        metavar='<positive integer>',
                        default=9,
                        type=int,
                        help=('Oct-tree depth used in the mesh reconstruction, '
                              'increase to get more vertices, recommended '
                              'values are 8-12. Default: %(default)s'))

    parser.add_argument('--mesh-samples',
                        metavar='<float >= 1.0>',
                        default=1.0,
                        type=float,
                        help=('Number of points per octree node, recommended '
                              'and default value: %(default)s'))

    parser.add_argument('--mesh-solver-divide',
                        metavar='<positive integer>',
                        default=9,
                        type=int,
                        help=('Oct-tree depth at which the Laplacian equation '
                              'is solved in the surface reconstruction step. '
                              'Increasing this value increases computation '
                              'times slightly but helps reduce memory usage. '
                              'Default: %(default)s'))
    
    parser.add_argument('--mesh-neighbors',
                        metavar='<positive integer>',
                        default=24,
                        type=int,
                        help=('Number of neighbors to select when estimating the surface model used to compute the mesh and for statistical outlier removal. Higher values lead to smoother meshes but take longer to process. '
                              'Applies to 2.5D mesh only. '
                              'Default: %(default)s'))

    parser.add_argument('--mesh-resolution',
                        metavar='<positive float>',
                        default=0,
                        type=float,
                        help=('Size of the interpolated surface model used for deriving the 2.5D mesh, expressed in pixels per meter. '
                              'Higher values work better for complex or urban terrains. '
                              'Lower values work better on flat areas. '
                              'Resolution has no effect on the number of vertices, but high values can severely impact runtime speed and memory usage. '
                              'When set to zero, the program automatically attempts to find a good value based on the point cloud extent and target vertex count. '
                              'Applies to 2.5D mesh only. '
                              'Default: %(default)s'))

    parser.add_argument('--fast-orthophoto',
                action='store_true',
                default=False,
                help='Skips dense reconstruction and 3D model generation. '
                'It generates an orthophoto directly from the sparse reconstruction. '
                'If you just need an orthophoto and do not need a full 3D model, turn on this option. '
                'Experimental.')

    parser.add_argument('--crop',
                    metavar='<positive float>',
                    default=3,
                    type=float,
                    help=('Automatically crop image outputs by creating a smooth buffer '
                          'around the dataset boundaries, shrinked by N meters. '
                          'Use 0 to disable cropping. '
                          'Default: %(default)s'))

    parser.add_argument('--pc-classify',
            metavar='<string>',
            default='none',
            choices=['none', 'smrf', 'pmf'],
            help='Classify the .LAS point cloud output using either '
            'a Simple Morphological Filter or a Progressive Morphological Filter. '
            'If --dtm is set this parameter defaults to smrf. '
            'You can control the behavior of both smrf and pmf by tweaking the --dem-* parameters. '
            'Default: '
            '%(default)s')

    parser.add_argument('--texturing-data-term',
                        metavar='<string>',
                        default='gmi',
                        choices=['gmi', 'area'],
                        help=('Data term: [area, gmi]. Default: '
                              '%(default)s'))

    parser.add_argument('--texturing-outlier-removal-type',
                        metavar='<string>',
                        default='gauss_clamping',
                        choices=['none', 'gauss_clamping', 'gauss_damping'],
                        help=('Type of photometric outlier removal method: '
                              '[none, gauss_damping, gauss_clamping]. Default: '
                              '%(default)s'))

    parser.add_argument('--texturing-skip-visibility-test',
                        action='store_true',
                        default=False,
                        help=('Skip geometric visibility test. Default: '
                              ' %(default)s'))

    parser.add_argument('--texturing-skip-global-seam-leveling',
                        action='store_true',
                        default=False,
                        help=('Skip global seam leveling. Useful for IR data.'
                              'Default: %(default)s'))

    parser.add_argument('--texturing-skip-local-seam-leveling',
                        action='store_true',
                        default=False,
                        help='Skip local seam blending. Default:  %(default)s')

    parser.add_argument('--texturing-skip-hole-filling',
                        action='store_true',
                        default=False,
                        help=('Skip filling of holes in the mesh. Default: '
                              ' %(default)s'))

    parser.add_argument('--texturing-keep-unseen-faces',
                        action='store_true',
                        default=False,
                        help=('Keep faces in the mesh that are not seen in any camera. '
                              'Default:  %(default)s'))

    parser.add_argument('--texturing-tone-mapping',
                        metavar='<string>',
                        choices=['none', 'gamma'],
                        default='none',
                        help='Turn on gamma tone mapping or none for no tone '
                             'mapping. Choices are  \'gamma\' or \'none\'. '
                             'Default: %(default)s ')

    parser.add_argument('--gcp',
                        metavar='<path string>',
                        default=None,
                        help=('path to the file containing the ground control '
                              'points used for georeferencing.  Default: '
                              '%(default)s. The file needs to '
                              'be on the following line format: \neasting '
                              'northing height pixelrow pixelcol imagename'))

    parser.add_argument('--use-exif',
                        action='store_true',
                        default=False,
                        help=('Use this tag if you have a gcp_list.txt but '
                              'want to use the exif geotags instead'))

    parser.add_argument('--dtm',
                        action='store_true',
                        default=False,
                        help='Use this tag to build a DTM (Digital Terrain Model, ground only) using a progressive '
                             'morphological filter. Check the --dem* parameters for fine tuning.')

    parser.add_argument('--dsm',
                        action='store_true',
                        default=False,
                        help='Use this tag to build a DSM (Digital Surface Model, ground + objects) using a progressive '
                             'morphological filter. Check the --dem* parameters for fine tuning.')

    parser.add_argument('--dem-gapfill-steps',
                        metavar='<positive integer>',
                        default=4,
                        type=int,
                        help='Number of steps used to fill areas with gaps. Set to 0 to disable gap filling. '
                             'Starting with a radius equal to the output resolution, N different DEMs are generated with '
                             'progressively bigger radius using the inverse distance weighted (IDW) algorithm '
                             'and merged together. Remaining gaps are then merged using nearest neighbor interpolation. '
                             '\nDefault=%(default)s')

    parser.add_argument('--dem-resolution',
                        metavar='<float>',
                        type=float,
                        default=0.1,
                        help='Length of raster cell edges in meters.'
                             '\nDefault: %(default)s')

    parser.add_argument('--dem-maxangle',
                        metavar='<positive float>',
                        type=float,
                        default=20,
                        help='Points that are more than maxangle degrees off-nadir are discarded. '
                             '\nDefault: '
                             '%(default)s')

    parser.add_argument('--dem-maxsd',
                        metavar='<positive float>',
                        type=float,
                        default=2.5,
                        help='Points that deviate more than maxsd standard deviations from the local mean '
                             'are discarded. \nDefault: '
                             '%(default)s')

    parser.add_argument('--dem-initial-distance',
                        metavar='<positive float>',
                        type=float,
                        default=0.15,
                        help='Used to classify ground vs non-ground points. Set this value to account for Z noise in meters. '
                             'If you have an uncertainty of around 15 cm, set this value large enough to not exclude these points. '
                             'Too small of a value will exclude valid ground points, while too large of a value will misclassify non-ground points for ground ones. '
                             '\nDefault: '
                             '%(default)s')

    parser.add_argument('--dem-approximate',
                        action='store_true',
                        default=False,
                        help='Use this tag use the approximate progressive  '
                             'morphological filter, which computes DEMs faster '
                             'but is not as accurate.')

    parser.add_argument('--dem-decimation',
                        metavar='<positive integer>',
                        default=1,
                        type=int,
                        help='Decimate the points before generating the DEM. 1 is no decimation (full quality). '
                             '100 decimates ~99%% of the points. Useful for speeding up '
                             'generation.\nDefault=%(default)s')

    parser.add_argument('--dem-terrain-type',
                        metavar='<string>',
                        choices=['FlatNonForest', 'FlatForest', 'ComplexNonForest', 'ComplexForest'],
                        default='ComplexForest',
                        help='One of: %(choices)s. Specifies the type of terrain. This mainly helps reduce processing time. '
                             '\nFlatNonForest: Relatively flat region with little to no vegetation'
                             '\nFlatForest: Relatively flat region that is forested'
                             '\nComplexNonForest: Varied terrain with little to no vegetation'
                             '\nComplexForest: Varied terrain that is forested'
                             '\nDefault=%(default)s')

    parser.add_argument('--orthophoto-resolution',
                        metavar='<float > 0.0>',
                        default=20.0,
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
                        metavar='<string>',
                        type=str,
                        choices=['JPEG', 'LZW', 'PACKBITS', 'DEFLATE', 'LZMA', 'NONE'],
                        default='DEFLATE',
                        help='Set the compression to use. Note that this could '
                             'break gdal_translate if you don\'t know what you '
                             'are doing. Options: %(choices)s.\nDefault: %(default)s')

    parser.add_argument('--orthophoto-bigtiff',
                        type=str,
                        choices=['YES', 'NO','IF_NEEDED','IF_SAFER'],
                        default='IF_SAFER',
                        help='Control whether the created orthophoto is a BigTIFF or '
                             'classic TIFF. BigTIFF is a variant for files larger than '
                             '4GiB of data. Options are %(choices)s. See GDAL specs: '
                             'https://www.gdal.org/frmt_gtiff.html for more info. '
                             '\nDefault: %(default)s')

    parser.add_argument('--build-overviews',
                        action='store_true',
                        default=False,
                        help='Build orthophoto overviews using gdaladdo.')

    parser.add_argument('--zip-results',
                        action='store_true',
                        default=False,
                        help='compress the results using gunzip')

    parser.add_argument('--verbose', '-v',
                        action='store_true',
                        default=False,
                        help='Print additional messages to the console\n'
                             'Default: %(default)s')

    parser.add_argument('--time',
                        action='store_true',
                        default=False,
                        help='Generates a benchmark file with runtime info\n'
                             'Default: %(default)s')

    parser.add_argument('--version',
                        action='version',
                        version='OpenDroneMap {0}'.format(__version__),
                        help='Displays version number and exits. ')

    args = parser.parse_args()

    # check that the project path setting has been set properly
    if not args.project_path:
        log.ODM_ERROR('You need to set the project path in the '
                      'settings.yaml file before you can run ODM, '
                      'or use `--project-path <path>`. Run `python '
                      'run.py --help` for more information. ')
        sys.exit(1)

    if args.fast_orthophoto:
      log.ODM_INFO('Fast orthophoto is turned on, automatically setting --use-25dmesh')
      args.use_25dmesh = True

      # Cannot use pmvs
      if args.use_pmvs:
        log.ODM_INFO('Fast orthophoto is turned on, cannot use pmvs (removing --use-pmvs)')
        args.use_pmvs = False

    if args.dtm and args.pc_classify == 'none':
      log.ODM_INFO("DTM is turned on, automatically turning on point cloud classification")
      args.pc_classify = "smrf"

    return args
