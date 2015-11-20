import argparse

# parse arguments
processopts = ['resize', 'opensfm', 'cmvs', 'pmvs',
               'odm_meshing', 'odm_texturing', 'odm_georeferencing',
               'odm_orthophoto']

parser = argparse.ArgumentParser(description='OpenDroneMap')
parser.add_argument('--images-src', 
                    metavar='<string>',
                    help='Path to the images to process')

parser.add_argument('--resize-to',  # currently doesn't support 'orig'
                    metavar='<integer>',
                    default=2400,
                    type=int,
                    help='resizes images by the largest side')

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

parser.add_argument('--run-only',
                    metavar='<string>',
                    choices=processopts,
                    help=('Can be one of:' + ' | '.join(processopts)))

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
                    default=4000,
                    type=int,
                    help=('Minimum number of features to extract per image. '
                          'More features leads to better results but slower '
                          'execution.'))

parser.add_argument('--matcher-threshold',
                    metavar='<percent>',
                    default=2.0,
                    type=float,
                    help=('Ignore matched keypoints if the two images share '
                            'less than <float> percent of keypoints'))

parser.add_argument('--matcher-ratio',
                    metavar='<float>',
                    default=0.6,
                    type=float,
                    help=('Ratio of the distance to the next best matched '
                            'keypoint'))

parser.add_argument('--matcher-preselect',
                    type=bool,
                    metavar='',
                    default=False,
                    help=('use GPS exif data, if available, to match each '
                            'image only with its k-nearest neighbors, or all '
                            'images within a certain distance threshold'))

parser.add_argument('--matcher-useKnn',
                    type=bool,
                    metavar='',
                    default=True,
                    help=('use GPS exif data, if available, to match each '
                            'image only with its k-nearest neighbors, or all '
                            'images within a certain distance threshold'))

parser.add_argument('--matcher-kDistance',
                    metavar='<integer>',
                    default=20,
                    type=int,
                    help='')

parser.add_argument('--matcher-k',
                    metavar='<integer>',
                    default=8,
                    type=int,
                    help='Number of k-nearest images to match '
                         'when using OpenSfM')

parser.add_argument('--cmvs-maxImages',
                    metavar='<integer>',
                    default=500,
                    type=int,
                    help='The maximum number of images per cluster')

parser.add_argument('--pmvs-level',
                    metavar='<positive integer>',
                    default=1,
                    type=int,
                    help=('The level in the image pyramid that is used '
                            'for the computation. see '
                            'http://www.di.ens.fr/pmvs/documentation.html for '
                            'more pmvs documentation'))

parser.add_argument('--pmvs-csize',
                    metavar='< positive integer>',
                    default=2,
                    type=int,
                    help='Cell size controls the density of reconstructions')

parser.add_argument('--pmvs-threshold',
                    metavar='<float: -1.0 <= x <= 1.0>',
                    default=0.7,
                    type=float,
                    help=('A patch reconstruction is accepted as a success '
                            'and kept, if its associcated photometric consistency '
                            'measure is above this threshold.'))

parser.add_argument('--pmvs-wsize',
                    metavar='<positive integer>',
                    default=7,
                    type=int,
                    help=('pmvs samples wsize x wsize pixel colors from '
                            'each image to compute photometric consistency '
                            'score. For example, when wsize=7, 7x7=49 pixel '
                            'colors are sampled in each image. Increasing the '
                            'value leads to more stable reconstructions, but '
                            'the program becomes slower.'))

parser.add_argument('--pmvs-minImageNum',
                    metavar='<positive integer>',
                    default=3,
                    type=int,
                    help=('Each 3D point must be visible in at least '
                            'minImageNum images for being reconstructed. 3 is '
                            'suggested in general.'))

parser.add_argument('--odm_meshing-maxVertexCount',
                    metavar='<positive integer>',
                    default=100000,
                    type=int,
                    help='The maximum vertex count of the output mesh')

parser.add_argument('--odm_meshing-octreeDepth',
                    metavar='<positive integer>',
                    default=9,
                    type=int,
                    help=('Oct-tree depth used in the mesh reconstruction, '
                            'increase to get more vertices, recommended '
                            'values are 8-12'))

parser.add_argument('--odm_meshing-samplesPerNode',
                    metavar='<float >= 1.0>',
                    default=1,
                    type=float,
                    help=('Number of points per octree node, recommended '
                            'value: 1.0'))

parser.add_argument('--odm_meshing-solverDivide',
                    metavar='<positive integer>',
                    default=9,
                    type=int,
                    help=('Oct-tree depth at which the Laplacian equation '
                            'is solved in the surface reconstruction step. '
                            'Increasing this value increases computation '
                            'times slightly but helps reduce memory usage.'))

parser.add_argument('--odm_texturing-textureResolution',
                    metavar='<positive integer>',
                    default=4096,
                    type=int,
                    help=('The resolution of the output textures. Must be '
                            'greater than textureWithSize.'))

parser.add_argument('--odm_texturing-textureWithSize',
                    metavar='<positive integer>',
                    default=3600,
                    type=int,
                    help=('The resolution to rescale the images performing '
                            'the texturing.'))

parser.add_argument('--odm_georeferencing-gcpFile',
                    metavar='<path string>',
                    default='gcp_list.txt',
                    help=('path to the file containing the ground control '
                            'points used for georeferencing.The file needs to '
                            'be on the following line format: \neasting '
                            'northing height pixelrow pixelcol imagename'))

parser.add_argument('--odm_georeferencing-useGcp',
                    type = bool,
                    default = False,
                    help = 'set to true for enabling GCPs from the file above')

parser.add_argument('--odm_orthophoto-resolution',
                    metavar='<float > 0.0>',
                    default=20.0,
                    type=float,
                    help=('Orthophoto ground resolution in pixels/meter'))

parser.add_argument('--zip-results',
                    action='store_true',
                    default=False,
                    help='compress the results using gunzip')

parser.add_argument('--use-opensfm',
                    type=bool,
                    default=True,
                    help='use OpenSfM instead of Bundler to find the camera positions '
                         '(replaces getKeypoints, match and bundler steps)')

args = vars(parser.parse_args())