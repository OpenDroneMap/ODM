#!/usr/bin/env python

"""Setup an ODM metadataset.

A metadatase will be split into multiple submodel folders.
Each submodel is reconstructed independently. Before dense
reconstruction the different submodels are aligned to each
other.
"""

import argparse
import os
import logging
import subprocess

import yaml

from opensfm.io import mkdir_p

from opendm import context

logger = logging.getLogger(__name__)

logging.basicConfig(format='%(asctime)s %(levelname)s: %(message)s',
                    level=logging.INFO)


def run_command(args):
    result = subprocess.Popen(args).wait()
    if result != 0:
        logger.error("The command '{}' exited with return value {}". format(
            ' '.join(args), result))


def resize_images(data_path, args):
    command = os.path.join(context.root_path, 'run.py')
    path, name = os.path.split(data_path.rstrip('/'))
    run_command(['python',
                 command,
                 '--project-path', path,
                 name,
                 '--resize-to', str(args.resize_to),
                 '--end-with', 'dataset',
                 ])


def is_image_file(filename):
    extensions = {'jpg', 'jpeg', 'png', 'tif', 'tiff', 'pgm', 'pnm', 'gif'}
    return filename.split('.')[-1].lower() in extensions


def create_image_list(image_path, opensfm_path):
    image_files = filter(is_image_file, os.listdir(image_path))

    lines = []
    relpath = os.path.relpath(image_path, opensfm_path)
    for image in image_files:
        lines.append(os.path.join(relpath, image))

    with open(os.path.join(opensfm_path, 'image_list.txt'), 'w') as fout:
        fout.write("\n".join(lines))


def create_config(opensfm_path, args):
    config = {
        "submodels_relpath": "../submodels/opensfm",
        "submodel_relpath_template": "../submodels/submodel_%04d/opensfm",
        "submodel_images_relpath_template": "../submodels/submodel_%04d/images",
        "submodel_size": args.submodel_size,
        "submodel_overlap": args.submodel_overlap,

        "feature_process_size": args.resize_to,
        "feature_min_frames": args.min_num_features,
        "processes": args.num_cores,
        "matching_gps_neighbors": args.matcher_neighbors,
    }
    with open(os.path.join(opensfm_path, 'config.yaml'), 'w') as fout:
        yaml.dump(config, fout, default_flow_style=False)


def link_image_groups(data_path, opensfm_path):
    src = os.path.join(data_path, 'image_groups.txt')
    dst = os.path.join(opensfm_path, 'image_groups.txt')
    if os.path.isfile(src) and not os.path.isfile(dst):
        os.symlink(src, dst)


def parse_command_line():
    parser = argparse.ArgumentParser(description='Setup an ODM metadataset')
    parser.add_argument('dataset',
                        help='path to the dataset to be processed')

    # TODO(pau): reduce redundancy with OpenDroneMap/opendm/config.py

    parser.add_argument('--resize-to',  # currently doesn't support 'orig'
                        metavar='<integer>',
                        default=2400,
                        type=int,
                        help='resizes images by the largest side')

    parser.add_argument('--min-num-features',
                        metavar='<integer>',
                        default=4000,
                        type=int,
                        help=('Minimum number of features to extract per image. '
                              'More features leads to better results but slower '
                              'execution. Default: %(default)s'))

    parser.add_argument('--num-cores',
                        metavar='<positive integer>',
                        default=4,
                        type=int,
                        help=('The maximum number of cores to use. '
                              'Default: %(default)s'))

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

    parser.add_argument('--submodel-size',
                        type=int,
                        default=80,
                        help='Average number of images per submodel. When '
                             'splitting a large dataset into smaller '
                             'submodels, images are grouped into clusters. '
                             'This value regulates the number of images that '
                             'each cluster should have on average.')

    parser.add_argument('--submodel-overlap',
                        type=float,
                        metavar='<positive integer>',
                        default=150,
                        help='Radius of the overlap between submodels. '
                        'After grouping images into clusters, images '
                        'that are closer than this radius to a cluster '
                        'are added to the cluster. This is done to ensure '
                        'that neighboring submodels overlap.')

    return parser.parse_args()


if __name__ == '__main__':
    args = parse_command_line()
    data_path = args.dataset

    resize_images(data_path, args)

    image_path = os.path.join(data_path, 'images')
    opensfm_path = os.path.join(data_path, 'opensfm')

    mkdir_p(opensfm_path)
    create_image_list(image_path, opensfm_path)
    create_config(opensfm_path, args)
    link_image_groups(data_path, opensfm_path)
