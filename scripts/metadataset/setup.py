#!/usr/bin/env python

"""Setup an ODM metadataset.

A metadatase will be split into multiple submodel folders.
Each submodel is reconstructed independently. Before dense
reconstruction the different submodels are aligned to each
other.
"""

import os
import subprocess
import ecto
import yaml

from opensfm.io import mkdir_p

from opendm import context
from opendm import log

def run_command(args):
    result = subprocess.Popen(args).wait()
    if result != 0:
        log.ODM_ERROR("The command '{}' exited with return value {}". format(
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
        "processes": args.max_concurrency,
        "matching_gps_neighbors": args.matcher_neighbors,
    }
    with open(os.path.join(opensfm_path, 'config.yaml'), 'w') as fout:
        yaml.dump(config, fout, default_flow_style=False)


def link_image_groups(data_path, opensfm_path):
    src = os.path.join(data_path, 'image_groups.txt')
    dst = os.path.join(opensfm_path, 'image_groups.txt')
    if os.path.isfile(src) and not os.path.isfile(dst):
        os.symlink(src, dst)

class SMSetupCell(ecto.Cell):

    def declare_params(self, params):
        params.declare("verbose", "indicate verbosity", False)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        outputs.declare("sm_meta", "ODMReconstruction", [])
        inputs.declare("args", "The application arguments.", {})


    def process(self, inputs, outputs):
        args = self.inputs.args
        tree = self.inputs.tree
        data_path = tree.root_path

        image_path = tree.dataset_raw
        opensfm_path = tree.opensfm

        mkdir_p(opensfm_path)
        create_image_list(image_path, opensfm_path)
        create_config(opensfm_path, args)
        link_image_groups(data_path, opensfm_path)