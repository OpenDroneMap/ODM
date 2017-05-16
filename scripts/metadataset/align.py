#!/usr/bin/env python

import argparse
import os
import subprocess

from opendm import context


def run_command(args):
    result = subprocess.Popen(args).wait()
    if result != 0:
        raise RuntimeError(result)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Align metadaset submodels')
    parser.add_argument('dataset',
                        help='path to the dataset to be processed')
    args = parser.parse_args()

    command = os.path.join(context.opensfm_path, 'bin', 'opensfm')
    path = os.path.join(args.dataset, 'opensfm')

    run_command([command, 'align_submodels', path])
