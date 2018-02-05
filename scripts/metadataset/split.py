#!/usr/bin/env python

import argparse
import logging
import os
import subprocess

from opendm import context

logger = logging.getLogger(__name__)

logging.basicConfig(format='%(asctime)s %(levelname)s: %(message)s',
                    level=logging.INFO)


def run_command(args):
    result = subprocess.Popen(args).wait()
    if result != 0:
        logger.error("The command '{}' exited with return value {}". format(
            ' '.join(args), result))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Split metadaset into submodels')
    parser.add_argument('dataset',
                        help='path to the dataset to be processed')
    args = parser.parse_args()

    command = os.path.join(context.opensfm_path, 'bin', 'opensfm')
    path = os.path.join(args.dataset, 'opensfm')

    run_command([command, 'create_submodels', path])
