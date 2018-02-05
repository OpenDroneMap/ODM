#!/usr/bin/env python

import argparse
import logging
import multiprocessing
import os
import subprocess

from opensfm.large import metadataset

from opendm import context

logger = logging.getLogger(__name__)

logging.basicConfig(format='%(asctime)s %(levelname)s: %(message)s',
                    level=logging.INFO)


def run_command(args):
    result = subprocess.Popen(args).wait()
    if result != 0:
        logger.error("The command '{}' exited with return value {}". format(
            ' '.join(args), result))


class DenseReconstructor:
    def __init__(self, command):
        self.command = command

    def __call__(self, opensfm_submodel_path):
        submodel_path = os.path.dirname(opensfm_submodel_path.rstrip('/'))

        logger.info("=======================================================")
        logger.info("Dense reconstruction submodel {}".format(submodel_path))
        logger.info("=======================================================")

        # Rename reconstruction.aligned.json
        unaligned = os.path.join(opensfm_submodel_path, 'reconstruction.unaligned.json')
        aligned = os.path.join(opensfm_submodel_path, 'reconstruction.aligned.json')
        main = os.path.join(opensfm_submodel_path, 'reconstruction.json')

        if not os.path.isfile(aligned):
            logger.warning("No SfM reconstruction for submodel {}."
                           " Skipping submodel.".format(submodel_path))
            return

        if not os.path.isfile(unaligned):
            os.rename(main, unaligned)
        if not os.path.islink(main):
            os.symlink(aligned, main)

        path, name = os.path.split(submodel_path)
        run_command(['python',
                     self.command,
                     '--project-path', path,
                     name,
                     '--start-with', 'opensfm'])

        logger.info("=======================================================")
        logger.info("Submodel {} reconstructed".format(submodel_path))
        logger.info("=======================================================")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Reconstruct all submodels')
    parser.add_argument('dataset',
                        help='path to the dataset to be processed')
    args = parser.parse_args()

    path = os.path.join(args.dataset, 'opensfm')
    meta_data = metadataset.MetaDataSet(path)
    command = os.path.join(context.root_path, 'run.py')

    submodel_paths = meta_data.get_submodel_paths()
    reconstructor = DenseReconstructor(command)

    processes = 1
    if processes == 1:
        for submodel_path in submodel_paths:
            reconstructor(submodel_path)
    else:
        p = multiprocessing.Pool(processes)
        p.map(reconstructor, submodel_paths)
