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


class Reconstructor:
    def __init__(self, command, run_matching):
        self.command = command
        self.run_matching = run_matching

    def __call__(self, submodel_path):
        logger.info("=======================================================")
        logger.info("Reconstructing submodel {}".format(submodel_path))
        logger.info("=======================================================")

        if self.run_matching:
            run_command([self.command, 'extract_metadata', submodel_path])
            run_command([self.command, 'detect_features', submodel_path])
            run_command([self.command, 'match_features', submodel_path])

        self._set_matching_done(submodel_path)

        run_command([self.command, 'create_tracks', submodel_path])
        run_command([self.command, 'reconstruct', submodel_path])

        logger.info("=======================================================")
        logger.info("Submodel {} reconstructed".format(submodel_path))
        logger.info("=======================================================")

    def _set_matching_done(self, submodel_path):
        """Tell ODM's opensfm not to rerun matching."""
        matching_done_file = os.path.join(submodel_path, 'matching_done.txt')
        with open(matching_done_file, 'w') as fout:
            fout.write("Matching done!\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Reconstruct all submodels')
    parser.add_argument('dataset',
                        help='path to the dataset to be processed')
    parser.add_argument('--run-matching',
                        help='Run matching for each submodel',
                        action='store_true')
    args = parser.parse_args()

    path = os.path.join(args.dataset, 'opensfm')
    meta_data = metadataset.MetaDataSet(path)
    command = os.path.join(context.opensfm_path, 'bin', 'opensfm')

    submodel_paths = meta_data.get_submodel_paths()
    reconstructor = Reconstructor(command, args.run_matching)

    processes = meta_data.config['processes']
    if processes == 1:
        for submodel_path in submodel_paths:
            reconstructor(submodel_path)
    else:
        p = multiprocessing.Pool(processes)
        p.map(reconstructor, submodel_paths)
