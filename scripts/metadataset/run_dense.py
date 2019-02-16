#!/usr/bin/env python

import argparse
import multiprocessing
import os
import subprocess
import ecto
from opensfm.large import metadataset
from opendm import util
from opendm import context
from opendm import log


def run_command(args):
    result = subprocess.Popen(args).wait()
    if result != 0:
	log.ODM_ERROR("The command '{}' exited with return value {}". format(
        	' '.join(args), result))


class DenseReconstructor:
    def __init__(self, command):
        self.command = command

    def __call__(self, opensfm_submodel_path):
        submodel_path = os.path.dirname(opensfm_submodel_path.rstrip('/'))

        log.ODM_INFO("=======================================================")
        log.ODM_INFO("Dense reconstruction submodel {}".format(submodel_path))
        log.ODM_INFO("=======================================================")

        # Rename reconstruction.aligned.json
        unaligned = os.path.join(opensfm_submodel_path, 'reconstruction.unaligned.json')
        aligned = os.path.join(opensfm_submodel_path, 'reconstruction.aligned.json')
        main = os.path.join(opensfm_submodel_path, 'reconstruction.json')

        if not os.path.isfile(aligned):
            log.ODM_WARNING("No SfM reconstruction for submodel {}."
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

        log.ODM_INFO("=======================================================")
        log.ODM_INFO("Submodel {} reconstructed".format(submodel_path))
        log.ODM_INFO("=======================================================")


class SMDenseCell(ecto.Cell):

    # def declare_params(self, params):
        #todo check if i can drop this

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("sm_meta", "SplitMerge metadata", [])
        outputs.declare("sm_meta", "SplitMerge metadata", [])

    def process(self, inputs, outputs):
        args = self.inputs.args
        tree = self.inputs.tree

        if True: # util.check_rerun(args, 'sm_dense'):
            command = os.path.join(context.root_path, 'run.py')
            path = tree.opensfm
            meta_data = metadataset.MetaDataSet(path)

            submodel_paths = meta_data.get_submodel_paths()
            reconstructor = DenseReconstructor(command)

            processes = args.max_concurrency
            if processes == 1:
                log.ODM_DEBUG("Running Dense")
                for submodel_path in submodel_paths:
                    log.ODM_DEBUG("Submodel {}".format(submodel_path))
                    reconstructor(submodel_path)
            else:
                p = multiprocessing.Pool(processes)
                p.map(reconstructor, submodel_paths)

        log.ODM_DEBUG("What")
        return ecto.OK if args.end_with != 'sm_dense' else ecto.QUIT
