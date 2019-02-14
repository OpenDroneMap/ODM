#!/usr/bin/env python

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



class Reconstructor:
    def __init__(self, command, run_matching):
        self.command = command
        self.run_matching = run_matching

    def __call__(self, submodel_path):
        log.ODM_INFO("=======================================================")
        log.ODM_INFO("Reconstructing submodel {}".format(submodel_path))
        log.ODM_INFO("=======================================================")

        if self.run_matching:
            run_command([self.command, 'extract_metadata', submodel_path])
            run_command([self.command, 'detect_features', submodel_path])
            run_command([self.command, 'match_features', submodel_path])

        self._set_matching_done(submodel_path)

        run_command([self.command, 'create_tracks', submodel_path])
        run_command([self.command, 'reconstruct', submodel_path])

        log.ODM_INFO("=======================================================")
        log.ODM_INFO("Submodel {} reconstructed".format(submodel_path))
        log.ODM_INFO("=======================================================")

    def _set_matching_done(self, submodel_path):
        """Tell ODM's opensfm not to rerun matching."""
        matching_done_file = os.path.join(submodel_path, 'matching_done.txt')
        with open(matching_done_file, 'w') as fout:
            fout.write("Matching done!\n")


class SMReconstructionCell(ecto.Cell):

    # def declare_params(self, params):

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("sm_meta", "SplitMerge metadata", [])
        outputs.declare("sm_meta", "SplitMerge metadata", [])

    def process(self, inputs, outputs):
        args = self.inputs.args
        tree = self.inputs.tree

        command = os.path.join(context.opensfm_path, 'bin', 'opensfm')
        path = tree.opensfm

        meta_data = metadataset.MetaDataSet(path)

        if True: # util.check_rerun(args, 'sm_reconstruction'):
            submodel_paths = meta_data.get_submodel_paths()
            reconstructor = Reconstructor(command, args.run_matching)

            processes = 1 # meta_data.config['processes']
            if processes == 1:
                for submodel_path in submodel_paths:
                    reconstructor(submodel_path)
            else:
                p = multiprocessing.Pool(processes)
                p.map(reconstructor, submodel_paths)
        else:
            log.ODM_DEBUG("Skipping Reconstruction")

        return ecto.OK if args.end_with != 'sm_reconstruction' else ecto.QUIT
