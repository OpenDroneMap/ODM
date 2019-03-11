#!/usr/bin/env python

import os
import subprocess
import ecto
from opendm import context
from opendm import log
from opendm import util

def run_command(args):
    result = subprocess.Popen(args).wait()
    if result != 0:
        log.ODM_ERROR("The command '{}' exited with return value {}". format(
            ' '.join(args), result))


class SMMatchingCell(ecto.Cell):

    # def declare_params(self, params):

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("sm_meta", "SplitMerge metadata", [])
        outputs.declare("sm_meta", "SplitMerge metadata", [])

    def process(self, inputs, outputs):
        args = self.inputs.args
        tree = self.inputs.tree
        sm_meta = self.inputs.sm_meta
        result = 0

        # check if we rerun cell or not
        if sm_meta.progress < 1:  # util.is_run(args, 'sm_reconstruction') or sm_meta.progress < 1:
            command = os.path.join(context.opensfm_path, 'bin', 'opensfm')
            path = tree.opensfm

            run_command([command, 'extract_metadata', path])
            run_command([command, 'detect_features', path])
            run_command([command, 'match_features', path])

            sm_meta.update_progress(1)
            sm_meta.save_progress(tree.sm_progress)
        else: 
            log.ODM_DEBUG("Skipping Matching")

        self.outputs.sm_meta = sm_meta

        return ecto.OK if args.end_with != 'sm_matching' else ecto.QUIT
