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


class SMSplitCell(ecto.Cell):

    # def declare_params(self, params):
    #     #todo check if i can drop this

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("sm_meta", "SplitMerge metadata", [])
        outputs.declare("sm_meta", "SplitMerge metadata", [])

    def process(self, inputs, outputs):
        args = self.inputs.args
        tree = self.inputs.tree
        sm_meta = self.inputs.sm_meta

        command = os.path.join(context.opensfm_path, 'bin', 'opensfm')
        path = tree.opensfm

        if sm_meta.progress < 2: ## True: # util.check_rerun(args, 'sm_split'):
            log.ODM_DEBUG("Running Split")
            run_command([command, 'create_submodels', path])
            sm_meta.update_progress(2)
            sm_meta.save_progress(tree.sm_progress)
        else: 
            log.ODM_DEBUG("Skipping Split")

        self.outputs.sm_meta = sm_meta

        return ecto.OK if args.end_with != 'sm_split' else ecto.QUIT
