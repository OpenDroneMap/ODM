#!/usr/bin/env python

import os
import subprocess
import ecto
from opendm import context
from opendm import log

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

        command = os.path.join(context.opensfm_path, 'bin', 'opensfm')
        path = tree.opensfm

        run_command([command, 'extract_metadata', path])
        run_command([command, 'detect_features', path])
        run_command([command, 'match_features', path])