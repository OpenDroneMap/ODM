import os

from opendm import io
from opendm import log
from opendm import types
from opendm.utils import copy_paths, get_processing_results_paths
from opendm.ogctiles import build_3dtiles

class ODMPostProcess(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']

        log.ODM_INFO("Post Processing")

        if getattr(args, '3d_tiles'):
            build_3dtiles(args, tree, reconstruction, self.rerun())

        if args.copy_to:
            try:
                copy_paths([os.path.join(args.project_path, p) for p in get_processing_results_paths()], args.copy_to, self.rerun())
            except Exception as e:
                log.ODM_WARNING("Cannot copy to %s: %s" % (args.copy_to, str(e)))

