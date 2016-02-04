import os

import ecto

from opendm import log
from opendm import system
from opendm import context


class ODMSlamCell(ecto.Cell):
    def declare_params(self, params):
        pass

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        outputs.declare("reconstruction", "list of ODMReconstructions", [])

    def process(self, inputs, outputs):

        log.ODM_INFO('Running OMD Slam Cell')

        # get inputs
        tree = self.inputs.tree
        args = self.inputs.args
        video = os.path.join(tree.root_path, args['video'])

        if not video:
            log.ODM_ERROR('No video provided')
            return ecto.QUIT

        # create working directories
        system.mkdir_p(tree.opensfm)
        system.mkdir_p(tree.pmvs)

        # run meshing binary
        system.run(
            '{}/odm_slam '
            'SuperBuild/src/orb_slam2/Vocabulary/ORBvoc.txt '
            'SuperBuild/src/orb_slam2/Examples/Monocular/TUM1.yaml '
            '{}'.format(context.odm_modules_path, video))

        log.ODM_INFO('Running OMD Slam Cell - Finished')
        return ecto.OK if args['end_with'] != 'opensfm' else ecto.QUIT
