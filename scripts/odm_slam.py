"""Cell to run odm_slam."""

import os

import ecto

from opendm import log
from opendm import io
from opendm import system
from opendm import context


class ODMSlamCell(ecto.Cell):
    """Run odm_slam on a video and export to opensfm format."""

    def declare_params(self, params):
        """Cell parameters."""
        pass

    def declare_io(self, params, inputs, outputs):
        """Cell inputs and outputs."""
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        outputs.declare("reconstruction", "list of ODMReconstructions", [])

    def process(self, inputs, outputs):
        """Run the cell."""
        log.ODM_INFO('Running OMD Slam Cell')

        # get inputs
        tree = self.inputs.tree
        args = self.inputs.args
        video = os.path.join(tree.root_path, args.video)
        slam_config = os.path.join(tree.root_path, args.slam_config)

        if not video:
            log.ODM_ERROR('No video provided')
            return ecto.QUIT

        # create working directories
        system.mkdir_p(tree.opensfm)
        system.mkdir_p(tree.pmvs)

        vocabulary = os.path.join(context.orb_slam2_path,
                                  'Vocabulary/ORBvoc.txt')
        orb_slam_cmd = os.path.join(context.odm_modules_path, 'odm_slam')
        trajectory = os.path.join(tree.opensfm, 'KeyFrameTrajectory.txt')
        map_points = os.path.join(tree.opensfm, 'MapPoints.txt')

        # check if we rerun cell or not
        rerun_cell = args.rerun == 'slam'

        # check if slam was run before
        if not io.file_exists(trajectory) or rerun_cell:
            # run slam binary
            system.run(' '.join([
                'cd {} &&'.format(tree.opensfm),
                orb_slam_cmd,
                vocabulary,
                slam_config,
                video,
            ]))
        else:
            log.ODM_WARNING('Found a valid slam trajectory in: {}'.format(
                trajectory))

        # check if trajectory was exported to opensfm before
        if not io.file_exists(tree.opensfm_reconstruction) or rerun_cell:
            # convert slam to opensfm
            system.run(' '.join([
                'cd {} &&'.format(tree.opensfm),
                'PYTHONPATH={}:{}'.format(context.pyopencv_path,
                                          context.opensfm_path),
                'python',
                os.path.join(context.odm_modules_src_path,
                             'odm_slam/src/orb_slam_to_opensfm.py'),
                video,
                trajectory,
                map_points,
                slam_config,
            ]))
            # link opensfm images to resized images
            os.symlink(tree.opensfm + '/images', tree.dataset_resize)
        else:
            log.ODM_WARNING('Found a valid OpenSfM file in: {}'.format(
                tree.opensfm_reconstruction))

        # check if reconstruction was exported to bundler before
        if not io.file_exists(tree.opensfm_bundle_list) or rerun_cell:
            # convert back to bundler's format
            system.run(
                'PYTHONPATH={} {}/bin/export_bundler {}'.format(
                    context.pyopencv_path, context.opensfm_path, tree.opensfm))
        else:
            log.ODM_WARNING(
                'Found a valid Bundler file in: {}'.format(
                    tree.opensfm_reconstruction))

        # check if reconstruction was exported to pmvs before
        if not io.file_exists(tree.pmvs_visdat) or rerun_cell:
            # run PMVS converter
            system.run(
                'PYTHONPATH={} {}/bin/export_pmvs {} --output {}'.format(
                    context.pyopencv_path, context.opensfm_path, tree.opensfm,
                    tree.pmvs))
        else:
            log.ODM_WARNING('Found a valid CMVS file in: {}'.format(
                tree.pmvs_visdat))

        log.ODM_INFO('Running OMD Slam Cell - Finished')
        return ecto.OK if args.end_with != 'odm_slam' else ecto.QUIT
