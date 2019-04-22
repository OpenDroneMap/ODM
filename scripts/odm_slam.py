"""Cell to run odm_slam."""

import os

from opendm import log
from opendm import io
from opendm import system
from opendm import context
from opendm import types

class ODMSlamStage(types.ODM_Stage):
    """Run odm_slam on a video and export to opensfm format."""

    def process(self, args, outputs):
        tree = outputs['tree']
        video = os.path.join(tree.root_path, args.video)
        slam_config = os.path.join(tree.root_path, args.slam_config)

        if not video:
            log.ODM_ERROR('No video provided')
            exit(1)

        # create working directories
        system.mkdir_p(tree.opensfm)

        vocabulary = os.path.join(context.orb_slam2_path,
                                  'Vocabulary/ORBvoc.txt')
        orb_slam_cmd = os.path.join(context.odm_modules_path, 'odm_slam')
        trajectory = os.path.join(tree.opensfm, 'KeyFrameTrajectory.txt')
        map_points = os.path.join(tree.opensfm, 'MapPoints.txt')

        # check if slam was run before
        if not io.file_exists(trajectory) or self.rerun():
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
        if not io.file_exists(tree.opensfm_reconstruction) or self.rerun():
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
        if not io.file_exists(tree.opensfm_bundle_list) or self.rerun():
            # convert back to bundler's format
            system.run(
                'PYTHONPATH={} {}/bin/export_bundler {}'.format(
                    context.pyopencv_path, context.opensfm_path, tree.opensfm))
        else:
            log.ODM_WARNING(
                'Found a valid Bundler file in: {}'.format(
                    tree.opensfm_reconstruction))

