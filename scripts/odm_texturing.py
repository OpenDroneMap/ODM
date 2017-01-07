import os

import ecto

from opendm import log
from opendm import io
from opendm import system
from opendm import context


class ODMTexturingCell(ecto.Cell):
    def declare_params(self, params):
        params.declare("resize", 'resizes images by the largest side', 2400)
        params.declare("resolution", 'The resolution of the output textures. Must be '
                                     'greater than textureWithSize.', 4096)
        params.declare("size", 'The resolution to rescale the images performing '
                               'the texturing.', 3600)
        params.declare("verbose", 'print additional messages to console', False)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("reconstruction", "Clusters output. list of ODMReconstructions", [])
        outputs.declare("reconstruction", "Clusters output. list of ODMReconstructions", [])

    def process(self, inputs, outputs):

        # Benchmarking
        start_time = system.now_raw()

        log.ODM_INFO('Running ODM Texturing Cell')

        # get inputs
        args = self.inputs.args
        tree = self.inputs.tree
        verbose = '-verbose' if self.params.verbose else ''

        # define paths and create working directories
        system.mkdir_p(tree.odm_texturing)

        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'odm_texturing') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'odm_texturing' in args.rerun_from)

        # Undistort radial distortion
        if not os.path.isdir(tree.odm_texturing_undistorted_image_path) or rerun_cell:
            system.run(' '.join([
                'cd {} &&'.format(tree.opensfm),
                'PYTHONPATH={}:{}'.format(context.pyopencv_path,
                                          context.opensfm_path),
                'python',
                os.path.join(context.odm_modules_src_path,
                             'odm_slam/src/undistort_radial.py'),
                '--output',
                tree.odm_texturing_undistorted_image_path,
                tree.opensfm,
            ]))

            system.run(
                'PYTHONPATH=%s %s/bin/export_bundler %s' %
                (context.pyopencv_path, context.opensfm_path, tree.opensfm))
        else:
            log.ODM_WARNING(
                'Found a valid Bundler file in: %s' %
                (tree.opensfm_reconstruction))

        if not io.file_exists(tree.odm_textured_model_obj) or rerun_cell:
            log.ODM_DEBUG('Writing ODM Textured file in: %s'
                          % tree.odm_textured_model_obj)

            # odm_texturing definitions
            kwargs = {
                'bin': context.odm_modules_path,
                'out_dir': tree.odm_texturing,
                'bundle': tree.opensfm_bundle,
                'imgs_path': tree.odm_texturing_undistorted_image_path,
                'imgs_list': tree.opensfm_bundle_list,
                'model': tree.odm_mesh,
                'log': tree.odm_texuring_log,
                'resize': self.params.resize,
                'resolution': self.params.resolution,
                'size': self.params.size,
                'verbose': verbose
            }

            # run texturing binary
            system.run('{bin}/odm_texturing -bundleFile {bundle} '
                       '-imagesPath {imgs_path} -imagesListPath {imgs_list} '
                       '-inputModelPath {model} -outputFolder {out_dir}/ '
                       '-textureResolution {resolution} -bundleResizedTo {resize} {verbose} '
                       '-textureWithSize {size} -logFile {log}'.format(**kwargs))
        else:
            log.ODM_WARNING('Found a valid ODM Texture file in: %s'
                            % tree.odm_textured_model_obj)

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'Texturing')

        log.ODM_INFO('Running ODM Texturing Cell - Finished')
        return ecto.OK if args.end_with != 'odm_texturing' else ecto.QUIT
