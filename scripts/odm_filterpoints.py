import ecto, os

from opendm import log
from opendm import io
from opendm import system
from opendm import context
from opendm import point_cloud

class ODMFilterPoints(ecto.Cell):
    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("reconstruction", "ODMReconstruction", [])
        outputs.declare("reconstruction", "list of ODMReconstructions", [])

    def process(self, inputs, outputs):
        # Benchmarking
        start_time = system.now_raw()

        log.ODM_INFO('Running ODM FilterPoints Cell')

        # get inputs
        tree = inputs.tree
        args = inputs.args
        reconstruction = inputs.reconstruction

        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'odm_filterpoints') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'odm_filterpoints' in args.rerun_from)
        if not os.path.exists(tree.odm_filterpoints): system.mkdir_p(tree.odm_filterpoints)

        # check if reconstruction was done before
        if not io.file_exists(tree.filtered_point_cloud) or rerun_cell:
            if args.fast_orthophoto:
                inputPointCloud = os.path.join(tree.opensfm, 'reconstruction.ply')
            elif args.use_opensfm_dense:
                inputPointCloud = tree.opensfm_model
            else:
                inputPointCloud = tree.mve_model

            confidence = None
            if not args.use_opensfm_dense and not args.fast_orthophoto:
                confidence = args.mve_confidence

            point_cloud.filter(inputPointCloud, tree.filtered_point_cloud, standard_deviation=args.pc_filter, confidence=confidence, verbose=args.verbose)
        else:
            log.ODM_WARNING('Found a valid point cloud file in: %s' %
                            tree.filtered_point_cloud)

        outputs.reconstruction = reconstruction

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'MVE')

        log.ODM_INFO('Running ODM FilterPoints Cell - Finished')
        return ecto.OK if args.end_with != 'odm_filterpoints' else ecto.QUIT
