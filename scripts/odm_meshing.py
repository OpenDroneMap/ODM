import ecto

from opendm import log
from opendm import io
from opendm import system
from opendm import context


class ODMeshingCell(ecto.Cell):
    def declare_params(self, params):
        params.declare("max_vertex", 'The maximum vertex count of the output '
                                     'mesh', 100000)
        params.declare("oct_tree", 'Oct-tree depth used in the mesh reconstruction, '
                                   'increase to get more vertices, recommended '
                                   'values are 8-12', 9)
        params.declare("samples", 'Number of points per octree node, recommended '
                                  'value: 1.0', 1)
        params.declare("solver", 'Oct-tree depth at which the Laplacian equation '
                                 'is solved in the surface reconstruction step. '
                                 'Increasing this value increases computation '
                                 'times slightly but helps reduce memory usage.', 9)
        params.declare("verbose", 'print additional messages to console', False)

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("reconstruction", "Clusters output. list of ODMReconstructions", [])
        outputs.declare("reconstruction", "Clusters output. list of ODMReconstructions", [])

    def process(self, inputs, outputs):

        # Benchmarking
        start_time = system.now_raw()

        log.ODM_INFO('Running ODM Meshing Cell')

        # get inputs
        args = self.inputs.args
        tree = self.inputs.tree
        verbose = '-verbose' if self.params.verbose else ''

        # define paths and create working directories
        system.mkdir_p(tree.odm_meshing)

        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'odm_meshing') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'odm_meshing' in args.rerun_from)

        if not io.file_exists(tree.odm_mesh) or rerun_cell:
            log.ODM_DEBUG('Writing ODM Mesh file in: %s' % tree.odm_mesh)

            kwargs = {
                'bin': context.odm_modules_path,
                'outfile': tree.odm_mesh,
                'log': tree.odm_meshing_log,
                'max_vertex': self.params.max_vertex,
                'oct_tree': self.params.oct_tree,
                'samples': self.params.samples,
                'solver': self.params.solver,
                'verbose': verbose
            }
            if args.use_opensfm_pointcloud:
                kwargs['infile'] = tree.opensfm_model
            else:
                kwargs['infile'] = tree.pmvs_model

            # run meshing binary
            system.run('{bin}/odm_meshing -inputFile {infile} '
                       '-outputFile {outfile} -logFile {log} '
                       '-maxVertexCount {max_vertex} -octreeDepth {oct_tree} {verbose} '
                       '-samplesPerNode {samples} -solverDivide {solver}'.format(**kwargs))
        else:
            log.ODM_WARNING('Found a valid ODM Mesh file in: %s' %
                            tree.odm_mesh)

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'Meshing')

        log.ODM_INFO('Running ODM Meshing Cell - Finished')
        return ecto.OK if args.end_with != 'odm_meshing' else ecto.QUIT
