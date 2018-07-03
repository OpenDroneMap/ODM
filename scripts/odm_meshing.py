import ecto, os

from opendm import log
from opendm import io
from opendm import system
from opendm import context
from opendm import mesh


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
        args = inputs.args
        tree = inputs.tree
        reconstruction = inputs.reconstruction
        verbose = '-verbose' if self.params.verbose else ''

        # define paths and create working directories
        system.mkdir_p(tree.odm_meshing)

        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'odm_meshing') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'odm_meshing' in args.rerun_from)

        infile = tree.smvs_model
        if args.use_opensfm_dense:
          infile = tree.opensfm_model
        elif args.fast_orthophoto:
          infile = os.path.join(tree.opensfm, 'reconstruction.ply')

        # Create full 3D model unless --skip-3dmodel is set
        if not args.skip_3dmodel:
          if not io.file_exists(tree.odm_mesh) or rerun_cell:
              log.ODM_DEBUG('Writing ODM Mesh file in: %s' % tree.odm_mesh)

              mesh.screened_poisson_reconstruction(infile, 
                tree.odm_mesh,
                depth=self.params.oct_tree, 
                samples=self.params.samples, 
                maxVertexCount=self.params.max_vertex, 
                verbose=verbose)

          else:
              log.ODM_WARNING('Found a valid ODM Mesh file in: %s' %
                              tree.odm_mesh)

        # Always generate a 2.5D mesh
        # unless --use-3dmesh is set.
        if not args.use_3dmesh:
          if not io.file_exists(tree.odm_25dmesh) or rerun_cell:

              log.ODM_DEBUG('Writing ODM 2.5D Mesh file in: %s' % tree.odm_25dmesh)

              mesh.create_25dmesh(infile, tree.odm_25dmesh, 
                    dsm_resolution=float(1.0 / args.orthophoto_resolution), 
                    depth=self.params.oct_tree,
                    maxVertexCount=self.params.max_vertex,
                    verbose=self.params.verbose,
                    max_workers=args.max_concurrency)
          else:
              log.ODM_WARNING('Found a valid ODM 2.5D Mesh file in: %s' %
                              tree.odm_25dmesh)

        outputs.reconstruction = reconstruction

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'Meshing')

        log.ODM_INFO('Running ODM Meshing Cell - Finished')
        return ecto.OK if args.end_with != 'odm_meshing' else ecto.QUIT
