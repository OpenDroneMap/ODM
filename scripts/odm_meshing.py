import ecto, os, math

from opendm import log
from opendm import io
from opendm import system
from opendm import context
from opendm import mesh
from opendm import gsd


class ODMeshingCell(ecto.Cell):
    def declare_params(self, params):
        params.declare("max_vertex", 'The maximum vertex count of the output '
                                     'mesh', 100000)
        params.declare("oct_tree", 'Oct-tree depth used in the mesh reconstruction, '
                                   'increase to get more vertices, recommended '
                                   'values are 8-12', 9)
        params.declare("samples", 'Number of points per octree node, recommended '
                                  'value: 1.0', 1)
        params.declare("point_weight", "specifies the importance that interpolation of the point samples is given in the formulation of the screened Poisson equation.", 4)
        params.declare("max_concurrency", 'max threads', context.num_cores)
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

        # define paths and create working directories
        system.mkdir_p(tree.odm_meshing)

        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'odm_meshing') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'odm_meshing' in args.rerun_from)

        infile = tree.smvs_model
        if args.fast_orthophoto:
          infile = os.path.join(tree.opensfm, 'reconstruction.ply')
        elif args.use_opensfm_dense:
          infile = tree.opensfm_model

        # Create full 3D model unless --skip-3dmodel is set
        if not args.skip_3dmodel:
          if not io.file_exists(tree.odm_mesh) or rerun_cell:
              log.ODM_DEBUG('Writing ODM Mesh file in: %s' % tree.odm_mesh)

              mesh.screened_poisson_reconstruction(infile,
                tree.odm_mesh,
                depth=self.params.oct_tree,
                samples=self.params.samples,
                maxVertexCount=self.params.max_vertex,
                pointWeight=self.params.point_weight,
                threads=self.params.max_concurrency,
                verbose=self.params.verbose)

          else:
              log.ODM_WARNING('Found a valid ODM Mesh file in: %s' %
                              tree.odm_mesh)

        # Always generate a 2.5D mesh
        # unless --use-3dmesh is set.
        if not args.use_3dmesh:
          if not io.file_exists(tree.odm_25dmesh) or rerun_cell:

              log.ODM_DEBUG('Writing ODM 2.5D Mesh file in: %s' % tree.odm_25dmesh)
              ortho_resolution = gsd.cap_resolution(args.orthophoto_resolution, tree.opensfm_reconstruction, ignore_gsd=args.ignore_gsd) / 100.0 
              
              dsm_multiplier = max(1.0, gsd.rounded_gsd(tree.opensfm_reconstruction, default_value=4, ndigits=3, ignore_gsd=args.ignore_gsd))
              
              # A good DSM size depends on the flight altitude.
              # Flights at low altitude need more details (higher resolution) 
              # Flights at higher altitude benefit from smoother surfaces (lower resolution)
              dsm_resolution = ortho_resolution * dsm_multiplier
              
              dsm_radius = dsm_resolution * math.sqrt(2)

              # Sparse point clouds benefits from using
              # a larger radius interolation --> less holes
              if args.fast_orthophoto:
                  dsm_radius *= 2

              log.ODM_DEBUG('ODM 2.5D DSM resolution: %s' % dsm_resolution)
              
              mesh.create_25dmesh(infile, tree.odm_25dmesh,
                    dsm_radius=dsm_radius,
                    dsm_resolution=dsm_resolution, 
                    depth=self.params.oct_tree,
                    maxVertexCount=self.params.max_vertex,
                    samples=self.params.samples,
                    verbose=self.params.verbose,
                    available_cores=args.max_concurrency,
                    method='poisson' if args.fast_orthophoto else 'gridded')
          else:
              log.ODM_WARNING('Found a valid ODM 2.5D Mesh file in: %s' %
                              tree.odm_25dmesh)

        outputs.reconstruction = reconstruction

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'Meshing')

        log.ODM_INFO('Running ODM Meshing Cell - Finished')
        return ecto.OK if args.end_with != 'odm_meshing' else ecto.QUIT
