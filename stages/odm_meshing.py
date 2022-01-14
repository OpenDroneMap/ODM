import os, math

from opendm import log
from opendm import io
from opendm import system
from opendm import context
from opendm import mesh
from opendm import gsd
from opendm import types

class ODMeshingStage(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']

        # define paths and create working directories
        system.mkdir_p(tree.odm_meshing)

        # Create full 3D model unless --skip-3dmodel is set
        if not args.skip_3dmodel:
          if not io.file_exists(tree.odm_mesh) or self.rerun():
              log.ODM_INFO('Writing ODM Mesh file in: %s' % tree.odm_mesh)

              mesh.screened_poisson_reconstruction(tree.filtered_point_cloud,
                tree.odm_mesh,
                depth=self.params.get('oct_tree'),
                samples=self.params.get('samples'),
                maxVertexCount=self.params.get('max_vertex'),
                pointWeight=self.params.get('point_weight'),
                threads=max(1, self.params.get('max_concurrency') - 1), # poissonrecon can get stuck on some machines if --threads == all cores
                verbose=self.params.get('verbose'))

          else:
              log.ODM_WARNING('Found a valid ODM Mesh file in: %s' %
                              tree.odm_mesh)
        
        self.update_progress(50)

        # Always generate a 2.5D mesh
        # unless --use-3dmesh is set.
        if not args.use_3dmesh:
          if not io.file_exists(tree.odm_25dmesh) or self.rerun():

              log.ODM_INFO('Writing ODM 2.5D Mesh file in: %s' % tree.odm_25dmesh)
              ortho_resolution = gsd.cap_resolution(args.orthophoto_resolution, tree.opensfm_reconstruction, 
                                                    ignore_gsd=args.ignore_gsd,
                                                    ignore_resolution=(not reconstruction.is_georeferenced()) and args.ignore_gsd,
                                                    has_gcp=reconstruction.has_gcp()) / 100.0 
              
              dsm_multiplier = max(1.0, gsd.rounded_gsd(tree.opensfm_reconstruction, default_value=4, ndigits=3, ignore_gsd=args.ignore_gsd))
              
              # A good DSM size depends on the flight altitude.
              # Flights at low altitude need more details (higher resolution) 
              # Flights at higher altitude benefit from smoother surfaces (lower resolution)
              dsm_resolution = ortho_resolution * dsm_multiplier
              
              dsm_radius = dsm_resolution * math.sqrt(2)

              if args.fast_orthophoto:
                  dsm_radius *= 2
                  dsm_resolution *= 8

              log.ODM_INFO('ODM 2.5D DSM resolution: %s' % dsm_resolution)
              
              mesh.create_25dmesh(tree.filtered_point_cloud, tree.odm_25dmesh,
                    dsm_radius=dsm_radius,
                    dsm_resolution=dsm_resolution, 
                    depth=self.params.get('oct_tree'),
                    maxVertexCount=self.params.get('max_vertex'),
                    samples=self.params.get('samples'),
                    verbose=self.params.get('verbose'),
                    available_cores=args.max_concurrency,
                    method='poisson' if args.fast_orthophoto else 'gridded',
                    smooth_dsm=True)
          else:
              log.ODM_WARNING('Found a valid ODM 2.5D Mesh file in: %s' %
                              tree.odm_25dmesh)

