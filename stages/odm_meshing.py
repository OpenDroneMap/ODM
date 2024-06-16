import os, math

from opendm import log
from opendm import io
from opendm import system
from opendm import context
from opendm import mesh
from opendm import gsd
from opendm import types
from opendm.dem import commands

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
                threads=max(1, self.params.get('max_concurrency') - 1)), # poissonrecon can get stuck on some machines if --threads == all cores
          else:
              log.ODM_WARNING('Found a valid ODM Mesh file in: %s' %
                              tree.odm_mesh)
        
        self.update_progress(50)

        # Always generate a 2.5D mesh
        # unless --use-3dmesh is set.
        if not args.use_3dmesh:
          if not io.file_exists(tree.odm_25dmesh) or self.rerun():

              log.ODM_INFO('Writing ODM 2.5D Mesh file in: %s' % tree.odm_25dmesh)

              multiplier = math.pi / 2.0
              radius_steps = commands.get_dem_radius_steps(tree.filtered_point_cloud_stats, 3, args.orthophoto_resolution, multiplier=multiplier)
              dsm_resolution = radius_steps[0] / multiplier

              log.ODM_INFO('ODM 2.5D DSM resolution: %s' % dsm_resolution)
              
              if args.fast_orthophoto:
                  dsm_resolution *= 8.0

              mesh.create_25dmesh(tree.filtered_point_cloud, tree.odm_25dmesh,
                    radius_steps,
                    dsm_resolution=dsm_resolution, 
                    depth=self.params.get('oct_tree'),
                    maxVertexCount=self.params.get('max_vertex'),
                    samples=self.params.get('samples'),
                    available_cores=args.max_concurrency,
                    method='poisson' if args.fast_orthophoto else 'gridded',
                    smooth_dsm=True,
                    max_tiles=None if reconstruction.has_geotagged_photos() else math.ceil(len(reconstruction.photos) / 2))
          else:
              log.ODM_WARNING('Found a valid ODM 2.5D Mesh file in: %s' %
                              tree.odm_25dmesh)

