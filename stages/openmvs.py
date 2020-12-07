import shutil, os, glob, math

from opendm import log
from opendm import io
from opendm import system
from opendm import context
from opendm import point_cloud
from opendm import types
from opendm.utils import get_depthmap_resolution
from opendm.osfm import OSFMContext
from opendm.multispectral import get_primary_band_name

class ODMOpenMVSStage(types.ODM_Stage):
    def process(self, args, outputs):
        # get inputs
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']
        photos = reconstruction.photos

        if not photos:
            log.ODM_ERROR('Not enough photos in photos array to start OpenMVS')
            exit(1)

        # check if reconstruction was done before
        if not io.file_exists(tree.openmvs_model) or self.rerun():
            if io.dir_exists(tree.openmvs):
                shutil.rmtree(tree.openmvs)

            # export reconstruction from opensfm
            octx = OSFMContext(tree.opensfm)
            cmd = 'export_openmvs'
            octx.run(cmd)
            
            self.update_progress(10)

            depthmaps_dir = os.path.join(tree.openmvs, "depthmaps")
            if not io.dir_exists(depthmaps_dir):
                os.mkdir(depthmaps_dir)
            
            depthmap_resolution = get_depthmap_resolution(args, photos)

            if outputs["undist_image_max_size"] <= depthmap_resolution:
                resolution_level = 0
            else:
                resolution_level = math.floor(math.log(outputs['undist_image_max_size'] / float(depthmap_resolution)) / math.log(2))

            config = [
                " --resolution-level %s" % int(resolution_level),
	            "--min-resolution %s" % depthmap_resolution,
                "--max-resolution %s" % int(outputs['undist_image_max_size']),
                "--max-threads %s" % args.max_concurrency,
                '-w "%s"' % depthmaps_dir, 
                "-v 0",
            ]

            log.ODM_INFO("Running dense reconstruction. This might take a while.")
            
            system.run('%s "%s" %s' % (context.omvs_densify_path, 
                                       os.path.join(tree.openmvs, 'scene.mvs'),
                                      ' '.join(config)))

            self.update_progress(85)

            # Filter points
            scene_dense = os.path.join(tree.openmvs, 'scene_dense.mvs')
            if os.path.exists(scene_dense):
                config = [
                    "--filter-point-cloud -1",
                    '-i "%s"' % scene_dense,
                    "-v 0"
                ]
                system.run('%s %s' % (context.omvs_densify_path, ' '.join(config)))
            else:
                log.ODM_WARNING("Cannot find scene_dense.mvs, dense reconstruction probably failed. Exiting...")
                exit(1)

            self.update_progress(95)

            if args.optimize_disk_space:
                files = [scene_dense,
                         os.path.join(tree.openmvs, 'scene_dense.ply'),
                         os.path.join(tree.openmvs, 'scene_dense_dense_filtered.mvs'),
                         octx.path("undistorted", "tracks.csv"),
                         octx.path("undistorted", "reconstruction.json")
                        ]
                for f in files:
                    if os.path.exists(f):
                        os.remove(f)
                shutil.rmtree(depthmaps_dir)
        else:
            log.ODM_WARNING('Found a valid OpenMVS reconstruction file in: %s' %
                            tree.openmvs_model)
