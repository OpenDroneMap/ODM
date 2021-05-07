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
from opendm.point_cloud import fast_merge_ply

class ODMOpenMVSStage(types.ODM_Stage):
    def process(self, args, outputs):
        # get inputs
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']
        photos = reconstruction.photos
        octx = OSFMContext(tree.opensfm)

        if not photos:
            log.ODM_ERROR('Not enough photos in photos array to start OpenMVS')
            exit(1)

        # check if reconstruction was done before
        if not io.file_exists(tree.openmvs_model) or self.rerun():
            if self.rerun():
                if io.dir_exists(tree.openmvs):
                    shutil.rmtree(tree.openmvs)

            # export reconstruction from opensfm
            openmvs_scene_file = os.path.join(tree.openmvs, "scene.mvs")
            if not io.file_exists(openmvs_scene_file) or self.rerun():
                cmd = 'export_openmvs'
                octx.run(cmd)
            else:
                log.ODM_WARNING("Found existing %s" % openmvs_scene_file)
            
            self.update_progress(10)

            depthmaps_dir = os.path.join(tree.openmvs, "depthmaps")

            if io.dir_exists(depthmaps_dir) and self.rerun():
                shutil.rmtree(depthmaps_dir)

            if not io.dir_exists(depthmaps_dir):
                os.mkdir(depthmaps_dir)
            
            depthmap_resolution = get_depthmap_resolution(args, photos)
            if outputs["undist_image_max_size"] <= depthmap_resolution:
                resolution_level = 0
            else:
                resolution_level = int(round(math.log(outputs['undist_image_max_size'] / float(depthmap_resolution)) / math.log(2)))

            log.ODM_INFO("Running dense reconstruction. This might take a while.")
            
            log.ODM_INFO("Estimating depthmaps")

            densify_ini_file = os.path.join(tree.openmvs, 'config.ini')
            with open(densify_ini_file, 'w+') as f:
                f.write("Optimize = 0\n") # Disable depth-maps re-filtering
            
            config = [
                " --resolution-level %s" % int(resolution_level),
	            "--min-resolution %s" % depthmap_resolution,
                "--max-resolution %s" % int(outputs['undist_image_max_size']),
                "--max-threads %s" % args.max_concurrency,
                "--number-views-fuse 2",
                '-w "%s"' % depthmaps_dir, 
                "-v 0"
            ]

            if args.pc_tile:
                config.append("--fusion-mode 1")

            system.run('%s "%s" %s' % (context.omvs_densify_path, 
                                       openmvs_scene_file,
                                      ' '.join(config)))

            self.update_progress(85)
            files_to_remove = []
            scene_dense = os.path.join(tree.openmvs, 'scene_dense.mvs')

            if args.pc_tile:
                log.ODM_INFO("Computing sub-scenes")
                config = [
                    "--sub-scene-area 660000",
                    "--max-threads %s" % args.max_concurrency,
                    '-w "%s"' % depthmaps_dir, 
                    "-v 0",
                ]
                system.run('%s "%s" %s' % (context.omvs_densify_path, 
                                        openmvs_scene_file,
                                        ' '.join(config)))
                
                scene_files = glob.glob(os.path.join(tree.openmvs, "scene_[0-9][0-9][0-9][0-9].mvs"))
                if len(scene_files) == 0:
                    log.ODM_ERROR("No OpenMVS scenes found. This could be a bug, or the reconstruction could not be processed.")
                    exit(1)

                log.ODM_INFO("Fusing depthmaps for %s scenes" % len(scene_files))
                
                scene_ply_files = []

                for sf in scene_files:
                    p, _ = os.path.splitext(sf)
                    scene_ply = p + "_dense_dense_filtered.ply"
                    scene_dense_mvs = p + "_dense.mvs"

                    files_to_remove += [scene_ply, sf, scene_dense_mvs]
                    scene_ply_files.append(scene_ply)

                    if not io.file_exists(scene_ply) or self.rerun():
                        # Fuse
                        config = [
                            '--resolution-level %s' % int(resolution_level),
                            '--min-resolution %s' % depthmap_resolution,
                            '--max-resolution %s' % int(outputs['undist_image_max_size']),
                            '--dense-config-file "%s"' % densify_ini_file,
                            '--number-views-fuse 2',
                            '--max-threads %s' % args.max_concurrency,
                            '-w "%s"' % depthmaps_dir,
                            '-v 0',
                        ]

                        try:
                            system.run('%s "%s" %s' % (context.omvs_densify_path, sf, ' '.join(config)))

                            # Filter
                            system.run('%s "%s" --filter-point-cloud -1 -v 0' % (context.omvs_densify_path, scene_dense_mvs))
                        except:
                            log.ODM_WARNING("Sub-scene %s could not be reconstructed, skipping..." % sf)

                        if not io.file_exists(scene_ply):
                            scene_ply_files.pop()
                            log.ODM_WARNING("Could not compute PLY for subscene %s" % sf)
                    else:
                        log.ODM_WARNING("Found existing dense scene file %s" % scene_ply)

                # Merge
                log.ODM_INFO("Merging %s scene files" % len(scene_ply_files))
                if len(scene_ply_files) == 0:
                    log.ODM_ERROR("Could not compute dense point cloud (no PLY files available).")
                if len(scene_ply_files) == 1:
                    # Simply rename
                    os.rename(scene_ply_files[0], tree.openmvs_model)
                    log.ODM_INFO("%s --> %s"% (scene_ply_files[0], tree.openmvs_model))
                else:
                    # Merge
                    fast_merge_ply(scene_ply_files, tree.openmvs_model)
            else:
                # Filter all at once
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

            # TODO: add support for image masks

            self.update_progress(95)

            if args.optimize_disk_space:
                files = [scene_dense,
                         os.path.join(tree.openmvs, 'scene_dense.ply'),
                         os.path.join(tree.openmvs, 'scene_dense_dense_filtered.mvs'),
                         octx.path("undistorted", "tracks.csv"),
                         octx.path("undistorted", "reconstruction.json")
                        ] + files_to_remove
                for f in files:
                    if os.path.exists(f):
                        os.remove(f)
                shutil.rmtree(depthmaps_dir)
        else:
            log.ODM_WARNING('Found a valid OpenMVS reconstruction file in: %s' %
                            tree.openmvs_model)
