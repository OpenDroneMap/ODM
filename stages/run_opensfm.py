import sys
import os
import shutil
import glob

from opendm import log
from opendm import io
from opendm import system
from opendm import context
from opendm import gsd
from opendm import point_cloud
from opendm import types
from opendm.osfm import OSFMContext
from opendm import multispectral

class ODMOpenSfMStage(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']
        photos = reconstruction.photos

        if not photos:
            log.ODM_ERROR('Not enough photos in photos array to start OpenSfM')
            exit(1)

        octx = OSFMContext(tree.opensfm)
        octx.setup(args, tree.dataset_raw, photos, reconstruction=reconstruction, rerun=self.rerun())
        octx.extract_metadata(self.rerun())
        self.update_progress(20)
        octx.feature_matching(self.rerun())
        self.update_progress(30)
        octx.reconstruct(self.rerun())
        octx.extract_cameras(tree.path("cameras.json"), self.rerun())
        self.update_progress(70)

        if args.optimize_disk_space:
            shutil.rmtree(octx.path("features"))
            shutil.rmtree(octx.path("matches"))
            shutil.rmtree(octx.path("exif"))
            shutil.rmtree(octx.path("reports"))

        # If we find a special flag file for split/merge we stop right here
        if os.path.exists(octx.path("split_merge_stop_at_reconstruction.txt")):
            log.ODM_INFO("Stopping OpenSfM early because we found: %s" % octx.path("split_merge_stop_at_reconstruction.txt"))
            self.next_stage = None
            return

        if args.fast_orthophoto:
            output_file = octx.path('reconstruction.ply')
        elif args.use_opensfm_dense:
            output_file = tree.opensfm_model
        else:
            output_file = tree.opensfm_reconstruction

        updated_config_flag_file = octx.path('updated_config.txt')

        # Make sure it's capped by the depthmap-resolution arg,
        # since the undistorted images are used for MVS
        outputs['undist_image_max_size'] = max(
            gsd.image_max_size(photos, args.orthophoto_resolution, tree.opensfm_reconstruction, ignore_gsd=args.ignore_gsd, has_gcp=reconstruction.has_gcp()),
            args.depthmap_resolution
        )

        if not io.file_exists(updated_config_flag_file) or self.rerun():
            octx.update_config({'undistorted_image_max_size': outputs['undist_image_max_size']})
            octx.touch(updated_config_flag_file)

        # These will be used for texturing / MVS
        if args.radiometric_calibration == "none":
            octx.convert_and_undistort(self.rerun())
        else:
            def radiometric_calibrate(shot_id, image):
                photo = reconstruction.get_photo(shot_id)
                return multispectral.dn_to_reflectance(photo, image, use_sun_sensor=args.radiometric_calibration=="camera+sun")

            octx.convert_and_undistort(self.rerun(), radiometric_calibrate)

        self.update_progress(80)

        if reconstruction.multi_camera:
            # Dump band image lists
            log.ODM_INFO("Multiple bands found")
            for band in reconstruction.multi_camera:
                log.ODM_INFO("Exporting %s band" % band['name'])
                image_list_file = octx.path("image_list_%s.txt" % band['name'].lower())

                if not io.file_exists(image_list_file) or self.rerun():
                    with open(image_list_file, "w") as f:
                        f.write("\n".join([p.filename for p in band['photos']]))
                        log.ODM_INFO("Wrote %s" % image_list_file)
                else:
                    log.ODM_WARNING("Found a valid image list in %s for %s band" % (image_list_file, band['name']))
                
                nvm_file = octx.path("undistorted", "reconstruction_%s.nvm" % band['name'].lower())
                if not io.file_exists(nvm_file) or self.rerun():
                    octx.run('export_visualsfm --points --image_list "%s"' % image_list_file)
                    os.rename(tree.opensfm_reconstruction_nvm, nvm_file)
                else:
                    log.ODM_WARNING("Found a valid NVM file in %s for %s band" % (nvm_file, band['name']))

        if not io.file_exists(tree.opensfm_reconstruction_nvm) or self.rerun():
            octx.run('export_visualsfm --points')
        else:
            log.ODM_WARNING('Found a valid OpenSfM NVM reconstruction file in: %s' %
                            tree.opensfm_reconstruction_nvm)

        self.update_progress(85)

        # Skip dense reconstruction if necessary and export
        # sparse reconstruction instead
        if args.fast_orthophoto:
            if not io.file_exists(output_file) or self.rerun():
                octx.run('export_ply --no-cameras')
            else:
                log.ODM_WARNING("Found a valid PLY reconstruction in %s" % output_file)

        elif args.use_opensfm_dense:
            if not io.file_exists(output_file) or self.rerun():
                octx.run('compute_depthmaps')
            else:
                log.ODM_WARNING("Found a valid dense reconstruction in %s" % output_file)

        self.update_progress(90)

        if reconstruction.is_georeferenced() and (not io.file_exists(tree.opensfm_transformation) or self.rerun()):
            octx.run('export_geocoords --transformation --proj \'%s\'' % reconstruction.georef.proj4())
        else:
            log.ODM_WARNING("Will skip exporting %s" % tree.opensfm_transformation)
        
        if args.optimize_disk_space:
            os.remove(octx.path("tracks.csv"))
            os.remove(octx.path("undistorted", "tracks.csv"))
            os.remove(octx.path("undistorted", "reconstruction.json"))
            if io.dir_exists(octx.path("undistorted", "depthmaps")):
                files = glob.glob(octx.path("undistorted", "depthmaps", "*.npz"))
                for f in files:
                    os.remove(f)