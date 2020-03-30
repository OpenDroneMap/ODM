import os, shutil

from opendm import log
from opendm import io
from opendm import system
from opendm import context
from opendm import types

class ODMMvsTexStage(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']

        class nonloc:
            runs = []

        def add_run(nvm_file, primary=True, band=None):
            subdir = ""
            if not primary and band is not None:
                subdir = band

            if not args.skip_3dmodel and (primary or args.use_3dmesh):
                nonloc.runs += [{
                    'out_dir': os.path.join(tree.odm_texturing, subdir),
                    'model': tree.odm_mesh,
                    'nadir': False,
                    'nvm_file': nvm_file
                }]

            if not args.use_3dmesh:
                nonloc.runs += [{
                    'out_dir': os.path.join(tree.odm_25dtexturing, subdir),
                    'model': tree.odm_25dmesh,
                    'nadir': True,
                    'nvm_file': nvm_file
                }]

        if reconstruction.multi_camera:
            for band in reconstruction.multi_camera:
                primary = band == reconstruction.multi_camera[0]
                nvm_file = os.path.join(tree.opensfm, "undistorted", "reconstruction_%s.nvm" % band['name'].lower())
                add_run(nvm_file, primary, band['name'].lower())
        else:
            add_run(tree.opensfm_reconstruction_nvm)

        progress_per_run = 100.0 / len(nonloc.runs)
        progress = 0.0

        for r in nonloc.runs:
            if not io.dir_exists(r['out_dir']):
                system.mkdir_p(r['out_dir'])

            odm_textured_model_obj = os.path.join(r['out_dir'], tree.odm_textured_model_obj)

            if not io.file_exists(odm_textured_model_obj) or self.rerun():
                log.ODM_INFO('Writing MVS Textured file in: %s'
                              % odm_textured_model_obj)

                # Format arguments to fit Mvs-Texturing app
                skipGeometricVisibilityTest = ""
                skipGlobalSeamLeveling = ""
                skipLocalSeamLeveling = ""
                skipHoleFilling = ""
                keepUnseenFaces = ""
                nadir = ""

                if (self.params.get('skip_vis_test')):
                    skipGeometricVisibilityTest = "--skip_geometric_visibility_test"
                if (self.params.get('skip_glob_seam_leveling')):
                    skipGlobalSeamLeveling = "--skip_global_seam_leveling"
                if (self.params.get('skip_loc_seam_leveling')):
                    skipLocalSeamLeveling = "--skip_local_seam_leveling"
                if (self.params.get('skip_hole_fill')):
                    skipHoleFilling = "--skip_hole_filling"
                if (self.params.get('keep_unseen_faces')):
                    keepUnseenFaces = "--keep_unseen_faces"
                if (r['nadir']):
                    nadir = '--nadir_mode'

                # mvstex definitions
                kwargs = {
                    'bin': context.mvstex_path,
                    'out_dir': io.join_paths(r['out_dir'], "odm_textured_model"),
                    'model': r['model'],
                    'dataTerm': self.params.get('data_term'),
                    'outlierRemovalType': self.params.get('outlier_rem_type'),
                    'skipGeometricVisibilityTest': skipGeometricVisibilityTest,
                    'skipGlobalSeamLeveling': skipGlobalSeamLeveling,
                    'skipLocalSeamLeveling': skipLocalSeamLeveling,
                    'skipHoleFilling': skipHoleFilling,
                    'keepUnseenFaces': keepUnseenFaces,
                    'toneMapping': self.params.get('tone_mapping'),
                    'nadirMode': nadir,
                    'nadirWeight': 2 ** args.texturing_nadir_weight - 1,
                    'nvm_file': r['nvm_file']
                }

                mvs_tmp_dir = os.path.join(r['out_dir'], 'tmp')

                # Make sure tmp directory is empty
                if io.dir_exists(mvs_tmp_dir):
                    log.ODM_INFO("Removing old tmp directory {}".format(mvs_tmp_dir))
                    shutil.rmtree(mvs_tmp_dir)

                # run texturing binary
                system.run('{bin} {nvm_file} {model} {out_dir} '
                        '-d {dataTerm} -o {outlierRemovalType} '
                        '-t {toneMapping} '
                        '{skipGeometricVisibilityTest} '
                        '{skipGlobalSeamLeveling} '
                        '{skipLocalSeamLeveling} '
                        '{skipHoleFilling} '
                        '{keepUnseenFaces} '
                        '{nadirMode} '
                        '-n {nadirWeight}'.format(**kwargs))
                
                if args.optimize_disk_space:
                    cleanup_files = [
                        os.path.join(r['out_dir'], "odm_textured_model_data_costs.spt"),
                        os.path.join(r['out_dir'], "odm_textured_model_labeling.vec"),
                    ]
                    for f in cleanup_files:
                        if io.file_exists(f):
                            os.remove(f)
                
                progress += progress_per_run
                self.update_progress(progress)
            else:
                log.ODM_WARNING('Found a valid ODM Texture file in: %s'
                                % odm_textured_model_obj)
        
        if args.optimize_disk_space:
            for r in nonloc.runs:
                if io.file_exists(r['model']):
                    os.remove(r['model'])
            
            undistorted_images_path = os.path.join(tree.opensfm, "undistorted", "images")
            if io.dir_exists(undistorted_images_path):
                shutil.rmtree(undistorted_images_path)

