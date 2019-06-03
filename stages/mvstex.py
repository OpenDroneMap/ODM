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

        # define paths and create working directories
        system.mkdir_p(tree.odm_texturing)
        if not args.use_3dmesh: system.mkdir_p(tree.odm_25dtexturing)

        runs = [{
            'out_dir': tree.odm_texturing,
            'model': tree.odm_mesh,
            'nadir': False
        }]

        if args.skip_3dmodel:
            runs = []

        if not args.use_3dmesh:
            runs += [{
                    'out_dir': tree.odm_25dtexturing,
                    'model': tree.odm_25dmesh,
                    'nadir': True
                }]

        for r in runs:
            odm_textured_model_obj = os.path.join(r['out_dir'], tree.odm_textured_model_obj)

            if not io.file_exists(odm_textured_model_obj) or self.rerun():
                log.ODM_DEBUG('Writing MVS Textured file in: %s'
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
                    'nvm_file': io.join_paths(tree.opensfm, "reconstruction.nvm")
                }

                mvs_tmp_dir = os.path.join(r['out_dir'], 'tmp')

                # TODO: find out why mvs-texturing is rarely crashing at random
                # Temporary workaround is to retry the command until we get it right
                # (up to a certain number of retries).
                retry_count = 1
                while retry_count < 5:
                    try:
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
                        break
                    except Exception as e:
                        if str(e) == "Child returned 134":
                            retry_count += 1
                            log.ODM_WARNING("Caught error code, retrying attempt #%s" % retry_count)
                        else:
                            raise e
                
                self.update_progress(50)
            else:
                log.ODM_WARNING('Found a valid ODM Texture file in: %s'
                                % odm_textured_model_obj)

