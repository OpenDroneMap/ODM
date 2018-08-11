import ecto, os, shutil

from opendm import log
from opendm import io
from opendm import system
from opendm import context

class ODMMvsTexCell(ecto.Cell):
    def declare_params(self, params):
        params.declare("data_term", 'Data term: [area, gmi] default: gmi', "gmi")
        params.declare("outlier_rem_type", 'Type of photometric outlier removal method: [none, gauss_damping, gauss_clamping]. default: none', "none")
        params.declare("skip_vis_test", 'Skip geometric visibility test based on ray intersection.', False)
        params.declare("skip_glob_seam_leveling", 'Skip global seam leveling.', False)
        params.declare("skip_loc_seam_leveling", 'Skip local seam leveling (Poisson editing).', False)
        params.declare("skip_hole_fill", 'Skip hole filling.', False)
        params.declare("keep_unseen_faces", 'Keep unseen faces.', False)
        params.declare("tone_mapping", 'Type of tone mapping: [none, gamma]. Default: gamma', "gamma")

    def declare_io(self, params, inputs, outputs):
        inputs.declare("tree", "Struct with paths", [])
        inputs.declare("args", "The application arguments.", {})
        inputs.declare("reconstruction", "Clusters output. list of ODMReconstructions", [])
        outputs.declare("reconstruction", "Clusters output. list of ODMReconstructions", [])

    def process(self, inputs, outputs):

        # Benchmarking
        start_time = system.now_raw()

        log.ODM_INFO('Running MVS Texturing Cell')

        # get inputs
        args = inputs.args
        tree = inputs.tree
        reconstruction = inputs.reconstruction

        # define paths and create working directories
        system.mkdir_p(tree.odm_texturing)
        if not args.use_3dmesh: system.mkdir_p(tree.odm_25dtexturing)

        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'mvs_texturing') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'mvs_texturing' in args.rerun_from)

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

            if not io.file_exists(odm_textured_model_obj) or rerun_cell:
                log.ODM_DEBUG('Writing MVS Textured file in: %s'
                              % odm_textured_model_obj)

                # Format arguments to fit Mvs-Texturing app
                skipGeometricVisibilityTest = ""
                skipGlobalSeamLeveling = ""
                skipLocalSeamLeveling = ""
                skipHoleFilling = ""
                keepUnseenFaces = ""
                nadir = ""

                if (self.params.skip_vis_test):
                    skipGeometricVisibilityTest = "--skip_geometric_visibility_test"
                if (self.params.skip_glob_seam_leveling):
                    skipGlobalSeamLeveling = "--skip_global_seam_leveling"
                if (self.params.skip_loc_seam_leveling):
                    skipLocalSeamLeveling = "--skip_local_seam_leveling"
                if (self.params.skip_hole_fill):
                    skipHoleFilling = "--skip_hole_filling"
                if (self.params.keep_unseen_faces):
                    keepUnseenFaces = "--keep_unseen_faces"
                if (r['nadir']):
                    nadir = '--nadir_mode'

                # mvstex definitions
                kwargs = {
                    'bin': context.mvstex_path,
                    'out_dir': io.join_paths(r['out_dir'], "odm_textured_model"),
                    'model': r['model'],
                    'dataTerm': self.params.data_term,
                    'outlierRemovalType': self.params.outlier_rem_type,
                    'skipGeometricVisibilityTest': skipGeometricVisibilityTest,
                    'skipGlobalSeamLeveling': skipGlobalSeamLeveling,
                    'skipLocalSeamLeveling': skipLocalSeamLeveling,
                    'skipHoleFilling': skipHoleFilling,
                    'keepUnseenFaces': keepUnseenFaces,
                    'toneMapping': self.params.tone_mapping,
                    'nadirMode': nadir,
                    'nadirWeight': 2 ** args.texturing_nadir_weight - 1,
                    'nvm_file': io.join_paths(tree.opensfm, "reconstruction.nvm")
                }

                # Make sure tmp directory is empty
                mvs_tmp_dir = os.path.join(r['out_dir'], 'tmp')
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
            else:
                log.ODM_WARNING('Found a valid ODM Texture file in: %s'
                                % odm_textured_model_obj)

        outputs.reconstruction = reconstruction

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'Texturing')

        log.ODM_INFO('Running ODM Texturing Cell - Finished')
        return ecto.OK if args.end_with != 'mvs_texturing' else ecto.QUIT
