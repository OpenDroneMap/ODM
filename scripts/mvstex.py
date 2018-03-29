import ecto, os

from opendm import log
from opendm import io
from opendm import system
from opendm import context

import pmvs2nvmcams

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
        if args.use_25dmesh: system.mkdir_p(tree.odm_25dtexturing) 

        # check if we rerun cell or not
        rerun_cell = (args.rerun is not None and
                      args.rerun == 'mvs_texturing') or \
                     (args.rerun_all) or \
                     (args.rerun_from is not None and
                      'mvs_texturing' in args.rerun_from)

        runs = [{
            'out_dir': tree.odm_texturing,
            'model': tree.odm_mesh,
            'force_skip_vis_test': False
        }]

        if args.fast_orthophoto:
            runs = []

        if args.use_25dmesh:
            runs += [{
                    'out_dir': tree.odm_25dtexturing,
                    'model': tree.odm_25dmesh,

                    # We always skip the visibility test when using the 2.5D mesh
                    # because many faces end up being narrow, and almost perpendicular 
                    # to the ground plane. The visibility test improperly classifies
                    # them as "not seen" since the test is done on a single triangle vertex,
                    # and while one vertex might be occluded, the other two might not.
                    'force_skip_vis_test': True 
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
                
                if (self.params.skip_vis_test or r['force_skip_vis_test']):
                    skipGeometricVisibilityTest = "--skip_geometric_visibility_test"
                if (self.params.skip_glob_seam_leveling):
                    skipGlobalSeamLeveling = "--skip_global_seam_leveling"
                if (self.params.skip_loc_seam_leveling):
                    skipLocalSeamLeveling = "--skip_local_seam_leveling"
                if (self.params.skip_hole_fill):
                    skipHoleFilling = "--skip_hole_filling"
                if (self.params.keep_unseen_faces):
                    keepUnseenFaces = "--keep_unseen_faces"

                # mvstex definitions
                kwargs = {
                    'bin': context.mvstex_path,
                    'out_dir': io.join_paths(r['out_dir'], "odm_textured_model"),
                    'pmvs_folder': tree.pmvs_rec_path,
                    'nvm_file': io.join_paths(tree.pmvs_rec_path, "nvmCams.nvm"),
                    'model': r['model'],
                    'dataTerm': self.params.data_term,
                    'outlierRemovalType': self.params.outlier_rem_type,
                    'skipGeometricVisibilityTest': skipGeometricVisibilityTest,
                    'skipGlobalSeamLeveling': skipGlobalSeamLeveling,
                    'skipLocalSeamLeveling': skipLocalSeamLeveling,
                    'skipHoleFilling': skipHoleFilling,
                    'keepUnseenFaces': keepUnseenFaces,
                    'toneMapping': self.params.tone_mapping
                }

                if not args.use_pmvs:
                    kwargs['nvm_file'] = io.join_paths(tree.opensfm,
                                                       "reconstruction.nvm")
                else:
                    log.ODM_DEBUG('Generating .nvm file from pmvs output: %s'
                                  % '{nvm_file}'.format(**kwargs))

                    # Create .nvm camera file.
                    pmvs2nvmcams.run('{pmvs_folder}'.format(**kwargs),
                                     '{nvm_file}'.format(**kwargs))

                # run texturing binary
                system.run('{bin} {nvm_file} {model} {out_dir} '
                           '-d {dataTerm} -o {outlierRemovalType} '
                           '-t {toneMapping} '
                           '{skipGeometricVisibilityTest} '
                           '{skipGlobalSeamLeveling} '
                           '{skipLocalSeamLeveling} '
                           '{skipHoleFilling} '
                           '{keepUnseenFaces}'.format(**kwargs))
            else:
                log.ODM_WARNING('Found a valid ODM Texture file in: %s'
                                % odm_textured_model_obj)

        outputs.reconstruction = reconstruction

        if args.time:
            system.benchmark(start_time, tree.benchmarking, 'Texturing')

        log.ODM_INFO('Running ODM Texturing Cell - Finished')
        return ecto.OK if args.end_with != 'mvs_texturing' else ecto.QUIT
