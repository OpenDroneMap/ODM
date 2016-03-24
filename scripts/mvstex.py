import ecto

from opendm import log
from opendm import io
from opendm import system
from opendm import context

import pmvs2nvmcams

class ODMMvsTexCell(ecto.Cell):
    def declare_params(self, params):
        params.declare("data_term", 'Data term: [area, gmi] default: gmi', "gmi")
        params.declare("outlier_rem_type", 'Type of photometric outlier removal method: [none, gauss_damping, gauss_clamping]. default: none', "none")
        params.declare("skip_vis_test", 'Skip geometric visibility test based on ray intersection.', "false")
        params.declare("skip_glob_seam_leveling", 'Skip global seam leveling.', "false")
        params.declare("skip_loc_seam_leveling", 'Skip local seam leveling (Poisson editing).', "false")
        params.declare("skip_hole_fill", 'Skip hole filling.', "false")
        params.declare("keep_unseen_faces", 'Keep unseen faces.', "false")

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
        args = self.inputs.args
        tree = self.inputs.tree

        # define paths and create working directories
        system.mkdir_p(tree.odm_texturing)

        # check if we rerun cell or not
        rerun_cell = (args['rerun'] is not None and
                      args['rerun'] == 'mvs_texturing') or \
                     (args['rerun_all']) or \
                     (args['rerun_from'] is not None and
                      'mvs_texturing' in args['rerun_from'])

        if not io.file_exists(tree.odm_textured_model_obj) or rerun_cell:
            log.ODM_DEBUG('Writing MVS Textured file in: %s'
                          % tree.odm_textured_model_obj)
            
            
            # Format arguments to fit Mvs-Texturing app
            skipGeometricVisibilityTest = ""
            skipGlobalSeamLeveling = ""
            skipLocalSeamLeveling = ""
            skipHoleFilling = ""
            keepUnseenFaces = ""
            
            if (self.params.skip_vis_test.lower() == "true"):
                skipGeometricVisibilityTest = "--skip_geometric_visibility_test"
            if (self.params.skip_glob_seam_leveling.lower() == "true"):
                skipGlobalSeamLeveling = "--skip_global_seam_leveling"
            if (self.params.skip_loc_seam_leveling.lower() == "true"):
                skipLocalSeamLeveling = "--skip_local_seam_leveling"
            if (self.params.skip_hole_fill.lower() == "true"):
                skipHoleFilling = "--skip_hole_filling"
            if (self.params.keep_unseen_faces.lower() == "true"):
                keepUnseenFaces = "--keep_unseen_faces"
            
            # mvstex definitions
            kwargs = {
                'bin': context.mvstex_path,
                'out_dir': io.join_paths(tree.odm_texturing, "odm_textured_model"),
                'pmvs_folder': tree.pmvs_rec_path,
                'nvm_file': io.join_paths(tree.pmvs_rec_path, "nvmCams.nvm"),
                'model': tree.odm_mesh,
                'dataTerm': self.params.data_term,
                'outlierRemovalType': self.params.outlier_rem_type,
                'skipGeometricVisibilityTest': skipGeometricVisibilityTest,
                'skipGlobalSeamLeveling': skipGlobalSeamLeveling,
                'skipLocalSeamLeveling': skipLocalSeamLeveling,
                'skipHoleFilling': skipHoleFilling,
                'keepUnseenFaces': keepUnseenFaces
            }
            
            log.ODM_DEBUG('Generating .nvm file from pmvs output: %s'
                          % '{nvm_file}'.format(**kwargs))
                          
            # Create .nvm camera file.
            pmvs2nvmcams.run('{pmvs_folder}'.format(**kwargs), 
                             '{nvm_file}'.format(**kwargs))
            
            # run texturing binary
            system.run('{bin} {nvm_file} {model} {out_dir} '
                       '-d {dataTerm} -o {outlierRemovalType} '
                       '{skipGeometricVisibilityTest} '
                       '{skipGlobalSeamLeveling} '
                       '{skipLocalSeamLeveling} '
                       '{skipHoleFilling} '
                       '{keepUnseenFaces}'.format(**kwargs))
        else:
            log.ODM_WARNING('Found a valid ODM Texture file in: %s'
                            % tree.odm_textured_model_obj)

        if args['time']:
            system.benchmark(start_time, tree.benchmarking, 'Texturing')

        log.ODM_INFO('Running OMD Texturing Cell - Finished')
        return ecto.OK if args['end_with'] != 'odm_texturing' else ecto.QUIT
