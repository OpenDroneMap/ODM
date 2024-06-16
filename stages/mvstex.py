import os, shutil

from opendm import log
from opendm import io
from opendm import system
from opendm import context
from opendm import types
from opendm.multispectral import get_primary_band_name
from opendm.photo import find_largest_photo_dim
from opendm.objpacker import obj_pack
from opendm.gltf import obj2glb

class ODMMvsTexStage(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']

        max_dim = find_largest_photo_dim(reconstruction.photos)
        max_texture_size = 8 * 1024 # default

        if max_dim > 8000:
            log.ODM_INFO("Large input images (%s pixels), increasing maximum texture size." % max_dim)
            max_texture_size *= 3

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
                    'primary': primary,
                    'nvm_file': nvm_file,
                    'labeling_file': os.path.join(tree.odm_texturing, "odm_textured_model_geo_labeling.vec") if subdir else None
                }]

            if not args.use_3dmesh:
                nonloc.runs += [{
                    'out_dir': os.path.join(tree.odm_25dtexturing, subdir),
                    'model': tree.odm_25dmesh,
                    'nadir': True,
                    'primary': primary,
                    'nvm_file': nvm_file,
                    'labeling_file': os.path.join(tree.odm_25dtexturing, "odm_textured_model_geo_labeling.vec") if subdir else None
                }]

        if reconstruction.multi_camera:

            for band in reconstruction.multi_camera:
                primary = band['name'] == get_primary_band_name(reconstruction.multi_camera, args.primary_band)
                nvm_file = os.path.join(tree.opensfm, "undistorted", "reconstruction_%s.nvm" % band['name'].lower())
                add_run(nvm_file, primary, band['name'].lower())
            
            # Sort to make sure primary band is processed first
            nonloc.runs.sort(key=lambda r: r['primary'], reverse=True)
        else:
            add_run(tree.opensfm_reconstruction_nvm)
        
        progress_per_run = 100.0 / len(nonloc.runs)
        progress = 0.0

        for r in nonloc.runs:
            if not io.dir_exists(r['out_dir']):
                system.mkdir_p(r['out_dir'])

            odm_textured_model_obj = os.path.join(r['out_dir'], tree.odm_textured_model_obj)
            unaligned_obj = io.related_file_path(odm_textured_model_obj, postfix="_unaligned")

            if not io.file_exists(odm_textured_model_obj) or self.rerun():
                log.ODM_INFO('Writing MVS Textured file in: %s'
                              % odm_textured_model_obj)

                if os.path.isfile(unaligned_obj):
                    os.unlink(unaligned_obj)

                # Format arguments to fit Mvs-Texturing app
                skipGlobalSeamLeveling = ""
                keepUnseenFaces = ""
                nadir = ""

                if args.texturing_skip_global_seam_leveling:
                    skipGlobalSeamLeveling = "--skip_global_seam_leveling"
                if args.texturing_keep_unseen_faces:
                    keepUnseenFaces = "--keep_unseen_faces"
                if (r['nadir']):
                    nadir = '--nadir_mode'

                # mvstex definitions
                kwargs = {
                    'bin': context.mvstex_path,
                    'out_dir': os.path.join(r['out_dir'], "odm_textured_model_geo"),
                    'model': r['model'],
                    'dataTerm': 'gmi',
                    'outlierRemovalType': 'gauss_clamping',
                    'skipGlobalSeamLeveling': skipGlobalSeamLeveling,
                    'keepUnseenFaces': keepUnseenFaces,
                    'toneMapping': 'none',
                    'nadirMode': nadir,
                    'maxTextureSize': '--max_texture_size=%s' % max_texture_size,
                    'nvm_file': r['nvm_file'],
                    'intermediate': '--no_intermediate_results' if (r['labeling_file'] or not reconstruction.multi_camera) else '',
                    'labelingFile': '-L "%s"' % r['labeling_file'] if r['labeling_file'] else ''
                }

                mvs_tmp_dir = os.path.join(r['out_dir'], 'tmp')

                # mvstex creates a tmp directory, so make sure it is empty
                if io.dir_exists(mvs_tmp_dir):
                    log.ODM_INFO("Removing old tmp directory {}".format(mvs_tmp_dir))
                    shutil.rmtree(mvs_tmp_dir)

                # run texturing binary
                system.run('"{bin}" "{nvm_file}" "{model}" "{out_dir}" '
                        '-d {dataTerm} -o {outlierRemovalType} '
                        '-t {toneMapping} '
                        '{intermediate} '
                        '{skipGlobalSeamLeveling} '
                        '{keepUnseenFaces} '
                        '{nadirMode} '
                        '{labelingFile} '
                        '{maxTextureSize} '.format(**kwargs))

                if r['primary'] and (not r['nadir'] or args.skip_3dmodel):
                    # GlTF?
                    if args.gltf:
                        log.ODM_INFO("Generating glTF Binary")
                        odm_textured_model_glb = os.path.join(r['out_dir'], tree.odm_textured_model_glb)
            
                        try:
                            obj2glb(odm_textured_model_obj, odm_textured_model_glb, rtc=reconstruction.get_proj_offset(), _info=log.ODM_INFO)
                        except Exception as e:
                            log.ODM_WARNING(str(e))

                    # Single material?
                    if args.texturing_single_material:
                        log.ODM_INFO("Packing to single material")

                        packed_dir = os.path.join(r['out_dir'], 'packed')
                        if io.dir_exists(packed_dir):
                            log.ODM_INFO("Removing old packed directory {}".format(packed_dir))
                            shutil.rmtree(packed_dir)
                        
                        try:
                            obj_pack(os.path.join(r['out_dir'], tree.odm_textured_model_obj), packed_dir, _info=log.ODM_INFO)
                            
                            # Move packed/* into texturing folder
                            system.delete_files(r['out_dir'], (".vec", ))
                            system.move_files(packed_dir, r['out_dir'])
                            if os.path.isdir(packed_dir):
                                os.rmdir(packed_dir)
                        except Exception as e:
                            log.ODM_WARNING(str(e))


                # Backward compatibility: copy odm_textured_model_geo.mtl to odm_textured_model.mtl
                # for certain older WebODM clients which expect a odm_textured_model.mtl
                # to be present for visualization
                # We should remove this at some point in the future
                geo_mtl = os.path.join(r['out_dir'], 'odm_textured_model_geo.mtl')
                if io.file_exists(geo_mtl):
                    nongeo_mtl = os.path.join(r['out_dir'], 'odm_textured_model.mtl')
                    shutil.copy(geo_mtl, nongeo_mtl)

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

