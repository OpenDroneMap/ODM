import os
import shutil
from opendm import log
from opendm.osfm import OSFMContext, get_submodel_argv, get_submodel_paths
from opendm import types
from opendm import io
from opendm import system
from opendm.dem import pdal
from opensfm.large import metadataset
from opendm import concurrency
from pipes import quote

class ODMSplitStage(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']
        photos = reconstruction.photos

        outputs['large'] = len(photos) > args.split

        if outputs['large']:
            octx = OSFMContext(tree.opensfm)
            split_done_file = octx.path("split_done.txt")

            if not io.file_exists(split_done_file) or self.rerun():

                log.ODM_INFO("Large dataset detected (%s photos) and split set at %s. Preparing split merge." % (len(photos), args.split))
                config = [
                    "submodels_relpath: ../submodels/opensfm",
                    "submodel_relpath_template: ../submodels/submodel_%04d/opensfm",
                    "submodel_images_relpath_template: ../submodels/submodel_%04d/images",
                    "submodel_size: %s" % args.split,
                    "submodel_overlap: %s" % args.split_overlap,
                ]

                octx.setup(args, tree.dataset_raw, photos, gcp_path=tree.odm_georeferencing_gcp, append_config=config, rerun=self.rerun())
            
                octx.feature_matching(self.rerun())

                # Create submodels
                if not io.dir_exists(tree.submodels_path) or self.rerun():
                    if io.dir_exists(tree.submodels_path):
                        log.ODM_WARNING("Removing existing submodels directory: %s" % tree.submodels_path)
                        shutil.rmtree(tree.submodels_path)

                    octx.run("create_submodels")
                else:
                    log.ODM_WARNING("Submodels directory already exist at: %s" % tree.submodels_path)

                # TODO: on a network workflow we probably stop here
                # and let NodeODM take over
                # exit(0)

                # Find paths of all submodels
                mds = metadataset.MetaDataSet(tree.opensfm)
                submodel_paths = [os.path.abspath(p) for p in mds.get_submodel_paths()]

                # Reconstruct each submodel
                log.ODM_INFO("Dataset has been split into %s submodels. Reconstructing each submodel..." % len(submodel_paths))

                for sp in submodel_paths:
                    log.ODM_INFO("Reconstructing %s" % sp)
                    OSFMContext(sp).reconstruct(self.rerun())

                # Align
                alignment_file = octx.path('alignment_done.txt')
                if not io.file_exists(alignment_file) or self.rerun():
                    log.ODM_INFO("Aligning submodels...")
                    octx.run('align_submodels')

                    with open(alignment_file, 'w') as fout:
                        fout.write("Alignment done!\n")
                else:
                    log.ODM_WARNING('Found a alignment matching done progress file in: %s' % alignment_file)

                # Dense reconstruction for each submodel
                for sp in submodel_paths:

                    # TODO: network workflow
                    
                    # We have already done matching
                    sp_octx = OSFMContext(sp)
                    sp_octx.mark_feature_matching_done()

                    submodel_name = os.path.basename(os.path.abspath(sp_octx.path("..")))

                    # Aligned reconstruction is in reconstruction.aligned.json
                    # We need to rename it to reconstruction.json

                    aligned_recon = sp_octx.path('reconstruction.aligned.json')
                    main_recon = sp_octx.path('reconstruction.json')

                    if not io.file_exists(aligned_recon):
                        log.ODM_WARNING("Submodel %s does not have an aligned reconstruction (%s). "
                                        "This could mean that the submodel could not be reconstructed "
                                        " (are there enough features to reconstruct it?). Skipping." % (submodel_name, aligned_recon))
                        continue

                    if io.file_exists(main_recon):
                        os.remove(main_recon)

                    os.rename(aligned_recon, main_recon)
                    log.ODM_DEBUG("%s is now %s" % (aligned_recon, main_recon))

                    log.ODM_INFO("========================")
                    log.ODM_INFO("Processing %s" % submodel_name)
                    log.ODM_INFO("========================")

                    argv = get_submodel_argv(args, tree.submodels_path, submodel_name)

                    # Re-run the ODM toolchain on the submodel
                    system.run(" ".join(map(quote, argv)), env_vars=os.environ.copy())

                with open(split_done_file, 'w') as fout: 
                    fout.write("Split done!\n")
            else:
                log.ODM_WARNING('Found a split done file in: %s' % split_done_file)
        else:
            log.ODM_INFO("Normal dataset, will process all at once.")


class ODMMergeStage(types.ODM_Stage):
    def process(self, args, outputs):
        from opendm.grass_engine import grass 

        tree = outputs['tree']
        reconstruction = outputs['reconstruction']

        if outputs['large']:
            # Merge point clouds
            # all_point_clouds = get_submodel_paths(tree.submodels_path, "odm_georeferencing", "odm_georeferenced_model.laz")
            # pdal.merge_point_clouds(all_point_clouds, tree.odm_georeferencing_model_laz, args.verbose)

            # Merge orthophotos
            all_orthophotos = get_submodel_paths(tree.submodels_path, "odm_orthophoto", "odm_orthophoto.tif")
            if len(all_orthophotos) > 1:
                gctx = grass.create_context({'auto_cleanup' : False})

                gctx.add_param('orthophoto_files', ",".join(map(quote, all_orthophotos)))
                gctx.add_param('max_concurrency', args.max_concurrency)
                gctx.add_param('memory', concurrency.get_max_memory_mb(300))
                gctx.set_location(all_orthophotos[0])

                cutline_file = gctx.execute(os.path.join("opendm", "grass", "generate_cutlines.grass"))
            
            elif len(all_orthophotos) == 1:
                # Simply copy
                log.ODM_WARNING("A single orthophoto was found between all submodels.")
                shutil.copyfile(all_orthophotos[0], tree.odm_orthophoto_tif)
            else:
                log.ODM_WARNING("No orthophotos were found in any of the submodels. No orthophoto will be generated.")

            # TODO: crop ortho if necessary

            # Merge DEM

            # TODO: crop DEM if necessary


            # Stop the pipeline short! We're done.
            self.next_stage = None
        else:
            log.ODM_INFO("Normal dataset, nothing to merge.")


        