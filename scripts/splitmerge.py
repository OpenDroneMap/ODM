import os
import shutil
from opendm import log
from opendm.osfm import OSFMContext, get_submodel_argv, get_submodel_paths, get_all_submodel_paths
from opendm import types
from opendm import io
from opendm import system
from opendm import orthophoto
from opendm.dem import pdal
from opensfm.large import metadataset
from opendm.cropper import Cropper
from opendm.concurrency import get_max_memory
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

                    shutil.move(aligned_recon, main_recon)
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
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']

        if outputs['large']:
            # Merge point clouds
            if not io.file_exists(tree.odm_georeferencing_model_laz) or self.rerun():
                pass
                # all_point_clouds = get_submodel_paths(tree.submodels_path, "odm_georeferencing", "odm_georeferenced_model.laz")
                # pdal.merge_point_clouds(all_point_clouds, tree.odm_georeferencing_model_laz, args.verbose)
            else:
                log.ODM_WARNING("Found merged point cloud in %s" % tree.odm_georeferencing_model_laz)
            
            # Merge crop bounds
            merged_bounds_file = os.path.join(tree.odm_georeferencing, 'odm_georeferenced_model.bounds.gpkg')
            if not io.file_exists(merged_bounds_file) or self.rerun():
                all_bounds = get_submodel_paths(tree.submodels_path, 'odm_georeferencing', 'odm_georeferenced_model.bounds.gpkg')
                log.ODM_DEBUG("Merging all crop bounds: %s" % all_bounds)
                if len(all_bounds) > 0:
                    # Calculate a new crop area
                    # based on the convex hull of all crop areas of all submodels
                    # (without a buffer, otherwise we are double-cropping)
                    Cropper.merge_bounds(all_bounds, merged_bounds_file, 0)
                else:
                    log.ODM_WARNING("No bounds found for any submodel.")

            # Merge orthophotos
            if not io.file_exists(tree.odm_orthophoto_tif) or self.rerun():
                all_orthos_and_cutlines = get_all_submodel_paths(tree.submodels_path,
                    os.path.join("odm_orthophoto", "odm_orthophoto.tif"),
                    os.path.join("odm_orthophoto", "cutline.gpkg"),
                )

                if len(all_orthos_and_cutlines) > 1:
                    log.ODM_DEBUG("Found %s submodels with valid orthophotos and cutlines" % len(all_orthos_and_cutlines))
                    
                    merged_geotiff = os.path.join(tree.odm_orthophoto, "odm_orthophoto.merged.tif")

                    kwargs = {
                        'orthophoto_merged': merged_geotiff,
                        'input_files': ' '.join(map(lambda i: quote(i[0]), all_orthos_and_cutlines)),
                    }

                    # use bounds as cutlines (blending)
                    if io.file_exists(merged_geotiff):
                        os.remove(merged_geotiff)

                    system.run('gdal_merge.py -o {orthophoto_merged} '
                            '-createonly '
                            '-co "BIGTIFF=YES" '
                            '-co "BLOCKXSIZE=512" '
                            '-co "BLOCKYSIZE=512" '
                            '{input_files} '.format(**kwargs)
                            )

                    for ortho_cutline in all_orthos_and_cutlines:
                        kwargs['input_file'], kwargs['cutline'] = ortho_cutline

                        system.run('gdalwarp -cutline {cutline} '
                                #'-cblend 2 '
                                '-r lanczos -multi '
                                ' {input_file} {orthophoto_merged}'.format(**kwargs)
                        )

                    # Apply orthophoto settings (compression, tiling, etc.)
                    orthophoto_vars = orthophoto.get_orthophoto_vars(args)

                    if io.file_exists(tree.odm_orthophoto_tif):
                        os.remove(tree.odm_orthophoto_tif)

                    kwargs = {
                        'vars': ' '.join(['-co %s=%s' % (k, orthophoto_vars[k]) for k in orthophoto_vars]),
                        'max_memory': get_max_memory(),
                        'merged': merged_geotiff,
                        'log': tree.odm_orthophoto_tif_log,
                        'orthophoto': tree.odm_orthophoto_tif,
                    }

                    system.run('gdal_translate '
                           '{vars} '
                           '--config GDAL_CACHEMAX {max_memory}% '
                           '{merged} {orthophoto} > {log}'.format(**kwargs))

                    os.remove(merged_geotiff)

                    # Crop
                    if args.crop > 0:
                        Cropper.crop(merged_bounds_file, tree.odm_orthophoto_tif, orthophoto_vars)

                    # Overviews
                    if args.build_overviews:
                        orthophoto.build_overviews(tree.odm_orthophoto_tif) 
                    
                elif len(all_orthos_and_cutlines) == 1:
                    # Simply copy
                    log.ODM_WARNING("A single orthophoto/cutline pair was found between all submodels.")
                    shutil.copyfile(all_orthos_and_cutlines[0][0], tree.odm_orthophoto_tif)
                else:
                    log.ODM_WARNING("No orthophoto/cutline pairs were found in any of the submodels. No orthophoto will be generated.")
            else:
                log.ODM_WARNING("Found merged orthophoto in %s" % tree.odm_orthophoto_tif)

            # Merge DEM

            # TODO: crop DEM if necessary


            # Stop the pipeline short! We're done.
            self.next_stage = None
        else:
            log.ODM_INFO("Normal dataset, nothing to merge.")


        