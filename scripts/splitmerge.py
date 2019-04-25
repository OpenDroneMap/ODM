import os
import shutil
from opendm import log
from opendm import osfm
from opendm import types
from opendm import io
from opendm import system
from opendm.dem import pdal
from opensfm.large import metadataset
from pipes import quote

class ODMSplitStage(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']
        photos = reconstruction.photos

        outputs['large'] = len(photos) > args.split

        if outputs['large']:
            split_done_file = os.path.join(tree.opensfm, "split_done.txt")

            if not io.file_exists(split_done_file) or self.rerun():

                log.ODM_INFO("Large dataset detected (%s photos) and split set at %s. Preparing split merge." % (len(photos), args.split))
                config = [
                    "submodels_relpath: ../submodels/opensfm",
                    "submodel_relpath_template: ../submodels/submodel_%04d/opensfm",
                    "submodel_images_relpath_template: ../submodels/submodel_%04d/images",
                    "submodel_size: %s" % args.split,
                    "submodel_overlap: %s" % args.split_overlap,
                ]
                
                osfm.setup(args, tree.dataset_raw, tree.opensfm, photos, gcp_path=tree.odm_georeferencing_gcp, append_config=config, rerun=self.rerun())
            
                osfm.feature_matching(tree.opensfm, self.rerun())

                # Create submodels
                if not io.dir_exists(tree.submodels_path) or self.rerun():
                    if io.dir_exists(tree.submodels_path):
                        log.ODM_WARNING("Removing existing submodels directory: %s" % tree.submodels_path)
                        shutil.rmtree(tree.submodels_path)

                    osfm.run("create_submodels", tree.opensfm)
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
                    osfm.reconstruct(sp, self.rerun())

                # Align
                alignment_file = io.join_paths(tree.opensfm, 'alignment_done.txt')
                if not io.file_exists(alignment_file) or self.rerun():
                    log.ODM_INFO("Aligning submodels...")
                    osfm.run('align_submodels', tree.opensfm)
                    with open(alignment_file, 'w') as fout:
                        fout.write("Alignment done!\n")
                else:
                    log.ODM_WARNING('Found a alignment matching done progress file in: %s' % alignment_file)

                # Dense reconstruction for each submodel
                for sp in submodel_paths:

                    # TODO: network workflow
                    
                    # We have already done matching
                    osfm.mark_feature_matching_done(sp)

                    submodel_name = os.path.basename(os.path.abspath(os.path.join(sp, "..")))

                    log.ODM_INFO("========================")
                    log.ODM_INFO("Processing %s" % submodel_name)
                    log.ODM_INFO("========================")

                    argv = osfm.get_submodel_argv(args, tree.submodels_path, submodel_name)

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
        from opendm import grass_engine

        tree = outputs['tree']
        reconstruction = outputs['reconstruction']

        if outputs['large']:
            # Merge point clouds
            all_point_clouds = osfm.get_submodel_paths(tree.submodels_path, "odm_georeferencing", "odm_georeferenced_model.laz")
            pdal.merge_point_clouds(all_point_clouds, tree.odm_georeferencing_model_laz, args.verbose)

            # Merge orthophoto

            # TODO: crop ortho if necessary

            # Merge DEM

            # TODO: crop DEM if necessary


            # Stop the pipeline short! We're done.
            self.next_stage = None
        else:
            log.ODM_INFO("Normal dataset, nothing to merge.")


        