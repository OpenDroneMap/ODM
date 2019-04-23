import os
from opendm import log
from opendm import osfm
from opendm import types
from opendm import io
from opensfm.large import metadataset

class ODMSplitStage(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']
        photos = reconstruction.photos

        outputs['large'] = len(photos) > args.split

        if outputs['large']:
            log.ODM_INFO("Large dataset detected (%s photos) and split set at %s. Preparing split merge." % (len(photos), args.split))
            config = [
                "submodels_relpath: ../submodels/opensfm",
                "submodel_relpath_template: ../submodels/submodel_%04d/opensfm",
                "submodel_images_relpath_template: ../submodels/submodel_%04d/images",
                "submodel_size: %s" % args.split,
                "submodel_overlap: %s" % args.split_overlap,
            ]
            
            osfm.setup(args, tree.dataset_raw, tree.opensfm, photos, gcp_path=tree.odm_georeferencing_gcp, append_config=config)
        
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
            log.ODM_INFO("Aligning submodels...")
            osfm.run('align_submodels', tree.opensfm)

            # Dense reconstruction for each submodel
            # TODO

            exit(1)
        else:
            log.ODM_INFO("Normal dataset, will process all at once.")



