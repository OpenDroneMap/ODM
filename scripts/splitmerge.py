from opendm import log
from opendm import osfm
from opendm import types

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
        
            osfm.run_feature_matching(tree.opensfm, self.rerun())

            # Create submodels
            osfm.run("create_submodels", tree.opensfm)
            exit(1)
        else:
            log.ODM_INFO("Normal dataset, will process all at once.")

