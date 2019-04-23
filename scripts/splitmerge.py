from opendm import log
from opendm import osfm

class ODMSplitStage(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']
        photos = reconstruction.photos

        config = [
            "submodels_relpath: ../submodels/opensfm",
            "submodel_relpath_template: ../submodels/submodel_%04d/opensfm",
            "submodel_images_relpath_template: ../submodels/submodel_%04d/images",
            "submodel_size: %s" % args.split,
            "submodel_overlap: %s" % args.split_overlap,
        ]
        
        osfm.setup(args, self.params, tree.dataset_raw, tree.opensfm, photos, gcp_path=tree.odm_georeferencing_gcp, append_config=config)


