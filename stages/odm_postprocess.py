import os

from osgeo import gdal
from opendm import io
from opendm import log
from opendm import types
from opendm.utils import copy_paths, get_processing_results_paths

class ODMPostProcess(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']

        log.ODM_INFO("Post Processing")

        if not outputs['large']:
            # TODO: support for split-merge?

            # Embed GCP info in 2D results via
            # XML metadata fields
            gcp_gml_export_file = tree.path("odm_georeferencing", "ground_control_points.gml")

            if reconstruction.has_gcp() and io.file_exists(gcp_gml_export_file):
                skip_embed_gcp = False
                gcp_xml = ""

                with open(gcp_gml_export_file) as f:
                    gcp_xml = f.read()

                for product in [tree.odm_orthophoto_tif,
                                tree.path("odm_dem", "dsm.tif"),
                                tree.path("odm_dem", "dtm.tif")]:
                    if os.path.isfile(product):
                        ds = gdal.Open(product)
                        if ds is not None:
                            if ds.GetMetadata('xml:GROUND_CONTROL_POINTS') is None or self.rerun():
                                ds.SetMetadata(gcp_xml, 'xml:GROUND_CONTROL_POINTS')
                                ds = None
                                log.ODM_INFO("Wrote xml:GROUND_CONTROL_POINTS metadata to %s" % product)
                            else:
                                skip_embed_gcp = True
                                log.ODM_WARNING("Already embedded ground control point information")
                                break
                        else:
                            log.ODM_WARNING("Cannot open %s for writing, skipping GCP embedding" % product)
                
        if args.copy_to:
            try:
                copy_paths([os.path.join(args.project_path, p) for p in get_processing_results_paths()], args.copy_to, self.rerun())
            except Exception as e:
                log.ODM_WARNING("Cannot copy to %s: %s" % (args.copy_to, str(e)))
