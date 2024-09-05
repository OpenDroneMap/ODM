import os
import rasterio

from datetime import datetime
from osgeo import gdal
from opendm import io
from opendm import log
from opendm import types
from opendm import photo
from opendm.utils import copy_paths, get_processing_results_paths
from opendm.ogctiles import build_3dtiles

class ODMPostProcess(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']

        log.ODM_INFO("Post Processing")

        rasters = [tree.odm_orthophoto_tif,
                    tree.path("odm_dem", "dsm.tif"),
                    tree.path("odm_dem", "dtm.tif")]

        mean_capture_time = photo.find_mean_utc_time(reconstruction.photos)
        mean_capture_dt = None
        if mean_capture_time is not None:
            mean_capture_dt = datetime.fromtimestamp(mean_capture_time).strftime('%Y:%m:%d %H:%M:%S') + '+00:00'

        # Add TIFF tags
        for product in rasters:
            if os.path.isfile(product):
                log.ODM_INFO("Adding TIFFTAGs to {}".format(product))
                with rasterio.open(product, 'r+') as rst:
                    if mean_capture_dt is not None:
                        rst.update_tags(TIFFTAG_DATETIME=mean_capture_dt)
                    rst.update_tags(TIFFTAG_SOFTWARE='ODM {}'.format(log.odm_version()))

        # GCP info
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

                for product in rasters:
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

        if getattr(args, '3d_tiles'):
            build_3dtiles(args, tree, reconstruction, self.rerun())

        if args.copy_to:
            try:
                copy_paths([os.path.join(args.project_path, p) for p in get_processing_results_paths()], args.copy_to, self.rerun())
            except Exception as e:
                log.ODM_WARNING("Cannot copy to %s: %s" % (args.copy_to, str(e)))

