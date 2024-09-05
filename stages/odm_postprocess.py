import os
import rasterio
import json
import numpy as np

from datetime import datetime
from osgeo import gdal
from opendm import io
from opendm import log
from opendm import types
from opendm import context
from opendm.utils import copy_paths, get_processing_results_paths
from opendm.ogctiles import build_3dtiles

class ODMPostProcess(types.ODM_Stage):
    def process(self, args, outputs):
        tree = outputs['tree']
        reconstruction = outputs['reconstruction']

        log.ODM_INFO("Post Processing")

        # -- TIFFTAG information - add datetime and software version to .tif metadata 

        # Gather information 
        # ODM version ...
        with open(os.path.join(context.root_path, 'VERSION')) as version_file:
            version = version_file.read().strip()
        # Datetimes  ...
        shots_file = tree.path("odm_report", "shots.geojson")
        if not args.skip_report and os.path.isfile(shots_file): 
            # Open file
            with open(shots_file, 'r') as f:
                odm_shots = json.loads(f.read())
            # Compute mean time 
            cts = []
            for feat in odm_shots["features"]:
                ct = feat["properties"]["capture_time"]
                cts.append(ct)
            mean_dt = datetime.fromtimestamp(np.mean(cts)) 
            # Format it
            CAPTURE_DATETIME = mean_dt.strftime('%Y:%m:%d %H:%M') + '+00:00' #UTC
        else:
            #try instead with images.json file
            images_file = tree.path('images.json') 
            if os.path.isfile(images_file): 
                # Open file
                with open(images_file, 'r') as f:
                    imgs = json.loads(f.read())
                # Compute mean time 
                cts = []
                for img in imgs: 
                    ct = img["utc_time"]/1000. #ms to s
                    cts.append(ct) 
                mean_dt = datetime.fromtimestamp(np.mean(cts)) 
                # Format it
                CAPTURE_DATETIME = mean_dt.strftime('%Y:%m:%d %H:%M') + '+00:00' #UTC
            else:
                CAPTURE_DATETIME = None

        # Add it 
        for product in [tree.odm_orthophoto_tif,
                        tree.path("odm_dem", "dsm.tif"),
                        tree.path("odm_dem", "dtm.tif")]:
            for pdt in [product, product.replace('.tif', '.original.tif')]:
                if os.path.isfile(pdt):
                    log.ODM_INFO("Adding TIFFTAGs to {} ...".format(pdt))
                    with rasterio.open(pdt, 'r+') as rst:
                        rst.update_tags(TIFFTAG_DATETIME=CAPTURE_DATETIME)
                        rst.update_tags(TIFFTAG_SOFTWARE='OpenDroneMap {}'.format(version))

        #  -- GCP info
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

        # -- 3D tiles
        if getattr(args, '3d_tiles'):
            build_3dtiles(args, tree, reconstruction, self.rerun())

        # -- Copy to 
        if args.copy_to:
            try:
                copy_paths([os.path.join(args.project_path, p) for p in get_processing_results_paths()], args.copy_to, self.rerun())
            except Exception as e:
                log.ODM_WARNING("Cannot copy to %s: %s" % (args.copy_to, str(e)))

