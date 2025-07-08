import os, shutil
import numpy as np
import json
import rasterio
from osgeo import gdal
from datetime import datetime

from opendm import log
from opendm.photo import find_largest_photo_dims, find_mean_utc_time
from osgeo import gdal
from opendm.arghelpers import double_quote

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


def get_depthmap_resolution(args, photos):
    max_dims = find_largest_photo_dims(photos)
    min_dim = 320 # Never go lower than this

    if max_dims is not None:
        w, h = max_dims
        max_dim = max(w, h)

        megapixels = (w * h) / 1e6
        multiplier = 1
        
        if megapixels < 6:
            multiplier = 2
        elif megapixels > 42:
            multiplier = 0.5
        
        pc_quality_scale = {
            'ultra': 0.5,
            'high': 0.25,
            'medium': 0.125,
            'low': 0.0675,
            'lowest': 0.03375
        }

        return max(min_dim, int(max_dim * pc_quality_scale[args.pc_quality] * multiplier))
    else:
        log.ODM_WARNING("Cannot compute max image dimensions, going with default depthmap_resolution of 640")
        return 640 # Sensible default

def get_raster_stats(geotiff):
    stats = []
    gtif = gdal.Open(geotiff)
    for b in range(gtif.RasterCount):
        srcband = gtif.GetRasterBand(b + 1)
        s = srcband.GetStatistics(True, True)

        stats.append({
            'min': s[0],
            'max': s[1],
            'mean': s[2],
            'stddev': s[3]
        })
            
    return stats

def get_processing_results_paths():
    return [
        "odm_georeferencing",
        "odm_orthophoto",
        "odm_dem",
        "odm_report",
        "odm_texturing",
        "entwine_pointcloud",
        "dsm_tiles",
        "dtm_tiles",
        "orthophoto_tiles",
        "3d_tiles",
        "images.json",
        "cameras.json",
        "log.json",
    ]

def copy_paths(paths, destination, rerun):
    if not os.path.isdir(destination):
        os.makedirs(destination)

    for p in paths:
        basename = os.path.basename(p)
        dst_path = os.path.join(destination, basename)

        if rerun:
            try:
                if os.path.isfile(dst_path) or os.path.islink(dst_path):
                    os.remove(dst_path)
                elif os.path.isdir(dst_path):
                    shutil.rmtree(dst_path)
            except Exception as e:
                log.ODM_WARNING("Cannot remove file %s: %s, skipping..." % (dst_path, str(e)))

        if not os.path.exists(dst_path):
            if os.path.isfile(p):
                log.ODM_INFO("Copying %s --> %s" % (p, dst_path))
                shutil.copy(p, dst_path)
            elif os.path.isdir(p):
                shutil.copytree(p, dst_path)
                log.ODM_INFO("Copying %s --> %s" % (p, dst_path))

def rm_r(path):
    try:
        if os.path.isdir(path) and not os.path.islink(path):
            shutil.rmtree(path)
        elif os.path.exists(path):
            os.remove(path)
    except:
        log.ODM_WARNING("Cannot remove %s" % path)

def np_to_json(arr):
    return json.dumps(arr, cls=NumpyEncoder)

def np_from_json(json_dump):
    return np.asarray(json.loads(json_dump))

def add_raster_meta_tags(raster, reconstruction, tree, embed_gcp_meta=True):
    try:
        if os.path.isfile(raster):
            mean_capture_time = find_mean_utc_time(reconstruction.photos)
            mean_capture_dt = None
            if mean_capture_time is not None:
                mean_capture_dt = datetime.fromtimestamp(mean_capture_time).strftime('%Y:%m:%d %H:%M:%S') + '+00:00'

            log.ODM_INFO("Adding TIFFTAGs to {}".format(raster))
            with rasterio.open(raster, 'r+') as rst:
                if mean_capture_dt is not None:
                    rst.update_tags(TIFFTAG_DATETIME=mean_capture_dt)
                rst.update_tags(TIFFTAG_SOFTWARE='ODM {}'.format(log.odm_version()))

            if embed_gcp_meta:
                # Embed GCP info in 2D results via
                # XML metadata fields
                gcp_gml_export_file = tree.path("odm_georeferencing", "ground_control_points.gml")

                if reconstruction.has_gcp() and os.path.isfile(gcp_gml_export_file):
                    gcp_xml = ""

                    with open(gcp_gml_export_file) as f:
                        gcp_xml = f.read()

                    ds = gdal.Open(raster)
                    if ds is not None:
                        if ds.GetMetadata('xml:GROUND_CONTROL_POINTS') is None or self.rerun():
                            ds.SetMetadata(gcp_xml, 'xml:GROUND_CONTROL_POINTS')
                            ds = None
                            log.ODM_INFO("Wrote xml:GROUND_CONTROL_POINTS metadata to %s" % raster)
                        else:
                            log.ODM_WARNING("Already embedded ground control point information")
                    else:
                        log.ODM_WARNING("Cannot open %s for writing, skipping GCP embedding" % raster)
    except Exception as e:
        log.ODM_WARNING("Cannot write raster meta tags to %s: %s" % (raster, str(e)))
