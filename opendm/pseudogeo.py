import osr
import gdal
from gdalconst import GA_Update
from opendm import io
from opendm import log

def add_pseudo_georeferencing(geotiff, scale=1.0):
    if not io.file_exists(geotiff):
        log.ODM_WARNING("Cannot add pseudo georeferencing, %s does not exist" % geotiff)
        return

    try:
        log.ODM_INFO("Adding pseudo georeferencing (raster should show up at the equator) to %s" % geotiff)

        dst_ds = gdal.Open(geotiff, GA_Update)
        srs = osr.SpatialReference()
        srs.ImportFromProj4('+proj=utm +zone=30 +ellps=WGS84 +datum=WGS84 +units=m +no_defs')
        dst_ds.SetProjection( srs.ExportToWkt() )
        dst_ds.SetGeoTransform( [ 0.0, scale, 0.0, 0.0, 0.0, -scale ] )
        dst_ds = None

    except Exception as e:
        log.ODM_WARNING("Cannot add psuedo georeferencing to %s (%s), skipping..." % (geotiff, str(e)))