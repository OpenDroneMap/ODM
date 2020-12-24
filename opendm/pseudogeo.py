from osgeo import osr
from osgeo import gdal
from osgeo.gdalconst import GA_Update
from opendm import io
from opendm import log

def get_pseudogeo_utm():
    return '+proj=utm +zone=30 +ellps=WGS84 +datum=WGS84 +units=m +no_defs'

def get_pseudogeo_scale():
    return 0.1 # Arbitrarily chosen

def add_pseudo_georeferencing(geotiff):
    if not io.file_exists(geotiff):
        log.ODM_WARNING("Cannot add pseudo georeferencing, %s does not exist" % geotiff)
        return

    try:
        log.ODM_INFO("Adding pseudo georeferencing (raster should show up at the equator) to %s" % geotiff)

        dst_ds = gdal.Open(geotiff, GA_Update)
        srs = osr.SpatialReference()
        srs.SetAxisMappingStrategy(osr.OAMS_TRADITIONAL_GIS_ORDER)
        srs.ImportFromProj4(get_pseudogeo_utm())
        dst_ds.SetProjection( srs.ExportToWkt() )
        dst_ds.SetGeoTransform( [ 0.0, get_pseudogeo_scale(), 0.0, 0.0, 0.0, -get_pseudogeo_scale() ] )
        dst_ds = None

    except Exception as e:
        log.ODM_WARNING("Cannot add psuedo georeferencing to %s (%s), skipping..." % (geotiff, str(e)))