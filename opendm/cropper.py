from opendm import context
from opendm.system import run
from opendm import log
from opendm.point_cloud import export_summary_json
from osgeo import ogr
import json, os
from opendm.concurrency import get_max_memory

class Cropper:
    def __init__(self, storage_dir, files_prefix = "crop"):
        self.storage_dir = storage_dir
        self.files_prefix = files_prefix

    def path(self, suffix):
        """
        @return a path relative to storage_dir and prefixed with files_prefix
        """
        return os.path.join(self.storage_dir, '{}.{}'.format(self.files_prefix, suffix))

    @staticmethod
    def crop(gpkg_path, geotiff_path, gdal_options, keep_original=True, warp_options=[]):
        if not os.path.exists(gpkg_path) or not os.path.exists(geotiff_path):
            log.ODM_WARNING("Either {} or {} does not exist, will skip cropping.".format(gpkg_path, geotiff_path))
            return geotiff_path

        log.ODM_INFO("Cropping %s" % geotiff_path)

        # Rename original file
        # path/to/odm_orthophoto.tif --> path/to/odm_orthophoto.original.tif
        
        path, filename = os.path.split(geotiff_path)
        # path = path/to
        # filename = odm_orthophoto.tif

        basename, ext = os.path.splitext(filename)
        # basename = odm_orthophoto
        # ext = .tif

        original_geotiff = os.path.join(path, "{}.original{}".format(basename, ext))
        os.replace(geotiff_path, original_geotiff)

        try:
            kwargs = {
                'gpkg_path': gpkg_path,
                'geotiffInput': original_geotiff,
                'geotiffOutput': geotiff_path,
                'options': ' '.join(map(lambda k: '-co {}={}'.format(k, gdal_options[k]), gdal_options)),
                'warpOptions': ' '.join(warp_options),
                'max_memory': get_max_memory()
            }

            run('gdalwarp -cutline {gpkg_path} '
                '-crop_to_cutline '
                '{options} '
                '{warpOptions} '
                '{geotiffInput} '
                '{geotiffOutput} '
                '--config GDAL_CACHEMAX {max_memory}%'.format(**kwargs))

            if not keep_original:
                os.remove(original_geotiff)

        except Exception as e:
            log.ODM_WARNING('Something went wrong while cropping: {}'.format(e))
            
            # Revert rename
            os.replace(original_geotiff, geotiff_path)

        return geotiff_path

    @staticmethod
    def merge_bounds(input_bound_files, output_bounds, buffer_distance = 0):
        """
        Merge multiple bound files into a single bound computed from the convex hull
        of all bounds (minus a buffer distance in meters)
        """
        geomcol = ogr.Geometry(ogr.wkbGeometryCollection)

        driver = ogr.GetDriverByName('GPKG')
        srs = None

        for input_bound_file in input_bound_files:
            ds = driver.Open(input_bound_file, 0) # ready-only

            layer = ds.GetLayer()
            srs = layer.GetSpatialRef()

            # Collect all Geometry
            for feature in layer:
                geomcol.AddGeometry(feature.GetGeometryRef())
            
            ds = None

        # Calculate convex hull
        convexhull = geomcol.ConvexHull()

        # If buffer distance is specified
        # Create two buffers, one shrinked by
        # N + 3 and then that buffer expanded by 3
        # so that we get smooth corners. \m/
        BUFFER_SMOOTH_DISTANCE = 3

        if buffer_distance > 0:
            convexhull = convexhull.Buffer(-(buffer_distance + BUFFER_SMOOTH_DISTANCE))
            convexhull = convexhull.Buffer(BUFFER_SMOOTH_DISTANCE)

        # Save to a new file
        if os.path.exists(output_bounds):
            driver.DeleteDataSource(output_bounds)

        out_ds = driver.CreateDataSource(output_bounds)
        layer = out_ds.CreateLayer("convexhull", srs=srs, geom_type=ogr.wkbPolygon)

        feature_def = layer.GetLayerDefn()
        feature = ogr.Feature(feature_def)
        feature.SetGeometry(convexhull)
        layer.CreateFeature(feature)
        feature = None

        # Save and close output data source
        out_ds = None

    def create_bounds_geojson(self, pointcloud_path, buffer_distance = 0, decimation_step=40):
        """
        Compute a buffered polygon around the data extents (not just a bounding box)
        of the given point cloud.

        @return filename to GeoJSON containing the polygon
        """
        if not os.path.exists(pointcloud_path):
            log.ODM_WARNING('Point cloud does not exist, cannot generate bounds {}'.format(pointcloud_path))
            return ''

        # Do decimation prior to extracting boundary information
        decimated_pointcloud_path = self.path('decimated.las')

        run("pdal translate -i \"{}\" "
            "-o \"{}\" "
            "decimation "
            "--filters.decimation.step={} ".format(pointcloud_path, decimated_pointcloud_path, decimation_step))

        if not os.path.exists(decimated_pointcloud_path):
            log.ODM_WARNING('Could not decimate point cloud, thus cannot generate GPKG bounds {}'.format(decimated_pointcloud_path))
            return ''

        # Use PDAL to dump boundary information
        # then read the information back

        boundary_file_path = self.path('boundary.json')

        run('pdal info --boundary --filters.hexbin.edge_size=1 --filters.hexbin.threshold=0 "{0}" > "{1}"'.format(decimated_pointcloud_path,  boundary_file_path))
        
        pc_geojson_boundary_feature = None

        with open(boundary_file_path, 'r') as f:
            json_f = json.loads(f.read())
            pc_geojson_boundary_feature = json_f['boundary']['boundary_json']

        if pc_geojson_boundary_feature is None: raise RuntimeError("Could not determine point cloud boundaries")

        # Write bounds to GeoJSON
        tmp_bounds_geojson_path = self.path('tmp-bounds.geojson')
        with open(tmp_bounds_geojson_path, "w") as f:
            f.write(json.dumps({
                "type": "FeatureCollection",
                "features": [{
                    "type": "Feature",
                    "geometry": pc_geojson_boundary_feature
                }]
            }))

        # Create a convex hull around the boundary
        # as to encompass the entire area (no holes)    
        driver = ogr.GetDriverByName('GeoJSON')
        ds = driver.Open(tmp_bounds_geojson_path, 0) # ready-only
        layer = ds.GetLayer()

        # Collect all Geometry
        geomcol = ogr.Geometry(ogr.wkbGeometryCollection)
        for feature in layer:
            geomcol.AddGeometry(feature.GetGeometryRef())

        # Calculate convex hull
        convexhull = geomcol.ConvexHull()

        # If buffer distance is specified
        # Create two buffers, one shrinked by
        # N + 3 and then that buffer expanded by 3
        # so that we get smooth corners. \m/
        BUFFER_SMOOTH_DISTANCE = 3

        if buffer_distance > 0:
            # For small areas, check that buffering doesn't obliterate 
            # our hull
            tmp = convexhull.Buffer(-(buffer_distance + BUFFER_SMOOTH_DISTANCE))
            tmp = tmp.Buffer(BUFFER_SMOOTH_DISTANCE)
            if tmp.Area() > 0:
                convexhull = tmp
            else:
                log.ODM_WARNING("Very small crop area detected, we will not smooth it.")

        # Save to a new file
        bounds_geojson_path = self.path('bounds.geojson')
        if os.path.exists(bounds_geojson_path):
            os.remove(bounds_geojson_path)

        out_ds = driver.CreateDataSource(bounds_geojson_path)
        layer = out_ds.CreateLayer("convexhull", geom_type=ogr.wkbPolygon)

        feature_def = layer.GetLayerDefn()
        feature = ogr.Feature(feature_def)
        feature.SetGeometry(convexhull)
        layer.CreateFeature(feature)
        feature = None

        # Save and close data sources
        out_ds = ds = None

        # Remove decimated point cloud
        if os.path.exists(decimated_pointcloud_path):
            os.remove(decimated_pointcloud_path)
        
        # Remove tmp bounds
        if os.path.exists(tmp_bounds_geojson_path):
            os.remove(tmp_bounds_geojson_path)

        return bounds_geojson_path


    def create_bounds_gpkg(self, pointcloud_path, buffer_distance = 0, decimation_step=40):
        """
        Compute a buffered polygon around the data extents (not just a bounding box)
        of the given point cloud.
        
        @return filename to Geopackage containing the polygon
        """
        if not os.path.exists(pointcloud_path):
            log.ODM_WARNING('Point cloud does not exist, cannot generate GPKG bounds {}'.format(pointcloud_path))
            return ''

        bounds_geojson_path = self.create_bounds_geojson(pointcloud_path, buffer_distance, decimation_step)

        summary_file_path = os.path.join(self.storage_dir, '{}.summary.json'.format(self.files_prefix))
        export_summary_json(pointcloud_path, summary_file_path)
        
        pc_proj4 = None
        with open(summary_file_path, 'r') as f:
            json_f = json.loads(f.read())
            pc_proj4 = json_f['summary']['srs']['proj4']

        if pc_proj4 is None: raise RuntimeError("Could not determine point cloud proj4 declaration")

        bounds_gpkg_path = os.path.join(self.storage_dir, '{}.bounds.gpkg'.format(self.files_prefix))

        # Convert bounds to GPKG
        kwargs = {
            'input': bounds_geojson_path,
            'output': bounds_gpkg_path,
            'proj4': pc_proj4
        }

        run('ogr2ogr -overwrite -f GPKG -a_srs "{proj4}" {output} {input}'.format(**kwargs))

        return bounds_gpkg_path

