from opendm import context
from opendm.system import run
from opendm import log
from osgeo import ogr
import json, os
from psutil import virtual_memory

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
    def crop(shapefile_path, geotiff_path, gdal_options, keep_original=True):
        if not os.path.exists(shapefile_path) or not os.path.exists(geotiff_path):
            log.ODM_WARNING("Either {} or {} does not exist, will skip cropping.".format(shapefile_path, geotiff_path))
            return geotiff_path

        # Rename original file
        # path/to/odm_orthophoto.tif --> path/to/odm_orthophoto.original.tif
        
        path, filename = os.path.split(geotiff_path)
        # path = path/to
        # filename = odm_orthophoto.tif

        basename, ext = os.path.splitext(filename)
        # basename = odm_orthophoto
        # ext = .tif

        original_geotiff = os.path.join(path, "{}.original{}".format(basename, ext))
        os.rename(geotiff_path, original_geotiff)

        try:
            kwargs = {
                'shapefile_path': shapefile_path,
                'geotiffInput': original_geotiff,
                'geotiffOutput': geotiff_path,
                'options': ' '.join(map(lambda k: '-co {}={}'.format(k, gdal_options[k]), gdal_options)),
                'max_memory': max(5, (100 - virtual_memory().percent) / 2)
            }

            run('gdalwarp -cutline {shapefile_path} '
                '-crop_to_cutline '
                '{options} '
                '{geotiffInput} '
                '{geotiffOutput} '
                '--config GDAL_CACHEMAX {max_memory}%'.format(**kwargs))

            if not keep_original:
                os.remove(original_geotiff)

        except Exception as e:
            log.ODM_WARNING('Something went wrong while cropping: {}'.format(e.message))
            
            # Revert rename
            os.rename(original_geotiff, geotiff_path)

        return geotiff_path

    def create_bounds_geojson(self, pointcloud_path, buffer_distance = 0):
        """
        Compute a buffered polygon around the data extents (not just a bounding box)
        of the given point cloud.

        @return filename to GeoJSON containing the polygon
        """
        if not os.path.exists(pointcloud_path):
            log.ODM_WARNING('Point cloud does not exist, cannot generate shapefile bounds {}'.format(pointcloud_path))
            return ''

        # Do basic outlier removal prior to extracting boundary information
        filtered_pointcloud_path = self.path('filtered.las')

        run("pdal translate -i \"{}\" "
            "-o \"{}\" "
            "decimation outlier range "
            "--filters.decimation.step=40 "
            "--filters.outlier.method=radius "
            "--filters.outlier.radius=20 "
            "--filters.outlier.min_k=2 "
            "--filters.range.limits='Classification![7:7]'".format(pointcloud_path, filtered_pointcloud_path))

        if not os.path.exists(filtered_pointcloud_path):
            log.ODM_WARNING('Could not filter point cloud, cannot generate shapefile bounds {}'.format(filtered_pointcloud_path))
            return ''

        # Use PDAL to dump boundary information
        # then read the information back

        boundary_file_path = self.path('boundary.json')

        run('pdal info --boundary --filters.hexbin.edge_length=1 --filters.hexbin.threshold=0 {0} > {1}'.format(filtered_pointcloud_path,  boundary_file_path))
        
        pc_geojson_boundary_feature = None

        with open(boundary_file_path, 'r') as f:
            json_f = json.loads(f.read())
            pc_geojson_boundary_feature = json_f['boundary']['boundary_json']

        if pc_geojson_boundary_feature is None: raise RuntimeError("Could not determine point cloud boundaries")

        # Write bounds to GeoJSON
        bounds_geojson_path = self.path('bounds.geojson')
        with open(bounds_geojson_path, "w") as f:
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
        ds = driver.Open(bounds_geojson_path, 0) # ready-only
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
            convexhull = convexhull.Buffer(-(buffer_distance + BUFFER_SMOOTH_DISTANCE))
            convexhull = convexhull.Buffer(BUFFER_SMOOTH_DISTANCE)

        # Save to a new file
        bounds_geojson_path = self.path('bounds.geojson')
        if os.path.exists(bounds_geojson_path):
            driver.DeleteDataSource(bounds_geojson_path)

        out_ds = driver.CreateDataSource(bounds_geojson_path)
        layer = out_ds.CreateLayer("convexhull", geom_type=ogr.wkbPolygon)

        feature_def = layer.GetLayerDefn()
        feature = ogr.Feature(feature_def)
        feature.SetGeometry(convexhull)
        layer.CreateFeature(feature)
        feature = None

        # Save and close data sources
        out_ds = ds = None

        # Remove filtered point cloud
        if os.path.exists(filtered_pointcloud_path):
            os.remove(filtered_pointcloud_path)

        return bounds_geojson_path


    def create_bounds_shapefile(self, pointcloud_path, buffer_distance = 0):
        """
        Compute a buffered polygon around the data extents (not just a bounding box)
        of the given point cloud.
        
        @return filename to Shapefile containing the polygon
        """
        if not os.path.exists(pointcloud_path):
            log.ODM_WARNING('Point cloud does not exist, cannot generate shapefile bounds {}'.format(pointcloud_path))
            return ''

        bounds_geojson_path = self.create_bounds_geojson(pointcloud_path, buffer_distance)

        summary_file_path = os.path.join(self.storage_dir, '{}.summary.json'.format(self.files_prefix))
        run('pdal info --summary {0} > {1}'.format(pointcloud_path, summary_file_path))
        
        pc_proj4 = None
        with open(summary_file_path, 'r') as f:
            json_f = json.loads(f.read())
            pc_proj4 = json_f['summary']['srs']['proj4']

        if pc_proj4 is None: raise RuntimeError("Could not determine point cloud proj4 declaration")

        bounds_shapefile_path = os.path.join(self.storage_dir, '{}.bounds.shp'.format(self.files_prefix))

        # Convert bounds to Shapefile
        kwargs = {
            'input': bounds_geojson_path,
            'output': bounds_shapefile_path,
            'proj4': pc_proj4
        }

        run('ogr2ogr -overwrite -a_srs "{proj4}" {output} {input}'.format(**kwargs))

        return bounds_shapefile_path

