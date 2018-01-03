from opendm import context
from opendm import system
from osgeo import ogr
import json, os

def run(command):
    env_paths = [context.superbuild_bin_path]
    return system.run(command, env_paths)

class Clipper:
    def __init__(self, storageDirectory, filesPrefix = "clip"):
        self.storageDirectory = storageDirectory
        self.filesPrefix = filesPrefix

    def path(self, suffix):
        return os.path.join(self.storageDirectory, '{}.{}'.format(self.filesPrefix, suffix))

    def create_buffer_geojson(self, pointCloudPath, bufferDistance = 0):
        # Use PDAL to dump boundary information
        # then read the information back

        boundary_file_path = self.path('boundary.json')

        run('pdal info --boundary --filters.hexbin.edge_length=1 --filters.hexbin.threshold=0 {0} > {1}'.format(pointCloudPath,  boundary_file_path))
        
        pc_geojson_boundary_feature = None

        with open(boundary_file_path, 'r') as f:
            json_f = json.loads(f.read())
            pc_geojson_boundary_feature = json_f['boundary']['boundary_json']

        if pc_geojson_boundary_feature is None: raise RuntimeError("Could not determine point cloud boundaries")

        # Write bounds to GeoJSON
        buffer_geojson_path = self.path('bounds.geojson')
        with open(buffer_geojson_path, "w") as f:
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
        ds = driver.Open(buffer_geojson_path, 0) # ready-only
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

        if bufferDistance > 0:
            convexhull = convexhull.Buffer(-(bufferDistance + BUFFER_SMOOTH_DISTANCE))
            convexhull = convexhull.Buffer(BUFFER_SMOOTH_DISTANCE)

        # Save to a new file
        buffer_geojson_path = self.path('buffer.geojson')
        if os.path.exists(buffer_geojson_path):
            driver.DeleteDataSource(buffer_geojson_path)

        out_ds = driver.CreateDataSource(buffer_geojson_path)
        layer = out_ds.CreateLayer("convexhull", geom_type=ogr.wkbPolygon)

        feature_def = layer.GetLayerDefn()
        feature = ogr.Feature(feature_def)
        feature.SetGeometry(convexhull)
        layer.CreateFeature(feature)
        feature = None

        # Save and close data sources
        out_ds = ds = None

        return buffer_geojson_path


    def create_buffer_shapefile(self, pointCloudPath, bufferDistance = 0):
        buffer_geojson_path = self.create_buffer_geojson(pointCloudPath, bufferDistance)

        summary_file_path = os.path.join(self.storageDirectory, '{}.summary.json'.format(self.filesPrefix))
        run('pdal info --summary {0} > {1}'.format(pointCloudPath, summary_file_path))
        
        pc_proj4 = None
        with open(summary_file_path, 'r') as f:
            json_f = json.loads(f.read())
            pc_proj4 = json_f['summary']['srs']['proj4']

        if pc_proj4 is None: raise RuntimeError("Could not determine point cloud proj4 declaration")

        bounds_shapefile_path = os.path.join(self.storageDirectory, '{}.buffer.shp'.format(self.filesPrefix))

        # Convert bounds to Shapefile
        kwargs = {
            'input': buffer_geojson_path,
            'output': bounds_shapefile_path,
            'proj4': pc_proj4
        }

        run('ogr2ogr -overwrite -a_srs "{proj4}" {output} {input}'.format(**kwargs))

        return bounds_shapefile_path

