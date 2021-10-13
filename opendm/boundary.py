import fiona
import fiona.crs
import os
import io
import json
from opendm import system
from pyproj import CRS
from opendm.location import transformer
from opendm.utils import double_quote
from osgeo import ogr
from opendm.shots import get_origin

def compute_boundary_from_shots(reconstruction_json, buffer=0, reconstruction_offset=(0, 0)):
    if not os.path.isfile(reconstruction_json):
        raise IOError(reconstruction_json + " does not exist.")

    with open(reconstruction_json) as f:
        data = json.load(f)
    reconstruction = data[0]

    mp = ogr.Geometry(ogr.wkbMultiPoint)

    for shot_image in reconstruction['shots']:
        shot = reconstruction['shots'][shot_image]
        if shot['gps_dop'] < 999999:
            camera = reconstruction['cameras'][shot['camera']]

            p = ogr.Geometry(ogr.wkbPoint)
            origin = get_origin(shot)

            p.AddPoint_2D(origin[0] + reconstruction_offset[0], origin[1] + reconstruction_offset[1])
            mp.AddGeometry(p)

    if mp.GetGeometryCount() < 3:
        return None

    convexhull = mp.ConvexHull()
    boundary = convexhull.Buffer(buffer)

    return load_boundary(boundary.ExportToJson())

def load_boundary(boundary_json, reproject_to_proj4=None):
    if not isinstance(boundary_json, str):
        boundary_json = json.dumps(boundary_json)

    with fiona.open(io.BytesIO(boundary_json.encode('utf-8')), 'r') as src:
        if len(src) != 1:
            raise IOError("Boundary must have a single polygon (found: %s)" % len(src))
        
        geom = src[0]['geometry']

        if geom['type'] != 'Polygon':
            raise IOError("Boundary must have a polygon feature (found: %s)" % geom['type'])

        rings = geom['coordinates']

        if len(rings) == 0:
            raise IOError("Boundary geometry has no rings")
        
        coords = rings[0]
        if len(coords) == 0:
            raise IOError("Boundary geometry has no coordinates")

        dimensions = len(coords[0])

        if reproject_to_proj4 is not None:
            t = transformer(CRS.from_proj4(fiona.crs.to_string(src.crs)),
                            CRS.from_proj4(reproject_to_proj4))
            coords = [t.TransformPoint(*c)[:dimensions] for c in coords]
        
        return coords

def boundary_offset(boundary, reconstruction_offset):
    if boundary is None or reconstruction_offset is None:
        return boundary
    
    res = []
    dims = len(boundary[0])
    for c in boundary:
        if dims == 2:
            res.append((c[0] - reconstruction_offset[0], c[1] - reconstruction_offset[1]))
        else:
            res.append((c[0] - reconstruction_offset[0], c[1] - reconstruction_offset[1], c[2]))
    
    return res

def as_polygon(boundary):
    if boundary is None:
        return None

    return "POLYGON((" + ", ".join([" ".join(map(str, c)) for c in boundary]) + "))"

def export_to_bounds_files(boundary, proj4, bounds_json_file, bounds_gpkg_file):
    with open(bounds_json_file, "w") as f:
        f.write(json.dumps({
            "type": "FeatureCollection",
            "name": "bounds",
            "features": [{
                "type": "Feature",
                "properties": {},
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [boundary]
                }
            }]
        }))
    
    if os.path.isfile(bounds_gpkg_file):
        os.remove(bounds_gpkg_file)
    
    kwargs = {
        'proj4': proj4,
        'input': double_quote(bounds_json_file),
        'output': double_quote(bounds_gpkg_file)
    }

    system.run('ogr2ogr -overwrite -f GPKG -a_srs "{proj4}" {output} {input}'.format(**kwargs))

