import fiona
import fiona.crs
import os
import io
import json
from opendm import system
from pyproj import CRS
from opendm.location import transformer
from opendm.utils import double_quote

def load_boundary(boundary_json, reproject_to_proj4=None):
    with fiona.open(io.BytesIO(json.dumps(boundary_json).encode('utf-8')), 'r') as src:
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

def as_polygon(boundary):
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

