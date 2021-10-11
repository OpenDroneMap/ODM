import fiona
import os

def load_boundary(boundary_json, reproject_to_crs=None):
    if not os.path.isfile(boundary_json):
        raise IOError("Cannot load boundary file: %s does not exist" % boundary_json)
    
    with fiona.open(boundary_json, 'r') as src:
        if len(src) != 1:
            raise IOError("Boundary file must have a single polygon (found: %s)" % len(src))
        
        geom = src[0]['geometry']

        if geom['type'] != 'Polygon':
            raise IOError("Boundary file must have a polygon feature (found: %s)" % geom['type'])

        coords = geom['coordinates']

        if reproject_to_crs is not None:
            pass
        
        return coords