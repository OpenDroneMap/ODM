import os, json
from opendm import log
from opendm.pseudogeo import get_pseudogeo_utm, get_pseudogeo_scale
from opendm.location import transformer
from pyproj import CRS
import gdal
import numpy as np
import cv2

def get_rotation_matrix(rotation):
    """Get rotation as a 3x3 matrix."""
    return cv2.Rodrigues(rotation)[0]

def matrix_to_rotation(rotation_matrix):
    R = np.array(rotation_matrix, dtype=float)
    # if not np.isclose(np.linalg.det(R), 1):
    #     raise ValueError("Determinant != 1")
    # if not np.allclose(np.linalg.inv(R), R.T):
    #     raise ValueError("Not orthogonal")
    return cv2.Rodrigues(R)[0].ravel()

def get_origin(shot):
    """The origin of the pose in world coordinates."""
    return -get_rotation_matrix(np.array(shot['rotation'])).T.dot(np.array(shot['translation']))

def get_geojson_shots_from_opensfm(reconstruction_file, geocoords_transformation_file=None, utm_srs=None, pseudo_geotiff=None):
    """
    Extract shots from OpenSfM's reconstruction.json
    """

    # Read transform (if available)
    if geocoords_transformation_file is not None and utm_srs is not None and os.path.exists(geocoords_transformation_file):
        geocoords = np.loadtxt(geocoords_transformation_file, usecols=range(4))
        pseudo = False
    elif pseudo_geotiff is not None and os.path.exists(pseudo_geotiff):
        # pseudogeo transform
        utm_srs = get_pseudogeo_utm()

        # the pseudo-georeferencing CRS UL corner is at 0,0
        # but our shot coordinates aren't, so we need to offset them
        raster = gdal.Open(pseudo_geotiff)
        ulx, xres, _, uly, _, yres  = raster.GetGeoTransform()
        lrx = ulx + (raster.RasterXSize * xres)
        lry = uly + (raster.RasterYSize * yres)

        geocoords = np.array([[1.0 / get_pseudogeo_scale() ** 2, 0, 0, ulx + lrx / 2.0],
                              [0, 1.0 / get_pseudogeo_scale() ** 2, 0, uly + lry / 2.0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])
        raster = None
        pseudo = True
    else:
        # Can't deal with this
        return

    crstrans = transformer(CRS.from_proj4(utm_srs), CRS.from_epsg("4326"))

    if os.path.exists(reconstruction_file):
        with open(reconstruction_file, 'r') as fin:
            reconstructions = json.loads(fin.read())
            
            feats = []
            cameras = {}
            added_shots = {}
            for recon in reconstructions:
                if 'cameras' in recon:
                    cameras = recon['cameras']
                
            for filename in recon.get('shots', {}):
                shot = recon['shots'][filename]
                cam = shot.get('camera')
                if (not cam in cameras) or (filename in added_shots):
                    continue
                
                cam = cameras[cam]
                Rs, T = geocoords[:3, :3], geocoords[:3, 3]
                Rs1 = np.linalg.inv(Rs)
                origin = get_origin(shot)

                # Translation
                utm_coords = np.dot(Rs, origin) + T
                trans_coords = crstrans.TransformPoint(utm_coords[0], utm_coords[1], utm_coords[2])

                # Rotation
                rotation_matrix = get_rotation_matrix(np.array(shot['rotation']))
                rotation = matrix_to_rotation(np.dot(rotation_matrix, Rs1))

                translation = origin if pseudo else utm_coords

                feats.append({
                    'type': 'Feature',
                    'properties': {
                        'filename': filename,
                        'focal': cam.get('focal', cam.get('focal_x')), # Focal ratio = focal length (mm) / max(sensor_width, sensor_height) (mm)
                        'width': cam.get('width', 0),
                        'height': cam.get('height', 0),
                        'translation': list(translation),
                        'rotation': list(rotation)
                    },
                    'geometry':{
                        'type': 'Point',
                        'coordinates': list(trans_coords)
                    }
                })

                added_shots[filename] = True

        return {
            'type': 'FeatureCollection',
            'features': feats
        }
    else:
        raise RuntimeError("%s does not exist." % reconstruction_file)

def merge_geojson_shots(geojson_shots_files, output_geojson_file):
    result = {}
    added_files = {}
    for shot_file in geojson_shots_files:
        with open(shot_file, "r") as f:
            shots = json.loads(f.read())

        if len(result) == 0:
            for feat in shots.get('features', []):
                added_files[feat['properties']['filename']] = True

            # Use first file as base
            result = shots
        else:
            # Append features if filename not already added
            for feat in shots.get('features', []):
                if not feat['properties']['filename'] in added_files:
                    result['features'].append(feat)
    
    with open(output_geojson_file, "w") as f:
        f.write(json.dumps(result))
