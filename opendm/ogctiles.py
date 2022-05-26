import os
import sys
import shutil
import json
import math
from opendm.utils import double_quote
from opendm import io
from opendm import log
from opendm import system
from opendm.entwine import build_entwine
import fiona
from shapely.geometry import shape

def build_textured_model(input_obj, output_path, reference_lla = None, model_bounds_file=None, rerun=False):
    if not os.path.isfile(input_obj):
        log.ODM_WARNING("No input OBJ file to process")
        return

    if rerun and io.dir_exists(output_path):
        log.ODM_WARNING("Removing previous 3D tiles directory: %s" % output_path)
        shutil.rmtree(output_path)

    log.ODM_INFO("Generating OGC 3D Tiles textured model")
    lat = lon = alt = 0
    
    # Read reference_lla.json (if provided)
    if reference_lla is not None and os.path.isfile(reference_lla):
        try:
            with open(reference_lla) as f:
                reference_lla = json.loads(f.read())
                lat = reference_lla['latitude']
                lon = reference_lla['longitude']
                alt = reference_lla['altitude']
        except Exception as e:
            log.ODM_WARNING("Cannot read %s: %s" % (reference_lla, str(e)))

    # Read model bounds (if provided)
    divisions = 1 # default
    DIV_THRESHOLD = 10000 # m^2 (this is somewhat arbitrary)

    if model_bounds_file is not None and os.path.isfile(model_bounds_file):
        try:
            with fiona.open(model_bounds_file, 'r') as f:
                if len(f) == 1:
                    poly = shape(f[1]['geometry'])
                    area = poly.area
                    log.ODM_INFO("Approximate area: %s m^2" % round(area, 2))

                    if area < DIV_THRESHOLD:
                        divisions = 0
                    else:
                        divisions = math.ceil(math.log((area / DIV_THRESHOLD), 4))
                else:
                    log.ODM_WARNING("Invalid boundary file: %s" % model_bounds_file)
        except Exception as e:
            log.ODM_WARNING("Cannot read %s: %s" % (model_bounds_file, str(e)))

    try:
        kwargs = {
            'input': input_obj,
            'output': output_path,
            'divisions': divisions,
            'lat': lat,
            'lon': lon,
            'alt': alt,
        }
        system.run('Obj2Tiles "{input}" "{output}" --divisions {divisions} '.format(**kwargs))

    except Exception as e:
        log.ODM_WARNING("Cannot build 3D tiles textured model: %s" % str(e))

def build_pointcloud(input_pointcloud, output_path, max_concurrency, rerun=False):
    if not os.path.isfile(input_pointcloud):
        log.ODM_WARNING("No input point cloud file to process")
        return

    if rerun and io.dir_exists(output_path):
        log.ODM_WARNING("Removing previous 3D tiles directory: %s" % output_path)
        shutil.rmtree(output_path)

    log.ODM_INFO("Generating OGC 3D Tiles point cloud")
    
    try:
        if not os.path.isdir(output_path):
            os.mkdir(output_path)

        tmpdir = os.path.join(output_path, "tmp")
        entwine_output = os.path.join(output_path, "entwine")
        
        build_entwine([input_pointcloud], tmpdir, entwine_output, max_concurrency, "EPSG:4978")
        
        kwargs = {
            'input': entwine_output,
            'output': output_path,
        }
        system.run('entwine convert -i "{input}" -o "{output}"'.format(**kwargs))

        for d in [tmpdir, entwine_output]:
            if os.path.isdir(d):
                shutil.rmtree(d)
    except Exception as e:
        log.ODM_WARNING("Cannot build 3D tiles point cloud: %s" % str(e))


def build_3dtiles(args, tree, reconstruction, rerun=False):
    tiles_output_path = tree.ogc_tiles
    model_output_path = os.path.join(tiles_output_path, "model")
    pointcloud_output_path = os.path.join(tiles_output_path, "pointcloud")

    if rerun and os.path.exists(tiles_output_path):
        shutil.rmtree(tiles_output_path)
    
    if not os.path.isdir(tiles_output_path):
        os.mkdir(tiles_output_path)

    # Model 

    if not os.path.isdir(model_output_path) or rerun:
        reference_lla = os.path.join(tree.opensfm, "reference_lla.json")
        model_bounds_file = os.path.join(tree.odm_georeferencing, 'odm_georeferenced_model.bounds.gpkg')

        input_obj = os.path.join(tree.odm_texturing, tree.odm_textured_model_obj)
        if not os.path.isfile(input_obj):
            input_obj = os.path.join(tree.odm_25dtexturing, tree.odm_textured_model_obj)

        build_textured_model(input_obj, model_output_path, reference_lla, model_bounds_file, rerun)
    else:
        log.ODM_WARNING("OGC 3D Tiles model %s already generated" % model_output_path)

    # Point cloud
    
    if not os.path.isdir(pointcloud_output_path) or rerun:
        build_pointcloud(tree.odm_georeferencing_model_laz, pointcloud_output_path, args.max_concurrency, rerun)
    else:
        log.ODM_WARNING("OGC 3D Tiles model %s already generated" % model_output_path)