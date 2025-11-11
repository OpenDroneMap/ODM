import argparse
import sys
sys.path.append("../../")

import os
from opendm import orthophoto
from opendm.cutline import compute_cutline
import glob
from opendm.system import run
from opendm import log
import shutil


parser = argparse.ArgumentParser(description='Quick Merge Preview')
parser.add_argument('input',
                    metavar='<paths>',
                    nargs='+',
                    help='Path to input images or image folder')
parser.add_argument('--size', '-s',
                    metavar='<percentage>',
                    type=str,
                    help='Size in percentage terms',
                    default='25%')
parser.add_argument('--force', '-f', 
                        action='store_true',
                        default=False, 
                        help="Force remove existing directories")

args = parser.parse_args()

try:
    log.ODM_INFO("Checking for DDB...")
    run("ddb --version")
except:
    log.ODM_ERROR("ddb is not installed. Install it first: https://docs.dronedb.app")

if len(args.input) == 1 and os.path.isdir(args.input[0]):
    input_images = []
    for ext in ["JPG", "JPEG", "TIF", "tiff", "tif", "TIFF"]:
        input_images += glob.glob(os.path.join(args.input[0], "*.%s" % ext))
else:
    input_images = args.input

log.ODM_INFO("Processing %s images" % len(input_images))

if len(input_images) == 0:
    log.ODM_ERROR("No images")
    exit(1)

cwd_path = os.path.dirname(input_images[0])
tmp_path = os.path.join(cwd_path, "tmp")
if os.path.isdir(tmp_path):
    if args.force:
        log.ODM_INFO("Removing previous directory %s" % tmp_path)
        shutil.rmtree(tmp_path)
    else:
        log.ODM_ERROR("%s exists. Pass --force to override." % tmp_path)
        exit(1)

os.makedirs(tmp_path)

for f in input_images:
    name, _ = os.path.splitext(os.path.basename(f))
    geojson = os.path.join(tmp_path, "%s.geojson" % name)
    gpkg = os.path.join(tmp_path, "%s.gpkg" % name)

    run("ddb geoproj \"%s\" \"%s\" -s \"%s\"" % (tmp_path, f, args.size))

    # Bounds (GPKG)
    run("ddb info --format geojson --geometry polygon \"%s\" > \"%s\"" % (f, geojson))
    run("ogr2ogr \"%s\" \"%s\"" % (gpkg, geojson))

log.ODM_INFO("Computing cutlines")

projected_images = glob.glob(os.path.join(tmp_path, "*.tif"))
all_orthos_and_ortho_cuts = []

for f in projected_images:
    name, _ = os.path.splitext(os.path.basename(f))
    cutline_file = os.path.join(tmp_path, "%s_cutline.gpkg" % name)
    bounds_file_path = os.path.join(tmp_path, "%s.gpkg" % name)
    
    compute_cutline(f, 
                    bounds_file_path,
                    cutline_file,
                    4,
                    scale=1)

    cut_raster = os.path.join(tmp_path, "%s_cut.tif" % name)
    orthophoto.compute_mask_raster(f, cutline_file, 
                            cut_raster,
                            blend_distance=20, only_max_coords_feature=True)

    feathered_raster = os.path.join(tmp_path, "%s_feathered.tif" % name)

    orthophoto.feather_raster(f, feathered_raster,
                            blend_distance=20
                        )

    all_orthos_and_ortho_cuts.append([feathered_raster, cut_raster])

log.ODM_INFO("Merging...")

if len(all_orthos_and_ortho_cuts) > 1:
    # TODO: histogram matching via rasterio
    # currently parts have different color tones
    output_file = os.path.join(cwd_path, 'mergepreview.tif')

    if os.path.isfile(output_file):
        os.remove(output_file)

    orthophoto.merge(all_orthos_and_ortho_cuts, output_file, {
        'TILED': 'YES',
        'COMPRESS': 'LZW',
        'PREDICTOR': '2',
        'BIGTIFF': 'IF_SAFER',
        'BLOCKXSIZE': 512,
        'BLOCKYSIZE': 512
    }, args.merge_skip_blending)


    log.ODM_INFO("Wrote %s" % output_file)
    shutil.rmtree(tmp_path)
else:
    log.ODM_ERROR("Error: no orthos found to merge")
    exit(1)