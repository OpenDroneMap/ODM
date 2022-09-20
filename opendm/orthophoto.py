import os
from opendm import log
from opendm import system
from opendm.cropper import Cropper
from opendm.concurrency import get_max_memory
import math
import numpy as np
import rasterio
import fiona
from edt import edt
from rasterio.transform import Affine, rowcol
from rasterio.mask import mask
from opendm import io
from opendm.tiles.tiler import generate_orthophoto_tiles
from opendm.cogeo import convert_to_cogeo
from osgeo import gdal


def get_orthophoto_vars(args):
    return {
        'TILED': 'NO' if args.orthophoto_no_tiled else 'YES',
        'COMPRESS': args.orthophoto_compression,
        'PREDICTOR': '2' if args.orthophoto_compression in ['LZW', 'DEFLATE'] else '1',
        'BIGTIFF': 'IF_SAFER',
        'BLOCKXSIZE': 512,
        'BLOCKYSIZE': 512,
        'NUM_THREADS': args.max_concurrency
    }

def build_overviews(orthophoto_file):
    log.ODM_INFO("Building Overviews")
    kwargs = {'orthophoto': orthophoto_file}
    
    # Run gdaladdo
    system.run('gdaladdo -r average '
                '--config BIGTIFF_OVERVIEW IF_SAFER '
                '--config COMPRESS_OVERVIEW JPEG '
                '{orthophoto} 2 4 8 16'.format(**kwargs))

def generate_png(orthophoto_file, output_file=None, outsize=None):
    if output_file is None:
        base, ext = os.path.splitext(orthophoto_file)
        output_file = base + '.png'
    
    # See if we need to select top three bands
    bandparam = ""

    gtif = gdal.Open(orthophoto_file)
    if gtif.RasterCount > 4:
        bands = []
        for idx in range(1, gtif.RasterCount+1):
            bands.append(gtif.GetRasterBand(idx).GetColorInterpretation())
        bands = dict(zip(bands, range(1, len(bands)+1)))

        try:
            red = bands.get(gdal.GCI_RedBand)
            green = bands.get(gdal.GCI_GreenBand)
            blue = bands.get(gdal.GCI_BlueBand)
            if red is None or green is None or blue is None:
                raise Exception("Cannot find bands")

            bandparam = "-b %s -b %s -b %s -a_nodata 0" % (red, green, blue)
        except:
            bandparam = "-b 1 -b 2 -b 3 -a_nodata 0"
    gtif = None

    osparam = ""
    if outsize is not None:
        osparam = "-outsize %s 0" % outsize

    system.run('gdal_translate -of png "%s" "%s" %s %s '
               '--config GDAL_CACHEMAX %s%% ' % (orthophoto_file, output_file, osparam, bandparam, get_max_memory()))

def generate_kmz(orthophoto_file, output_file=None, outsize=None):
    if output_file is None:
        base, ext = os.path.splitext(orthophoto_file)
        output_file = base + '.kmz'
    
    # See if we need to select top three bands
    bandparam = ""
    gtif = gdal.Open(orthophoto_file)
    if gtif.RasterCount > 4:
        bandparam = "-b 1 -b 2 -b 3 -a_nodata 0"

    system.run('gdal_translate -of KMLSUPEROVERLAY -co FORMAT=PNG "%s" "%s" %s '
               '--config GDAL_CACHEMAX %s%% ' % (orthophoto_file, output_file, bandparam, get_max_memory()))    
    
def post_orthophoto_steps(args, bounds_file_path, orthophoto_file, orthophoto_tiles_dir):
    if args.crop > 0 or args.boundary:
        Cropper.crop(bounds_file_path, orthophoto_file, get_orthophoto_vars(args), keep_original=not args.optimize_disk_space, warp_options=['-dstalpha'])

    if args.build_overviews and not args.cog:
        build_overviews(orthophoto_file)

    if args.orthophoto_png:
        generate_png(orthophoto_file)
        
    if args.orthophoto_kmz:
        generate_kmz(orthophoto_file)

    if args.tiles:
        generate_orthophoto_tiles(orthophoto_file, orthophoto_tiles_dir, args.max_concurrency)

    if args.cog:
        convert_to_cogeo(orthophoto_file, max_workers=args.max_concurrency, compression=args.orthophoto_compression)

def compute_mask_raster(input_raster, vector_mask, output_raster, blend_distance=20, only_max_coords_feature=False):
    if not os.path.exists(input_raster):
        log.ODM_WARNING("Cannot mask raster, %s does not exist" % input_raster)
        return
    
    if not os.path.exists(vector_mask):
        log.ODM_WARNING("Cannot mask raster, %s does not exist" % vector_mask)
        return

    log.ODM_INFO("Computing mask raster: %s" % output_raster)

    with rasterio.open(input_raster, 'r') as rast:
        with fiona.open(vector_mask) as src:
            burn_features = src

            if only_max_coords_feature:
                max_coords_count = 0
                max_coords_feature = None
                for feature in src:
                    if feature is not None:
                        # No complex shapes
                        if len(feature['geometry']['coordinates'][0]) > max_coords_count:
                            max_coords_count = len(feature['geometry']['coordinates'][0])
                            max_coords_feature = feature
                if max_coords_feature is not None:
                    burn_features = [max_coords_feature]
            
            shapes = [feature["geometry"] for feature in burn_features]
            out_image, out_transform = mask(rast, shapes, nodata=0)

            if blend_distance > 0:
                if out_image.shape[0] >= 4:
                    # alpha_band = rast.dataset_mask()
                    alpha_band = out_image[-1]
                    dist_t = edt(alpha_band, black_border=True, parallel=0)
                    dist_t[dist_t <= blend_distance] /= blend_distance
                    dist_t[dist_t > blend_distance] = 1
                    np.multiply(alpha_band, dist_t, out=alpha_band, casting="unsafe")
                else:
                    log.ODM_WARNING("%s does not have an alpha band, cannot blend cutline!" % input_raster)

            with rasterio.open(output_raster, 'w', BIGTIFF="IF_SAFER", **rast.profile) as dst:
                dst.colorinterp = rast.colorinterp
                dst.write(out_image)

            return output_raster

def feather_raster(input_raster, output_raster, blend_distance=20):
    if not os.path.exists(input_raster):
        log.ODM_WARNING("Cannot feather raster, %s does not exist" % input_raster)
        return

    log.ODM_INFO("Computing feather raster: %s" % output_raster)
    
    with rasterio.open(input_raster, 'r') as rast:
        out_image = rast.read()
        if blend_distance > 0:
            if out_image.shape[0] >= 4:
                alpha_band = out_image[-1]
                dist_t = edt(alpha_band, black_border=True, parallel=0)
                dist_t[dist_t <= blend_distance] /= blend_distance
                dist_t[dist_t > blend_distance] = 1
                np.multiply(alpha_band, dist_t, out=alpha_band, casting="unsafe")
            else:
                log.ODM_WARNING("%s does not have an alpha band, cannot feather raster!" % input_raster)

        with rasterio.open(output_raster, 'w', BIGTIFF="IF_SAFER", **rast.profile) as dst:
            dst.colorinterp = rast.colorinterp
            dst.write(out_image)

        return output_raster

def merge(input_ortho_and_ortho_cuts, output_orthophoto, orthophoto_vars={}):
    """
    Based on https://github.com/mapbox/rio-merge-rgba/
    Merge orthophotos around cutlines using a blend buffer.
    """
    inputs = []
    bounds=None
    precision=7

    for o, c in input_ortho_and_ortho_cuts:
        if not io.file_exists(o):
            log.ODM_WARNING("%s does not exist. Will skip from merged orthophoto." % o)
            continue
        if not io.file_exists(c):
            log.ODM_WARNING("%s does not exist. Will skip from merged orthophoto." % c)
            continue
        inputs.append((o, c))

    if len(inputs) == 0:
        log.ODM_WARNING("No input orthophotos, skipping merge.")
        return

    with rasterio.open(inputs[0][0]) as first:
        res = first.res
        dtype = first.dtypes[0]
        profile = first.profile
        num_bands = first.meta['count'] - 1 # minus alpha
        colorinterp = first.colorinterp

    log.ODM_INFO("%s valid orthophoto rasters to merge" % len(inputs))
    sources = [(rasterio.open(o), rasterio.open(c)) for o,c in inputs]

    # scan input files.
    # while we're at it, validate assumptions about inputs
    xs = []
    ys = []
    for src, _ in sources:
        left, bottom, right, top = src.bounds
        xs.extend([left, right])
        ys.extend([bottom, top])
        if src.profile["count"] < 4:
            raise ValueError("Inputs must be at least 4-band rasters")
    dst_w, dst_s, dst_e, dst_n = min(xs), min(ys), max(xs), max(ys)
    log.ODM_INFO("Output bounds: %r %r %r %r" % (dst_w, dst_s, dst_e, dst_n))

    output_transform = Affine.translation(dst_w, dst_n)
    output_transform *= Affine.scale(res[0], -res[1])

    # Compute output array shape. We guarantee it will cover the output
    # bounds completely.
    output_width = int(math.ceil((dst_e - dst_w) / res[0]))
    output_height = int(math.ceil((dst_n - dst_s) / res[1]))

    # Adjust bounds to fit.
    dst_e, dst_s = output_transform * (output_width, output_height)
    log.ODM_INFO("Output width: %d, height: %d" % (output_width, output_height))
    log.ODM_INFO("Adjusted bounds: %r %r %r %r" % (dst_w, dst_s, dst_e, dst_n))

    profile["transform"] = output_transform
    profile["height"] = output_height
    profile["width"] = output_width
    profile["tiled"] = orthophoto_vars.get('TILED', 'YES') == 'YES'
    profile["blockxsize"] = orthophoto_vars.get('BLOCKXSIZE', 512)
    profile["blockysize"] = orthophoto_vars.get('BLOCKYSIZE', 512)
    profile["compress"] = orthophoto_vars.get('COMPRESS', 'LZW')
    profile["predictor"] = orthophoto_vars.get('PREDICTOR', '2')
    profile["bigtiff"] = orthophoto_vars.get('BIGTIFF', 'IF_SAFER')
    profile.update()

    # create destination file
    with rasterio.open(output_orthophoto, "w", **profile) as dstrast:
        dstrast.colorinterp = colorinterp
        for idx, dst_window in dstrast.block_windows():
            left, bottom, right, top = dstrast.window_bounds(dst_window)

            blocksize = dst_window.width
            dst_rows, dst_cols = (dst_window.height, dst_window.width)

            # initialize array destined for the block
            dst_count = first.count
            dst_shape = (dst_count, dst_rows, dst_cols)

            dstarr = np.zeros(dst_shape, dtype=dtype)

            # First pass, write all rasters naively without blending
            for src, _ in sources:
                src_window = tuple(zip(rowcol(
                        src.transform, left, top, op=round, precision=precision
                    ), rowcol(
                        src.transform, right, bottom, op=round, precision=precision
                    )))

                temp = np.zeros(dst_shape, dtype=dtype)
                temp = src.read(
                    out=temp, window=src_window, boundless=True, masked=False
                )

                # pixels without data yet are available to write
                write_region = np.logical_and(
                    (dstarr[-1] == 0), (temp[-1] != 0)  # 0 is nodata
                )
                np.copyto(dstarr, temp, where=write_region)

                # check if dest has any nodata pixels available
                if np.count_nonzero(dstarr[-1]) == blocksize:
                    break

            # Second pass, write all feathered rasters
            # blending the edges
            for src, _ in sources:
                src_window = tuple(zip(rowcol(
                        src.transform, left, top, op=round, precision=precision
                    ), rowcol(
                        src.transform, right, bottom, op=round, precision=precision
                    )))

                temp = np.zeros(dst_shape, dtype=dtype)
                temp = src.read(
                    out=temp, window=src_window, boundless=True, masked=False
                )

                where = temp[-1] != 0
                for b in range(0, num_bands):
                    blended = temp[-1] / 255.0 * temp[b] + (1 - temp[-1] / 255.0) * dstarr[b]
                    np.copyto(dstarr[b], blended, casting='unsafe', where=where)
                dstarr[-1][where] = 255.0
                
                # check if dest has any nodata pixels available
                if np.count_nonzero(dstarr[-1]) == blocksize:
                    break

            # Third pass, write cut rasters
            # blending the cutlines
            for _, cut in sources:
                src_window = tuple(zip(rowcol(
                        cut.transform, left, top, op=round, precision=precision
                    ), rowcol(
                        cut.transform, right, bottom, op=round, precision=precision
                    )))

                temp = np.zeros(dst_shape, dtype=dtype)
                temp = cut.read(
                    out=temp, window=src_window, boundless=True, masked=False
                )

                # For each band, average alpha values between
                # destination raster and cut raster
                for b in range(0, num_bands):
                    blended = temp[-1] / 255.0 * temp[b] + (1 - temp[-1] / 255.0) * dstarr[b]
                    np.copyto(dstarr[b], blended, casting='unsafe', where=temp[-1]!=0)

            dstrast.write(dstarr, window=dst_window)

    return output_orthophoto
