import os
from opendm import log
from opendm import system
from opendm.cropper import Cropper
from opendm.concurrency import get_max_memory
import math
import numpy as np
import rasterio
from rasterio.transform import Affine, rowcol
from opendm import io

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
    system.run('gdaladdo -ro -r average '
                '--config BIGTIFF_OVERVIEW IF_SAFER '
                '--config COMPRESS_OVERVIEW JPEG '
                '{orthophoto} 2 4 8 16'.format(**kwargs))

def generate_png(orthophoto_file):
    log.ODM_INFO("Generating PNG")
    base, ext = os.path.splitext(orthophoto_file)
    orthophoto_png = base + '.png'

    system.run('gdal_translate -of png "%s" "%s" '
               '--config GDAL_CACHEMAX %s%% ' % (orthophoto_file, orthophoto_png, get_max_memory()))


def post_orthophoto_steps(args, bounds_file_path, orthophoto_file):
    if args.crop > 0:
        Cropper.crop(bounds_file_path, orthophoto_file, get_orthophoto_vars(args), warp_options=['-dstalpha'])

    if args.build_overviews:
        build_overviews(orthophoto_file)

    if args.orthophoto_png:
        generate_png(orthophoto_file)

def merge(input_ortho_and_cutlines, output_orthophoto, blend_distance=20, orthophoto_vars={}):
    """
    Based on https://github.com/mapbox/rio-merge-rgba/
    Merge orthophotos around cutlines using a blend buffer.
    """
    inputs = []
    bounds=None
    precision=7

    for o, c in input_ortho_and_cutlines:
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
        src_nodata = first.nodatavals[0] # TODO: this is None
        res = first.res
        dtype = first.dtypes[0]
        profile = first.profile

    log.ODM_INFO("%s valid orthophoto rasters to merge" % len(inputs))
    sources = [(rasterio.open(o)) for o,_ in inputs]

    # scan input files.
    # while we're at it, validate assumptions about inputs
    xs = []
    ys = []
    for src in sources:
        left, bottom, right, top = src.bounds
        xs.extend([left, right])
        ys.extend([bottom, top])
        # if src.profile["count"] != 1 or src.profile["count"] != 1:
        #     raise ValueError("Inputs must be 1-band rasters")
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
    profile["nodata"] = src_nodata
    profile.update()

    # create destination file
    with rasterio.open(output_orthophoto, "w", **profile) as dstrast:
        for idx, dst_window in dstrast.block_windows():
            left, bottom, right, top = dstrast.window_bounds(dst_window)

            blocksize = dst_window.width
            dst_rows, dst_cols = (dst_window.height, dst_window.width)

            # initialize array destined for the block
            dst_count = first.count
            dst_shape = (dst_count, dst_rows, dst_cols)

            dstarr = np.zeros(dst_shape, dtype=dtype)
            distsum = np.zeros(dst_shape, dtype=dtype)

            for src in sources:
                # The full_cover behavior is problematic here as it includes
                # extra pixels along the bottom right when the sources are
                # slightly misaligned
                #
                # src_window = get_window(left, bottom, right, top,
                #                         src.transform, precision=precision)
                #
                # With rio merge this just adds an extra row, but when the
                # imprecision occurs at each block, you get artifacts

                nodata = src.nodatavals[0]

                # Alternative, custom get_window using rounding
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
                    (dstarr[3] == 0), (temp[3] != 0)  # 0 is nodata
                )
                np.copyto(dstarr, temp, where=write_region)

                # check if dest has any nodata pixels available
                if np.count_nonzero(dstarr[3]) == blocksize:
                    break

            dstrast.write(dstarr, window=dst_window)

    return output_orthophoto
