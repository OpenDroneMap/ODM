import math
import numpy as np
from scipy import ndimage
import rasterio
from rasterio.transform import Affine, rowcol
from opendm import system
from opendm.dem.commands import compute_euclidean_map
from opendm import log
from opendm import io
import os

def euclidean_merge_dems(input_dems, output_dem, creation_options={}, euclidean_map_source=None):
    """
    Based on https://github.com/mapbox/rio-merge-rgba
    and ideas from Anna Petrasova
    implementation by Piero Toffanin

    Computes a merged DEM by computing/using a euclidean 
    distance to NODATA cells map for all DEMs and then blending all overlapping DEM cells 
    by a weighted average based on such euclidean distance.
    """
    inputs = []
    bounds=None
    precision=7

    existing_dems = []
    for dem in input_dems:
        if not io.file_exists(dem):
            log.ODM_WARNING("%s does not exist. Will skip from merged DEM." % dem)
            continue
        existing_dems.append(dem)

    if len(existing_dems) == 0:
        log.ODM_WARNING("No input DEMs, skipping euclidean merge.")
        return

    with rasterio.open(existing_dems[0]) as first:
        src_nodata = first.nodatavals[0]
        res = first.res
        dtype = first.dtypes[0]
        profile = first.profile

    for dem in existing_dems:
        eumap = compute_euclidean_map(dem, io.related_file_path(dem, postfix=".euclideand", replace_base=euclidean_map_source), overwrite=False)
        if eumap and io.file_exists(eumap):
            inputs.append((dem, eumap))

    log.ODM_INFO("%s valid DEM rasters to merge" % len(inputs))

    sources = [(rasterio.open(d), rasterio.open(e)) for d,e in inputs]

    # Extent from option or extent of all inputs.
    if bounds:
        dst_w, dst_s, dst_e, dst_n = bounds
    else:
        # scan input files.
        # while we're at it, validate assumptions about inputs
        xs = []
        ys = []
        for src_d, src_e in sources:
            left, bottom, right, top = src_d.bounds
            xs.extend([left, right])
            ys.extend([bottom, top])
            if src_d.profile["count"] != 1 or src_e.profile["count"] != 1:
                raise ValueError("Inputs must be 1-band rasters")
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
    profile["tiled"] = creation_options.get('TILED', 'YES') == 'YES'
    profile["blockxsize"] = creation_options.get('BLOCKXSIZE', 512)
    profile["blockysize"] = creation_options.get('BLOCKYSIZE', 512)
    profile["compress"] = creation_options.get('COMPRESS', 'LZW')
    profile["nodata"] = src_nodata

    # Creation opts
    profile.update(creation_options)

    # create destination file
    with rasterio.open(output_dem, "w", **profile) as dstrast:

        for idx, dst_window in dstrast.block_windows():

            left, bottom, right, top = dstrast.window_bounds(dst_window)

            blocksize = dst_window.width
            dst_rows, dst_cols = (dst_window.height, dst_window.width)

            # initialize array destined for the block
            dst_count = first.count
            dst_shape = (dst_count, dst_rows, dst_cols)

            dstarr = np.zeros(dst_shape, dtype=dtype)
            distsum = np.zeros(dst_shape, dtype=dtype)
            small_distance = 0.001953125

            for src_d, src_e in sources:
                # The full_cover behavior is problematic here as it includes
                # extra pixels along the bottom right when the sources are
                # slightly misaligned
                #
                # src_window = get_window(left, bottom, right, top,
                #                         src.transform, precision=precision)
                #
                # With rio merge this just adds an extra row, but when the
                # imprecision occurs at each block, you get artifacts

                nodata = src_d.nodatavals[0]

                # Alternative, custom get_window using rounding
                src_window_d = tuple(zip(rowcol(
                        src_d.transform, left, top, op=round, precision=precision
                    ), rowcol(
                        src_d.transform, right, bottom, op=round, precision=precision
                    )))

                src_window_e = tuple(zip(rowcol(
                        src_e.transform, left, top, op=round, precision=precision
                    ), rowcol(
                        src_e.transform, right, bottom, op=round, precision=precision
                    )))

                temp_d = np.zeros(dst_shape, dtype=dtype)
                temp_d = src_d.read(
                    out=temp_d, window=src_window_d, boundless=True, masked=False
                )

                temp_e = np.zeros(dst_shape, dtype=dtype)
                temp_e = src_e.read(
                    out=temp_e, window=src_window_e, boundless=True, masked=False
                )

                # Set NODATA areas in the euclidean map to a very low value
                # so that:
                #  - Areas with overlap prioritize DEM layers' cells that 
                #    are far away from NODATA areas
                #  - Areas that have no overlap are included in the final result
                #    even if they are very close to a NODATA cell
                temp_e[temp_e==0] = small_distance
                temp_e[temp_d==nodata] = 0

                np.multiply(temp_d, temp_e, out=temp_d)
                np.add(dstarr, temp_d, out=dstarr)
                np.add(distsum, temp_e, out=distsum)

            np.divide(dstarr, distsum, out=dstarr, where=distsum[0] != 0.0)

            # Perform nearest neighbor interpolation on areas where two or more rasters overlap
            # but where both rasters have only interpolated data. This prevents the creation
            # of artifacts that average areas of interpolation.
            indices = ndimage.distance_transform_edt(np.logical_and(distsum < 1, distsum > small_distance), 
                                                return_distances=False, 
                                                return_indices=True)
            dstarr = dstarr[tuple(indices)]

            dstarr[dstarr == 0.0] = src_nodata

            dstrast.write(dstarr, window=dst_window)

    return output_dem
