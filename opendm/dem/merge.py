import math
import numpy as np
import rasterio
from rasterio.transform import Affine, rowcol
from opendm import system
from opendm import log
from opendm import io
import os

from rasterio import logging

def euclidean_merge_dems(input_dems, output_dem, creation_options={}):
    """
    Based on https://github.com/mapbox/rio-merge-rgba
    and ideas from Anna Petrasova
    implementation by Piero Toffanin

    Computes a merged DEM by computing a euclidean distance map for all DEMs 
    to all NODATA cells (how far from the edge of the DEM cells are) and then blending
    all overlapping DEM cells by a weighted average based on such euclidean distance.
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

    # Suppress DEBUG logging for rasterio operations
    log_ = logging.getLogger()
    debug_enabled = log_.isEnabledFor(logging.DEBUG)
    if debug_enabled:
        logging.disable(logging.DEBUG)

    with rasterio.open(existing_dems[0]) as first:
        src_nodata = 0
        nodatavals = first.get_nodatavals()
        if len(nodatavals) > 0:
            src_nodata = nodatavals[0]
        res = first.res
        dtype = first.dtypes[0]
        profile = first.profile

    for dem in existing_dems:
        # Compute euclidean distance support files
        path, filename = os.path.split(dem)
        # path = path/to
        # filename = dsm.tif

        basename, ext = os.path.splitext(filename)
        # basename = dsm
        # ext = .tif

        euclidean_geotiff = os.path.join(path, "{}.euclideand{}".format(basename, ext))
        log.ODM_INFO("Computing euclidean distance: %s" % euclidean_geotiff)
        system.run('gdal_proximity.py "%s" "%s" -values -9999' % (dem, euclidean_geotiff))

        if io.file_exists(euclidean_geotiff):
            inputs.append((dem, euclidean_geotiff))
        else:
            log.ODM_WARNING("Cannot compute euclidean distance file: %s" % euclidean_geotiff)

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
            if src_d.bounds != src_e.bounds:
                raise ValueError("DEM and euclidean file must have the same bounds")

            left, bottom, right, top = src_d.bounds
            xs.extend([left, right])
            ys.extend([bottom, top])
            if src_d.profile["count"] != 1 or src_e.profile["count"] != 1:
                raise ValueError("Inputs must be 1-band rasters")
        dst_w, dst_s, dst_e, dst_n = min(xs), min(ys), max(xs), max(ys)
    log.ODM_INFO("Output bounds: %r", (dst_w, dst_s, dst_e, dst_n))
    output_transform = Affine.translation(dst_w, dst_n)
    log.ODM_INFO("Output transform, before scaling: %r", output_transform)

    output_transform *= Affine.scale(res[0], -res[1])
    log.ODM_INFO("Output transform, after scaling: %r", output_transform)

    # Compute output array shape. We guarantee it will cover the output
    # bounds completely.
    output_width = int(math.ceil((dst_e - dst_w) / res[0]))
    output_height = int(math.ceil((dst_n - dst_s) / res[1]))

    # Adjust bounds to fit.
    dst_e, dst_s = output_transform * (output_width, output_height)
    log.ODM_INFO("Output width: %d, height: %d", output_width, output_height)
    log.ODM_INFO("Adjusted bounds: %r", (dst_w, dst_s, dst_e, dst_n))

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
            log.ODM_DEBUG("Temp shape: %r", dst_shape)

            dstarr = np.zeros(dst_shape, dtype=dtype)
            distsum = np.zeros(dst_shape, dtype=dtype)

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

                nodata = 0
                nodatavals = src_d.get_nodatavals()
                if len(nodatavals) > 0:
                    nodata = nodatavals[0]

                # Alternative, custom get_window using rounding
                src_window = tuple(zip(rowcol(
                        src_d.transform, left, top, op=round, precision=precision
                    ), rowcol(
                        src_d.transform, right, bottom, op=round, precision=precision
                    )))

                temp_d = np.zeros(dst_shape, dtype=dtype)
                temp_d = src_d.read(
                    out=temp_d, window=src_window, boundless=True, masked=False
                )

                temp_e = np.zeros(dst_shape, dtype=dtype)
                temp_e = src_e.read(
                    out=temp_e, window=src_window, boundless=True, masked=False
                )

                np.multiply(temp_d, temp_e, out=temp_d)
                np.add(dstarr, temp_d, out=dstarr)
                np.add(distsum, temp_e, out=distsum)

            np.divide(dstarr, distsum, out=dstarr, where=distsum[0] != 0.0)
            dstarr[dstarr == 0.0] = src_nodata
            
            dstrast.write(dstarr, window=dst_window)

    # Cleanup
    for _, euclidean_geotiff in inputs:
        if io.file_exists(euclidean_geotiff):
            os.remove(euclidean_geotiff)
    
    # Restore logging
    if debug_enabled:
        logging.disable(logging.NOTSET)
    
    return output_dem