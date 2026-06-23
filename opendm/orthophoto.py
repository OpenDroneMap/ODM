import os
import threading
import contextlib
from collections import deque
from concurrent.futures import ThreadPoolExecutor
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
from opendm.utils import add_raster_meta_tags
from osgeo import gdal
from osgeo import ogr


@contextlib.contextmanager
def _bounded_gdal_cache(nbytes):
    """Temporarily cap GDAL's global block cache, restoring it on exit.

    A small cache keeps the parallel merge's output-tile flushes prompt and
    cheap. Restoring on the way out — even if the merge raises — avoids leaving
    the rest of the pipeline (notably the COG conversion) with a shrunken cache.
    """
    prev = gdal.GetCacheMax()
    gdal.SetCacheMax(nbytes)
    try:
        yield
    finally:
        gdal.SetCacheMax(prev)


def _read_window_gated(ds, src_window, dst_shape, dtype):
    """Read ``src_window`` into a ``dst_shape`` array — equivalent to a
    ``boundless=True`` read with 0 nodata fill, but avoiding rasterio's boundless
    VRT path for the common cases. ``boundless=True`` builds a VRT and serializes
    it via Python's ElementTree (``_serialize_xml``) on every read, which is a
    large per-read overhead on big merges. Behaviour:
      - window fully outside the dataset -> all zeros (no read), matching the 0
        nodata fill a boundless read would produce here;
      - window fully inside -> plain non-boundless read (no VRT), identical to a
        boundless read when no out-of-bounds padding is needed;
      - window partially overlapping the edge -> fall back to boundless (rare;
        only the true border blocks), preserving exact fill behaviour.
    """
    (r0, r1), (c0, c1) = src_window
    out = np.zeros(dst_shape, dtype=dtype)
    height, width = ds.height, ds.width
    if r1 <= 0 or c1 <= 0 or r0 >= height or c0 >= width:
        return out
    if r0 >= 0 and c0 >= 0 and r1 <= height and c1 <= width:
        return ds.read(out=out, window=src_window, boundless=False, masked=False)
    return ds.read(out=out, window=src_window, boundless=True, masked=False)


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
                '--config INTERLEAVE_OVERVIEW PIXEL '
                '--config PHOTOMETRIC_OVERVIEW YCBCR'
                '{orthophoto} 2 4 8 16'.format(**kwargs))

def generate_png(orthophoto_file, output_file=None, outsize=None):
    if output_file is None:
        base, ext = os.path.splitext(orthophoto_file)
        output_file = base + '.png'
    
    # See if we need to select top three bands
    params = []

    try:
        gtif = gdal.Open(orthophoto_file)
        bands = []
        for idx in range(1, gtif.RasterCount+1):
            bands.append(gtif.GetRasterBand(idx).GetColorInterpretation())
        bands = dict(zip(bands, range(1, len(bands)+1)))

        if gtif.RasterCount >= 3:
            red = bands.get(gdal.GCI_RedBand)
            green = bands.get(gdal.GCI_GreenBand)
            blue = bands.get(gdal.GCI_BlueBand)
            if red is None or green is None or blue is None:
                params.append("-b 1 -b 2 -b 3")
            else:
                params.append("-b %s -b %s -b %s" % (red, green, blue))                
        elif gtif.RasterCount <= 2:
            params.append("-b 1")
        
        alpha = bands.get(gdal.GCI_AlphaBand)
        if alpha is not None:
            params.append("-b %s" % alpha)
        else:
            params.append("-a_nodata 0")

        dtype = gtif.GetRasterBand(1).DataType
        if dtype != gdal.GDT_Byte:
            params.append("-ot Byte")
            if gtif.RasterCount >= 3:
                params.append("-scale_1 -scale_2 -scale_3")
            elif gtif.RasterCount <= 2:
                params.append("-scale_1")
        
        gtif = None
    except Exception as e:
        log.ODM_WARNING("Cannot read orthophoto information for PNG generation: %s" % str(e))

    if outsize is not None:
        params.append("-outsize %s 0" % outsize)

    system.run('gdal_translate -of png "%s" "%s" %s '
               '-co WORLDFILE=YES '
               '--config GDAL_CACHEMAX %s%% ' % (orthophoto_file, output_file, " ".join(params), get_max_memory()))

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

def generate_extent_polygon(orthophoto_file):
    """Function to return the orthophoto extent as a polygon into a gpkg file

    Args:
        orthophoto_file (str): the path to orthophoto file
    """
    base, ext = os.path.splitext(orthophoto_file)
    output_file = base + '_extent.dxf'

    try:
        gtif = gdal.Open(orthophoto_file)
        srs =  gtif.GetSpatialRef()
        geoTransform = gtif.GetGeoTransform()

        # calculate the coordinates
        minx = geoTransform[0]
        maxy = geoTransform[3]
        maxx = minx + geoTransform[1] * gtif.RasterXSize
        miny = maxy + geoTransform[5] * gtif.RasterYSize
        
        # create polygon in wkt format
        poly_wkt = "POLYGON ((%s %s, %s %s, %s %s, %s %s, %s %s))" % (minx, miny, minx, maxy, maxx, maxy, maxx, miny, minx, miny)
        
        # create vector file
        # just the DXF to support AutoCAD users
        # to load the geotiff raster correctly.
        driver = ogr.GetDriverByName("DXF")
        ds = driver.CreateDataSource(output_file)
        layer = ds.CreateLayer("extent", srs, ogr.wkbPolygon)

        # create the feature and set values
        featureDefn = layer.GetLayerDefn()
        feature = ogr.Feature(featureDefn)
        feature.SetGeometry(ogr.CreateGeometryFromWkt(poly_wkt))

        # add feature to layer
        layer.CreateFeature(feature)

        # save and close everything
        feature = None
        ds = None
        gtif = None
        log.ODM_INFO("Wrote %s" % output_file)
    except Exception as e:
        log.ODM_WARNING("Cannot create extent layer for %s: %s" % (orthophoto_file, str(e)))


def generate_tfw(orthophoto_file):
    base, ext = os.path.splitext(orthophoto_file)
    tfw_file = base + '.tfw'

    try:
        with rasterio.open(orthophoto_file) as ds:
            t = ds.transform
            with open(tfw_file, 'w') as f:
                # rasterio affine values taken by
                # https://mharty3.github.io/til/GIS/raster-affine-transforms/
                f.write("\n".join([str(v) for v in [t.a, t.d, t.b, t.e, t.c, t.f]]) + "\n")
            log.ODM_INFO("Wrote %s" % tfw_file)
    except Exception as e:
        log.ODM_WARNING("Cannot create .tfw for %s: %s" % (orthophoto_file, str(e)))


def post_orthophoto_steps(args, bounds_file_path, orthophoto_file, orthophoto_tiles_dir, resolution, reconstruction, tree, embed_gcp_meta=False):
    if args.crop > 0 or args.boundary:
        Cropper.crop(bounds_file_path, orthophoto_file, get_orthophoto_vars(args), keep_original=not args.optimize_disk_space, warp_options=['-dstalpha'])

    if args.build_overviews and not args.cog:
        build_overviews(orthophoto_file)

    if args.orthophoto_png:
        generate_png(orthophoto_file)
        
    if args.orthophoto_kmz:
        generate_kmz(orthophoto_file)

    add_raster_meta_tags(orthophoto_file, reconstruction, tree, embed_gcp_meta=embed_gcp_meta)

    if args.tiles:
        generate_orthophoto_tiles(orthophoto_file, orthophoto_tiles_dir, args.max_concurrency, resolution)

    if args.cog:
        convert_to_cogeo(orthophoto_file, max_workers=args.max_concurrency, compression=args.orthophoto_compression)

    generate_extent_polygon(orthophoto_file)
    generate_tfw(orthophoto_file)

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

def _read_window_gated(ds, src_window, dst_shape, dtype):
    """Read ``src_window`` into a ``dst_shape`` array, equivalent to a
    ``boundless=True`` read with 0 nodata fill, but avoiding rasterio's boundless
    VRT path for the common cases. ``boundless=True`` builds a VRT and serializes
    it via Python's ElementTree (``_serialize_xml``) on every read, which is a
    large per-read overhead on big merges. Behaviour:
      - window fully outside the dataset -> all zeros (no read), matching the 0
        nodata fill a boundless read would produce here;
      - window fully inside -> plain non-boundless read (no VRT), identical to a
        boundless read when no out-of-bounds padding is needed;
      - window partially overlapping the edge -> fall back to boundless (rare;
        only the true border blocks), preserving exact fill behaviour.
    """
    (r0, r1), (c0, c1) = src_window
    out = np.zeros(dst_shape, dtype=dtype)
    height, width = ds.height, ds.width
    if r1 <= 0 or c1 <= 0 or r0 >= height or c0 >= width:
        return out
    if r0 >= 0 and c0 >= 0 and r1 <= height and c1 <= width:
        return ds.read(out=out, window=src_window, boundless=False, masked=False)
    return ds.read(out=out, window=src_window, boundless=True, masked=False)

def merge(input_ortho_and_ortho_cuts, output_orthophoto, orthophoto_vars={}, merge_skip_blending=False, max_workers=1):
    """
    Based on https://github.com/mapbox/rio-merge-rgba/
    Merge orthophotos around cutlines using a blend buffer.

    Each output block is an independent pure function of its source windows and
    the fixed source ordering, so blocks are processed in parallel. With
    max_workers <= 1 (the default) processing is strictly serial and the output
    is byte-for-byte identical to the original single-threaded loop.

    Args:
        input_ortho_and_ortho_cuts: iterable of (orthophoto_path, cut_path) pairs.
        output_orthophoto: path for the merged output GeoTIFF.
        orthophoto_vars: rasterio profile overrides (TILED, COMPRESS, etc.).
        merge_skip_blending: if True, skip the feather/cutline blend passes
            (ODM #1934 --merge-skip-blending); only the first naive-copy pass runs.
        max_workers: number of parallel worker threads (default 1 = serial).
    Returns:
        The output_orthophoto path, or None if there were no valid inputs.
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
        dst_count = first.count

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
        if src.profile["count"] < 2:
            raise ValueError("Inputs must be at least 2-band rasters")
    dst_w, dst_s, dst_e, dst_n = min(xs), min(ys), max(xs), max(ys)
    # Close the pre-scan handles; they are unused in the parallel block loop.
    for s, c in sources:
        s.close()
        c.close()
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

    # Log here to avoid logging in each block processed
    if merge_skip_blending:
        log.ODM_INFO("Skipping second and third pass orthophoto blending, as --merge-skip-blending passed")

    # create destination file. Cap GDAL's global block cache for the merge (restored on
    # exit, see _bounded_gdal_cache): the default (5% of RAM) lets dirty output tiles
    # accumulate, so a source read can trigger a large eviction/flush under GDAL's global
    # lock that stalls the parallel readers; a small cache keeps flushes prompt and cheap.
    cache_bytes = 512 * 1024 * 1024
    with _bounded_gdal_cache(cache_bytes), \
            rasterio.open(output_orthophoto, "w", **profile) as dstrast:
        dstrast.colorinterp = colorinterp

        # Each output block is an independent function of its source windows and
        # the source ordering, so blocks can be COMPUTED in parallel. But writes
        # to a single compressed, tiled GeoTIFF must happen in row-major (block)
        # order: out-of-order writes cannot be flushed incrementally, so GDAL
        # hoards every dirty block in RAM until it thrashes or OOMs. So we compute
        # in a thread pool and write from one thread in strict block order, with a
        # small bounded look-ahead for backpressure. max_workers <= 1 is a plain
        # serial compute+write loop, identical to the original.
        tls = threading.local()
        block_windows = [(dst_window, dstrast.window_bounds(dst_window))
                         for _, dst_window in dstrast.block_windows()]
        total_blocks = len(block_windows)
        log_every = max(1, total_blocks // 20)

        opened_sources = []
        opened_lock = threading.Lock()

        def get_sources():
            """Return this thread's (ortho, cut) rasterio dataset handles.

            GDAL/rasterio handles are not safe to share across threads, so each
            worker thread lazily opens and caches its own set on first use and
            registers it in opened_sources for cleanup after the parallel run.
            """
            srcs = getattr(tls, "sources", None)
            if srcs is None:
                srcs = [(rasterio.open(o), rasterio.open(c)) for o, c in inputs]
                tls.sources = srcs
                with opened_lock:
                    opened_sources.append(srcs)
            return srcs

        def compute_block(item):
            """Compute one output block (read + 3 blend passes); return the array.

            Does NOT write — writing happens in block order on the main thread.
            """
            dst_window, (left, bottom, right, top) = item
            local_sources = get_sources()

            blocksize = dst_window.width
            dst_rows, dst_cols = (dst_window.height, dst_window.width)
            dst_shape = (dst_count, dst_rows, dst_cols)

            dstarr = np.zeros(dst_shape, dtype=dtype)

            # First pass, write all rasters naively without blending
            for src, _ in local_sources:
                src_window = tuple(zip(rowcol(
                        src.transform, left, top, op=round, precision=precision
                    ), rowcol(
                        src.transform, right, bottom, op=round, precision=precision
                    )))

                temp = _read_window_gated(src, src_window, dst_shape, dtype)

                # pixels without data yet are available to write
                write_region = np.logical_and(
                    (dstarr[-1] == 0), (temp[-1] != 0)  # 0 is nodata
                )
                np.copyto(dstarr, temp, where=write_region)

                # check if dest has any nodata pixels available
                if np.count_nonzero(dstarr[-1]) == blocksize:
                    break

            # Skip the feather/cutline blend passes if requested (ODM #1934)
            if merge_skip_blending:
                return dstarr

            # Second pass, write all feathered rasters
            # blending the edges
            for src, _ in local_sources:
                src_window = tuple(zip(rowcol(
                        src.transform, left, top, op=round, precision=precision
                    ), rowcol(
                        src.transform, right, bottom, op=round, precision=precision
                    )))

                temp = _read_window_gated(src, src_window, dst_shape, dtype)

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
            for _, cut in local_sources:
                src_window = tuple(zip(rowcol(
                        cut.transform, left, top, op=round, precision=precision
                    ), rowcol(
                        cut.transform, right, bottom, op=round, precision=precision
                    )))

                temp = _read_window_gated(cut, src_window, dst_shape, dtype)

                # For each band, average alpha values between
                # destination raster and cut raster
                for b in range(0, num_bands):
                    blended = temp[-1] / 255.0 * temp[b] + (1 - temp[-1] / 255.0) * dstarr[b]
                    np.copyto(dstarr[b], blended, casting='unsafe', where=temp[-1]!=0)

            return dstarr

        def write_block(idx, dst_window, dstarr):
            dstrast.write(dstarr, window=dst_window)
            if (idx + 1) % log_every == 0:
                log.ODM_INFO("Merging orthophoto: %s / %s blocks" % (idx + 1, total_blocks))

        if max_workers <= 1:
            # Serial: compute and write each block in order (original behavior).
            for idx, item in enumerate(block_windows):
                write_block(idx, item[0], compute_block(item))
        else:
            # Parallel compute; one in-order writer with bounded look-ahead so at
            # most `cap` blocks are in flight — keeps memory small and gives GDAL
            # strictly sequential, incrementally-flushable writes.
            cap = max_workers * 2
            with ThreadPoolExecutor(max_workers=max_workers) as ex:
                items = iter(enumerate(block_windows))
                pending = deque()
                for _ in range(cap):
                    nxt = next(items, None)
                    if nxt is None:
                        break
                    idx, item = nxt
                    pending.append((idx, item[0], ex.submit(compute_block, item)))
                while pending:
                    widx, wwin, fut = pending.popleft()
                    dstarr = fut.result()
                    write_block(widx, wwin, dstarr)
                    nxt = next(items, None)
                    if nxt is not None:
                        idx, item = nxt
                        pending.append((idx, item[0], ex.submit(compute_block, item)))

        # Close all thread-local source handles opened during the run.
        for srcs in opened_sources:
            for s, c in srcs:
                s.close()
                c.close()

    return output_orthophoto
