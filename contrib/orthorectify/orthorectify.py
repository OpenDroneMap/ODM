#!/usr/bin/env python3
# Author: Piero Toffanin
# License: AGPLv3

import os
import sys
sys.path.insert(0, os.path.join("..", "..", os.path.dirname(__file__)))

import rasterio
import numpy as np
import multiprocessing
import argparse
from opensfm import dataset

default_dem_path = "odm_dem/dsm.tif"
default_outdir = "orthorectified"
default_image_list = "img_list.txt"

parser = argparse.ArgumentParser(description='Orthorectification Tool')
parser.add_argument('dataset',
                type=str,
                help='Path to ODM dataset')
parser.add_argument('--dem',
                type=str,
                default=default_dem_path,
                help='Absolute path to DEM to use to orthorectify images. Default: %(default)s')
parser.add_argument('--no-alpha',
                    type=bool,
                    help="Don't output an alpha channel")
parser.add_argument('--interpolation',
                    type=str,
                    choices=('nearest', 'bilinear'),
                    default='bilinear',
                    help="Type of interpolation to use to sample pixel values.Default: %(default)s")
parser.add_argument('--outdir',
                    type=str,
                    default=default_outdir,
                    help="Output directory where to store results. Default: %(default)s")
parser.add_argument('--image-list',
                    type=str,
                    default=default_image_list,
                    help="Path to file that contains the list of image filenames to orthorectify. By default all images in a dataset are processed. Default: %(default)s")
parser.add_argument('--images',
                    type=str,
                    default="",
                    help="Comma-separeted list of filenames to rectify. Use as an alternative to --image-list. Default: process all images.")

args = parser.parse_args()

dataset_path = args.dataset
dem_path = os.path.join(dataset_path, default_dem_path) if args.dem == default_dem_path else args.dem
interpolation = args.interpolation
with_alpha = not args.no_alpha
image_list = os.path.join(dataset_path, default_image_list) if args.image_list == default_image_list else args.image_list

cwd_path = os.path.join(dataset_path, default_outdir) if args.outdir == default_outdir else args.outdir

if not os.path.exists(cwd_path):
    os.makedirs(cwd_path)

target_images = [] # all

if args.images:
    target_images = list(map(str.strip, args.images.split(",")))
    print("Processing %s images" % len(target_images))
elif args.image_list:
    with open(image_list) as f:
        target_images = list(filter(lambda filename: filename != '', map(str.strip, f.read().split("\n"))))
    print("Processing %s images" % len(target_images))

if not os.path.exists(dem_path):
    print("Whoops! %s does not exist. Provide a path to a valid DEM" % dem_path)
    exit(1)


def bilinear_interpolate(im, x, y):
    x = np.asarray(x)
    y = np.asarray(y)

    x0 = np.floor(x).astype(int)
    x1 = x0 + 1
    y0 = np.floor(y).astype(int)
    y1 = y0 + 1

    x0 = np.clip(x0, 0, im.shape[1]-1)
    x1 = np.clip(x1, 0, im.shape[1]-1)
    y0 = np.clip(y0, 0, im.shape[0]-1)
    y1 = np.clip(y1, 0, im.shape[0]-1)

    Ia = im[ y0, x0 ]
    Ib = im[ y1, x0 ]
    Ic = im[ y0, x1 ]
    Id = im[ y1, x1 ]

    wa = (x1-x) * (y1-y)
    wb = (x1-x) * (y-y0)
    wc = (x-x0) * (y1-y)
    wd = (x-x0) * (y-y0)

    return wa*Ia + wb*Ib + wc*Ic + wd*Id

# Read DEM
print("Reading DEM: %s" % dem_path)
with rasterio.open(dem_path) as dem_raster:
    dem = dem_raster.read()[0]
    h, w = dem.shape

    crs = dem_raster.profile.get('crs')
    dem_offset_x, dem_offset_y = (0, 0)

    if crs:
        print("DEM has a CRS: %s" % str(crs))

        # Read coords.txt
        coords_file = os.path.join(dataset_path, "odm_georeferencing", "coords.txt")
        if not os.path.exists(coords_file):
            print("Whoops! Cannot find %s (we need that!)" % coords_file)
            exit(1)
        
        with open(coords_file) as f:
            line = f.readline() # discard

            # second line is a northing/easting offset
            line = f.readline().rstrip()
            dem_offset_x, dem_offset_y = map(float, line.split(" "))
        
        print("DEM offset: (%s, %s)" % (dem_offset_x, dem_offset_y))

    print("DEM dimensions: %sx%s pixels" % (w, h))
   
    # Read reconstruction
    udata = dataset.UndistortedDataSet(dataset.DataSet(os.path.join(dataset_path, "opensfm")))
    reconstructions = udata.load_undistorted_reconstruction()
    if len(reconstructions) == 0:
        raise Exception("No reconstructions available")

    max_workers = multiprocessing.cpu_count()
    print("Using %s threads" % max_workers)

    reconstruction = reconstructions[0]
    for shot in reconstruction.shots.values():
        if len(target_images) == 0 or shot.id in target_images:

            print("Processing %s..." % shot.id)
            shot_image = udata.load_undistorted_image(shot.id)

            r = shot.pose.get_rotation_matrix()
            Xs, Ys, Zs = shot.pose.get_origin()

            print("Camera pose: (%f, %f, %f)" % (Xs, Ys, Zs))

            img_h, img_w, num_bands = shot_image.shape
            print("Image dimensions: %sx%s pixels" % (img_w, img_h))
            f = shot.camera.focal * max(img_h, img_w)
            has_nodata = dem_raster.profile.get('nodata') is not None

            def process_pixels(step):
                imgout = np.full((num_bands, h, w), np.nan)
                minx = w
                miny = h
                maxx = 0
                maxy = 0

                for j in range(h):
                    if j % max_workers == step:
                        for i in range(w):
                            # World coordinates
                            Xa, Ya = dem_raster.xy(j, i)
                            Za = dem[j][i]

                            # Skip nodata
                            if has_nodata and Za == dem_raster.nodata:
                                continue

                            # Remove offset (our cameras don't have the geographic offset)
                            Xa -= dem_offset_x
                            Ya -= dem_offset_y

                            # Colinearity function http://web.pdx.edu/~jduh/courses/geog493f14/Week03.pdf
                            dx = (Xa - Xs)
                            dy = (Ya - Ys)
                            dz = (Za - Zs)

                            den = r[2][0] * dx + r[2][1] * dy + r[2][2] * dz
                            x = (img_w - 1) / 2.0 - (f * (r[0][0] * dx + r[0][1] * dy + r[0][2] * dz) / den)
                            y = (img_h - 1) / 2.0 - (f * (r[1][0] * dx + r[1][1] * dy + r[1][2] * dz) / den)

                            if x >= 0 and y >= 0 and x <= img_w - 1 and y <= img_h - 1:
                                
                                if interpolation == 'bilinear':
                                    xi = img_w - 1 - x
                                    yi = img_h - 1 - y
                                    values = bilinear_interpolate(shot_image, xi, yi)
                                else:
                                    # nearest
                                    xi = img_w - 1 - int(round(x))
                                    yi = img_h - 1 - int(round(y))
                                    values = shot_image[yi][xi]

                                # We don't consider all zero values (pure black)
                                # to be valid sample values. This will sometimes miss
                                # valid sample values.

                                if not np.all(values == 0):
                                    minx = min(minx, i)
                                    miny = min(miny, j)
                                    maxx = max(maxx, i)
                                    maxy = max(maxy, j)

                                    for b in range(num_bands):
                                        imgout[b][j][i] = values[b]

                                # for b in range(num_bands):
                                #     imgout[b][j][i] = 255
                return (imgout, (minx, miny, maxx, maxy))

            with multiprocessing.Pool(max_workers) as p:
                results = p.map(process_pixels, range(max_workers))

            # Merge image
            imgout, _ = results[0]
            for j in range(h):
                resimg, _ = results[j % max_workers]
                for b in range(num_bands):
                    imgout[b][j] = resimg[b][j]
                
            # Merge bounds
            minx = w
            miny = h
            maxx = 0
            maxy = 0

            for _, bounds in results:
                minx = min(bounds[0], minx)
                miny = min(bounds[1], miny)
                maxx = max(bounds[2], maxx)
                maxy = max(bounds[3], maxy)

            print("Output bounds: (%s, %s), (%s, %s) pixels" % (minx, miny, maxx, maxy))
            if minx <= maxx and miny <= maxy:
                imgout = imgout[:,miny:maxy,minx:maxx]

                if with_alpha:
                    alpha = np.zeros((imgout.shape[1], imgout.shape[2]), dtype=np.uint8)

                    # Set all not-NaN indices to 255
                    alpha[~np.isnan(imgout[0])] = 255

                # Cast
                imgout = imgout.astype(shot_image.dtype)

                dem_transform = dem_raster.profile['transform']
                offset_x, offset_y = dem_raster.xy(miny, minx, offset='ul')
                 
                profile = {
                    'driver': 'GTiff',
                    'width': imgout.shape[2],
                    'height': imgout.shape[1],
                    'count': num_bands + 1 if with_alpha else num_bands,
                    'dtype': imgout.dtype.name,
                    'transform': rasterio.transform.Affine(dem_transform[0], dem_transform[1], offset_x, 
                                                           dem_transform[3], dem_transform[4], offset_y),
                    'nodata': None,
                    'crs': crs
                }

                outfile = os.path.join(cwd_path, shot.id)
                if not outfile.endswith(".tif"):
                    outfile = outfile + ".tif"

                with rasterio.open(outfile, 'w', **profile) as wout:
                    for b in range(num_bands):
                        wout.write(imgout[b], b + 1)
                    if with_alpha:
                        wout.write(alpha, num_bands + 1)

                print("Wrote %s" % outfile)
            else:
                print("Cannot orthorectify image (is the image inside the DEM bounds?)")
