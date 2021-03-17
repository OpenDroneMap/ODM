#!/usr/bin/env python3
# Author: Piero Toffanin
# License: AGPLv3

import os
import sys
sys.path.insert(0, os.path.join("..", "..", os.path.dirname(__file__)))

import rasterio
import numpy as np
from opensfm import dataset
import multiprocessing

# TODO: command argument parser

dataset_path = "/datasets/brighton2"
dem_path = "/datasets/brighton2/odm_meshing/tmp/mesh_dsm.tif"
interpolation = 'linear' # 'bilinear'
with_alpha = True

target_images = [] # all
target_images.append("DJI_0030.JPG")

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

# Read DSM
print("Reading DSM: %s" % dem_path)
with rasterio.open(dem_path) as dem_raster:
    dem = dem_raster.read()[0]
    h, w = dem.shape

    print("DSM dimensions: %sx%s pixels" % (w, h))
   
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

            r = shot.pose.get_rotation_matrix().T
            Xs, Ys, Zs = shot.pose.get_origin()

            print("Camera pose: (%f, %f, %f)" % (Xs, Ys, Zs))

            img_h, img_w, num_bands = shot_image.shape
            print("Image dimensions: %sx%s pixels" % (img_w, img_h))
            f = shot.camera.focal * max(img_h, img_w)

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

                            # Colinearity function http://web.pdx.edu/~jduh/courses/geog493f14/Week03.pdf
                            dx = (Xa - Xs)
                            dy = (Ya - Ys)
                            dz = (Za - Zs)

                            den = r[0][2] * dx + r[1][2] * dy + r[2][2] * dz
                            x = (img_w - 1) / 2.0 - (f * (r[0][0] * dx + r[1][0] * dy + r[2][0] * dz) / den)
                            y = (img_h - 1) / 2.0 - (f * (r[0][1] * dx + r[1][1] * dy + r[2][1] * dz) / den)

                            if x >= 0 and y >= 0 and x <= img_w - 1 and y <= img_h - 1:
                                
                                # for b in range(num_bands):
                                if interpolation == 'bilinear':
                                    xi = img_w - 1 - x
                                    yi = img_h - 1 - y
                                    values = bilinear_interpolate(shot_image, xi, yi)
                                else:
                                    # Linear
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

                profile = {
                    'driver': 'GTiff',
                    'width': imgout.shape[2],
                    'height': imgout.shape[1],
                    'count': num_bands + 1 if with_alpha else num_bands,
                    'dtype': imgout.dtype.name,
                    'nodata': None
                }
                with rasterio.open("/datasets/brighton2/odm_meshing/tmp/out.tif", 'w', **profile) as wout:
                    for b in range(num_bands):
                        wout.write(imgout[b], b + 1)
                    if with_alpha:
                        wout.write(alpha, num_bands + 1)
            else:
                print("Cannot orthorectify image (is the image inside the DSM bounds?)")
