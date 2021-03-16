#!/usr/bin/env python3
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

target_images = [] # all
target_images.append("DJI_0018.JPG")


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

            r = shot.pose.get_rotation_matrix()
            Xs, Ys, Zs = shot.pose.get_origin()
            cam_w = shot.camera.width
            cam_h = shot.camera.height
            img_w, img_h, num_bands = shot_image.shape
            f = shot.camera.focal


            def process_pixels(step):
                imgout = np.full((num_bands, w, h), np.nan, dtype=shot_image.dtype)
                for i in range(w):
                    if i % max_workers == step:
                        for j in range(h):
                            # i, j = (168, 442) # TODO REMOVE

                            # World coordinates
                            Xa, Ya = dem_raster.xy(i, j)
                            Za = dem[j][i]

                            # Colinearity function http://web.pdx.edu/~jduh/courses/geog493f14/Week03.pdf
                            dx = (Xa - Xs)
                            dy = (Ya - Ys)
                            dz = (Za - Zs)

                            den = r[0][2] * dx + r[1][2] * dy + r[2][2] * dz
                            x = 0.5 + (-f * (r[0][0] * dx + r[1][0] * dy + r[2][0] * dz) / den)
                            y = 0.5 + (-f * (r[0][1] * dx + r[1][1] * dy + r[2][1] * dz) / den)

                            # TEST
                            if x >= 0 and y >= 0 and x <= 1.0 and y <= 1.0:
                                for b in range(3):
                                    imgout[b][i][j] = 255
                return imgout

            with multiprocessing.Pool(max_workers) as p:
                results = p.map(process_pixels, range(max_workers))

            # Merge
            imgout = results[0]
            for i in range(w):
                for b in range(num_bands):
                    imgout[b][i] = results[i % max_workers][b][i]

            print(w)
            print(h)

            profile = {
                'driver': 'GTiff',
                'width': imgout.shape[1],
                'height': imgout.shape[2],
                'count': num_bands,
                'dtype': imgout.dtype.name,
                'nodata': None
            }
            with rasterio.open("/datasets/brighton2/odm_meshing/tmp/out.tif", 'w', **profile) as wout:
                for b in range(num_bands):
                    wout.write(imgout[b], b + 1)