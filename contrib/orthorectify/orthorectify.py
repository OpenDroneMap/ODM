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
target_images.append("DJI_0028.JPG")


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

            # A = np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]])
            # A = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
            # r = (np.linalg.inv(r).dot(A)).dot(r)
            # r = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
            
            # roll = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
            # pitch = [[-1, 0, 0], [0, 1, 0], [0, 0, -1]]
            # yaw = [[-1, 0, 0], [0, -1, 0], [0, 0, 1]]
            
            # print(r)
            # r = r.dot(pitch).dot(yaw)
            # print(r)

            img_h, img_w, num_bands = shot_image.shape
            print("Image dimensions: %sx%s pixels" % (img_w, img_h))
            f = shot.camera.focal * max(img_h, img_w)

            def process_pixels(step):
                imgout = np.full((num_bands, h, w), np.nan, dtype=shot_image.dtype)
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

                            # if abs(dx) < 3.0 and abs(dy) < 3.0:
                            #     imgout[0][j][i] = 255
                            #     imgout[1][j][i] = 0
                            #     imgout[2][j][i] = 0
                            # else:
                            #     for b in range(num_bands):
                            #         imgout[b][j][i] = 255

                            den = r[0][2] * dx + r[1][2] * dy + r[2][2] * dz
                            x = (img_w - 1) / 2.0 - (f * (r[0][0] * dx + r[1][0] * dy + r[2][0] * dz) / den)
                            y = (img_h - 1) / 2.0 - (f * (r[0][1] * dx + r[1][1] * dy + r[2][1] * dz) / den)

                            if x >= 0 and y >= 0 and x <= img_w - 1 and y <= img_h - 1:
                                xi = img_w - 1 - int(x)
                                yi = img_h - 1 - int(y)
                                for b in range(num_bands):
                                    imgout[b][j][i] = shot_image[yi][xi][b]
                                # for b in range(num_bands):
                                #     imgout[b][j][i] = 255
                return imgout

            with multiprocessing.Pool(max_workers) as p:
                results = p.map(process_pixels, range(max_workers))

            # Merge
            imgout = results[0]
            for j in range(h):
                for b in range(num_bands):
                    imgout[b][j] = results[j % max_workers][b][j]

            profile = {
                'driver': 'GTiff',
                'width': imgout.shape[2],
                'height': imgout.shape[1],
                'count': num_bands,
                'dtype': imgout.dtype.name,
                'nodata': None
            }
            with rasterio.open("/datasets/brighton2/odm_meshing/tmp/out.tif", 'w', **profile) as wout:
                for b in range(num_bands):
                    wout.write(imgout[b], b + 1)

            # # TODO REMOVE
            # profile = {
            #     'driver': 'GTiff',
            #     'width': w,
            #     'height': h,
            #     'count': 1,
            #     'dtype': dem.dtype.name,
            #     'nodata': None
            # }
            # with rasterio.open("/datasets/brighton2/odm_meshing/tmp/dsm_out.tif", 'w', **profile) as wout:
            #     wout.write(dem, 1)