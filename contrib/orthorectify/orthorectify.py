#!/usr/bin/env python3
# Author: Piero Toffanin
# License: AGPLv3

import os
import sys
sys.path.insert(0, os.path.join("..", "..", os.path.dirname(__file__)))

from math import sqrt
import rasterio
import numpy as np
import numpy.ma as ma
import multiprocessing
import argparse
from skimage.draw import line
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
                    help="Comma-separated list of filenames to rectify. Use as an alternative to --image-list. Default: process all images.")
parser.add_argument('--threads',
                    type=int,
                    default=multiprocessing.cpu_count(),
                    help="Number of CPU processes to use. Default: %(default)s")
parser.add_argument('--skip-visibility-test',
                    type=bool,
                    help="Skip visibility testing (faster but leaves artifacts due to relief displacement)")
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
    dem_has_nodata = dem_raster.profile.get('nodata') is not None

    if dem_has_nodata:
        m = ma.array(dem, mask=dem==dem_raster.nodata)
        dem_min_value = m.min()
        dem_max_value = m.max()
    else:
        dem_min_value = dem.min()
        dem_max_value = dem.max()

    print("DEM Minimum: %s" % dem_min_value)
    print("DEM Maximum: %s" % dem_max_value)
    
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
            l = f.readline() # discard

            # second line is a northing/easting offset
            l = f.readline().rstrip()
            dem_offset_x, dem_offset_y = map(float, l.split(" "))
        
        print("DEM offset: (%s, %s)" % (dem_offset_x, dem_offset_y))

    print("DEM dimensions: %sx%s pixels" % (w, h))

    # Read reconstruction
    udata = dataset.UndistortedDataSet(dataset.DataSet(os.path.join(dataset_path, "opensfm")), undistorted_data_path=os.path.join(dataset_path, "opensfm", "undistorted"))
    reconstructions = udata.load_undistorted_reconstruction()
    if len(reconstructions) == 0:
        raise Exception("No reconstructions available")

    max_workers = args.threads
    print("Using %s threads" % max_workers)

    reconstruction = reconstructions[0]
    for shot in reconstruction.shots.values():
        if len(target_images) == 0 or shot.id in target_images:

            print("Processing %s..." % shot.id)
            shot_image = udata.load_undistorted_image(shot.id)

            r = shot.pose.get_rotation_matrix()
            Xs, Ys, Zs = shot.pose.get_origin()
            cam_grid_y, cam_grid_x = dem_raster.index(Xs + dem_offset_x, Ys + dem_offset_y)

            a1 = r[0][0]
            b1 = r[0][1]
            c1 = r[0][2]
            a2 = r[1][0]
            b2 = r[1][1]
            c2 = r[1][2]
            a3 = r[2][0]
            b3 = r[2][1]
            c3 = r[2][2]

            if not args.skip_visibility_test:
                distance_map = np.full((h, w), np.nan)

                for j in range(0, h):
                    for i in range(0, w):
                        distance_map[j][i] = sqrt((cam_grid_x - i) ** 2 + (cam_grid_y - j) ** 2)
                distance_map[distance_map==0] = 1e-7

            print("Camera pose: (%f, %f, %f)" % (Xs, Ys, Zs))

            img_h, img_w, num_bands = shot_image.shape
            half_img_w = (img_w - 1) / 2.0
            half_img_h = (img_h - 1) / 2.0
            print("Image dimensions: %sx%s pixels" % (img_w, img_h))
            f = shot.camera.focal * max(img_h, img_w)
            has_nodata = dem_raster.profile.get('nodata') is not None

            def process_pixels(step):
                imgout = np.full((num_bands, dem_bbox_h, dem_bbox_w), np.nan)

                minx = dem_bbox_w
                miny = dem_bbox_h
                maxx = 0
                maxy = 0

                for j in range(dem_bbox_miny, dem_bbox_maxy + 1):
                    if j % max_workers == step:
                        im_j = j - dem_bbox_miny

                        for i in range(dem_bbox_minx, dem_bbox_maxx + 1):
                            im_i = i - dem_bbox_minx

                            # World coordinates
                            Za = dem[j][i]

                            # Skip nodata
                            if has_nodata and Za == dem_raster.nodata:
                                continue

                            Xa, Ya = dem_raster.xy(j, i)

                            # Remove offset (our cameras don't have the geographic offset)
                            Xa -= dem_offset_x
                            Ya -= dem_offset_y

                            # Colinearity function http://web.pdx.edu/~jduh/courses/geog493f14/Week03.pdf
                            dx = (Xa - Xs)
                            dy = (Ya - Ys)
                            dz = (Za - Zs)

                            den = a3 * dx + b3 * dy + c3 * dz
                            x = half_img_w - (f * (a1 * dx + b1 * dy + c1 * dz) / den)
                            y = half_img_h - (f * (a2 * dx + b2 * dy + c2 * dz) / den)

                            if x >= 0 and y >= 0 and x <= img_w - 1 and y <= img_h - 1:
                                # Visibility test
                                if not args.skip_visibility_test:
                                    check_dem_points = np.column_stack(line(i, j, cam_grid_x, cam_grid_y))
                                    check_dem_points = check_dem_points[np.all(np.logical_and(np.array([0, 0]) <= check_dem_points, check_dem_points < [w, h]), axis=1)]

                                    visible = True
                                    for p in check_dem_points:
                                        ray_z = Zs + (distance_map[p[1]][p[0]] / distance_map[j][i]) * dz
                                        if ray_z > dem_max_value:
                                            break

                                        if dem[p[1]][p[0]] > ray_z:
                                            visible = False
                                            break
                                    if not visible:
                                        continue

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
                                    minx = min(minx, im_i)
                                    miny = min(miny, im_j)
                                    maxx = max(maxx, im_i)
                                    maxy = max(maxy, im_j)

                                    for b in range(num_bands):
                                        imgout[b][im_j][im_i] = values[b]

                                # for b in range(num_bands):
                                #     minx = min(minx, im_i)
                                #     miny = min(miny, im_j)
                                #     maxx = max(maxx, im_i)
                                #     maxy = max(maxy, im_j)
                                #     imgout[b][im_j][im_i] = 255
                return (imgout, (minx, miny, maxx, maxy))

            # Compute bounding box of image coverage
            # assuming a flat plane at Z = min Z
            # (Otherwise we have to scan the entire DEM)
            # The Xa,Ya equations are just derived from the colinearity equations
            # solving for Xa and Ya instead of x,y
            def dem_coordinates(cpx, cpy):
                """
                :param cpx principal point X (image coordinates)
                :param cpy principal point Y (image coordinates)
                """
                Za = dem_min_value
                m = (a3*b1*cpy - a1*b3*cpy - (a3*b2 - a2*b3)*cpx - (a2*b1 - a1*b2)*f)
                Xa = dem_offset_x + (m*Xs + (b3*c1*cpy - b1*c3*cpy - (b3*c2 - b2*c3)*cpx - (b2*c1 - b1*c2)*f)*Za - (b3*c1*cpy - b1*c3*cpy - (b3*c2 - b2*c3)*cpx - (b2*c1 - b1*c2)*f)*Zs)/m
                Ya = dem_offset_y + (m*Ys - (a3*c1*cpy - a1*c3*cpy - (a3*c2 - a2*c3)*cpx - (a2*c1 - a1*c2)*f)*Za + (a3*c1*cpy - a1*c3*cpy - (a3*c2 - a2*c3)*cpx - (a2*c1 - a1*c2)*f)*Zs)/m

                y, x = dem_raster.index(Xa, Ya)
                return (x, y)

            dem_ul = dem_coordinates(-(img_w - 1) / 2.0, -(img_h - 1) / 2.0)
            dem_ur = dem_coordinates((img_w - 1) / 2.0, -(img_h - 1) / 2.0)
            dem_lr = dem_coordinates((img_w - 1) / 2.0, (img_h - 1) / 2.0)
            dem_ll = dem_coordinates(-(img_w - 1) / 2.0, (img_h - 1) / 2.0)
            dem_bbox = [dem_ul, dem_ur, dem_lr, dem_ll]
            dem_bbox_x = np.array(list(map(lambda xy: xy[0], dem_bbox)))
            dem_bbox_y = np.array(list(map(lambda xy: xy[1], dem_bbox)))

            dem_bbox_minx = min(w - 1, max(0, dem_bbox_x.min()))
            dem_bbox_miny = min(h - 1, max(0, dem_bbox_y.min()))
            dem_bbox_maxx = min(w - 1, max(0, dem_bbox_x.max()))
            dem_bbox_maxy = min(h - 1, max(0, dem_bbox_y.max()))
            
            dem_bbox_w = 1 + dem_bbox_maxx - dem_bbox_minx
            dem_bbox_h = 1 + dem_bbox_maxy - dem_bbox_miny

            print("Iterating over DEM box: [(%s, %s), (%s, %s)] (%sx%s pixels)" % (dem_bbox_minx, dem_bbox_miny, dem_bbox_maxx, dem_bbox_maxy, dem_bbox_w, dem_bbox_h))

            if max_workers > 1:
                with multiprocessing.Pool(max_workers) as p:
                    results = p.map(process_pixels, range(max_workers))
            else:
                results = [process_pixels(0)]

            results = list(filter(lambda r: r[1][0] <= r[1][2] and r[1][1] <= r[1][3], results))

            # Merge image
            imgout, _ = results[0]

            for j in range(dem_bbox_miny, dem_bbox_maxy + 1):
                im_j = j - dem_bbox_miny
                resimg, _ = results[j % max_workers]
                for b in range(num_bands):
                    imgout[b][im_j] = resimg[b][im_j]
                
            # Merge bounds
            minx = dem_bbox_w
            miny = dem_bbox_h
            maxx = 0
            maxy = 0

            for _, bounds in results:
                minx = min(bounds[0], minx)
                miny = min(bounds[1], miny)
                maxx = max(bounds[2], maxx)
                maxy = max(bounds[3], maxy)
            
            print("Output bounds: (%s, %s), (%s, %s) pixels" % (minx, miny, maxx, maxy))
            if minx <= maxx and miny <= maxy:
                imgout = imgout[:,miny:maxy+1,minx:maxx+1]

                if with_alpha:
                    alpha = np.zeros((imgout.shape[1], imgout.shape[2]), dtype=np.uint8)

                    # Set all not-NaN indices to 255
                    alpha[~np.isnan(imgout[0])] = 255

                # Cast
                imgout = imgout.astype(shot_image.dtype)

                dem_transform = dem_raster.profile['transform']
                offset_x, offset_y = dem_raster.xy(dem_bbox_miny + miny, dem_bbox_minx + minx, offset='ul')
                 
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

                with rasterio.open(outfile, 'w', BIGTIFF="IF_SAFER", **profile) as wout:
                    for b in range(num_bands):
                        wout.write(imgout[b], b + 1)
                    if with_alpha:
                        wout.write(alpha, num_bands + 1)

                print("Wrote %s" % outfile)
            else:
                print("Cannot orthorectify image (is the image inside the DEM bounds?)")
