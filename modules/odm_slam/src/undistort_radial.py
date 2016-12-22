#!/usr/bin/env python

import argparse
import os

import cv2
import numpy as np

import opensfm.dataset as dataset
import opensfm.io as io


def opencv_calibration_matrix(width, height, focal):
    '''Calibration matrix as used by OpenCV and PMVS
    '''
    f = focal * max(width, height)
    return np.matrix([[f, 0, 0.5 * (width - 1)],
                      [0, f, 0.5 * (height - 1)],
                      [0, 0, 1.0]])

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Undistort images')
    parser.add_argument('dataset', help='path to the dataset to be processed')
    parser.add_argument('--output', help='output folder for the undistorted images')
    args = parser.parse_args()

    data = dataset.DataSet(args.dataset)
    if args.output:
        output_path = args.output
    else:
        output_path = os.path.join(data.data_path, 'undistorted')

    print "Undistorting images from dataset [%s] to dir [%s]" % (data.data_path, output_path)

    io.mkdir_p(output_path)

    reconstructions = data.load_reconstruction()
    for h, reconstruction in enumerate(reconstructions):
        print "undistorting reconstruction", h
        for image in reconstruction['shots']:
            print "undistorting image", image
            shot = reconstruction["shots"][image]

            original_image = data.image_as_array(image)[:,:,::-1]
            camera = reconstruction['cameras'][shot['camera']]
            original_h, original_w = original_image.shape[:2]
            K = opencv_calibration_matrix(original_w, original_h, camera['focal'])
            k1 = camera["k1"]
            k2 = camera["k2"]
            undistorted_image = cv2.undistort(original_image, K, np.array([k1, k2, 0, 0]))

            new_image_path = os.path.join(output_path, image.split('/')[-1])
            cv2.imwrite(new_image_path, undistorted_image)
