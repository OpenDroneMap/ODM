#!/usr/bin/python
# -*- coding: utf-8 -*-
import rasterio, os, sys
import numpy as np

class bcolors:
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

try:
    file = sys.argv[1]
    typ = sys.argv[2]
    (fileRoot, fileExt) = os.path.splitext(file)
    outFileName = fileRoot + "_" + typ + fileExt
    if typ not in ['vari', 'tgi', 'ngrdi']:
        raise IndexError
except (TypeError, IndexError, NameError):
    print bcolors.FAIL + 'Arguments messed up. Check arguments order and index name' + bcolors.ENDC
    print 'Usage: ./vegind.py orto index'
    print '       orto - filepath to RGB orthophoto'
    print '       index - Vegetation Index'
    print bcolors.OKGREEN + 'Available indexes: vari, ngrdi, tgi' + bcolors.ENDC
    sys.exit()


def calcNgrdi(red, green):
    """
    Normalized green red difference index
    Tucker,C.J.,1979.
    Red and photographic infrared linear combinations for monitoring vegetation.
    Remote Sensing of Environment 8, 127–150
    :param red: red visible channel
    :param green: green visible channel
    :return: ngrdi index array
    """
    mask = np.not_equal(np.add(red,green), 0.0)
    return np.choose(mask, (-9999.0, np.true_divide(
    np.subtract(green,red),
    np.add(red,green))))

def calcVari(red,green,blue):
    """
    Calculates Visible Atmospheric Resistant Index
    Gitelson, A.A., Kaufman, Y.J., Stark, R., Rundquist, D., 2002.
    Novel algorithms for remote estimation of vegetation fraction.
    Remote Sensing of Environment 80, 76–87.
    :param red: red visible channel
    :param green: green visible channel
    :param blue: blue visible channel
    :return: vari index array, that will be saved to tiff
    """
    mask = np.not_equal(np.subtract(np.add(green,red),blue), 0.0)
    return np.choose(mask, (-9999.0, np.true_divide(np.subtract(green,red),np.subtract(np.add(green,red),blue))))

def calcTgi(red,green,blue):
    """
    Calculates Triangular Greenness Index
    Hunt, E. Raymond Jr.; Doraiswamy, Paul C.; McMurtrey, James E.; Daughtry, Craig S.T.; Perry, Eileen M.; and Akhmedov, Bakhyt,
    A visible band index for remote sensing leaf chlorophyll content at the canopy scale (2013).
    Publications from USDA-ARS / UNL Faculty. Paper 1156.
    http://digitalcommons.unl.edu/usdaarsfacpub/1156
    :param red: red channel
    :param green: green channel
    :param blue: blue channel
    :return: tgi index array, that will be saved to tiff
    """
    mask = np.not_equal(green-red+blue-255.0, 0.0)
    return np.choose(mask, (-9999.0, np.subtract(green, np.multiply(0.39,red), np.multiply(0.61, blue))))

try:
    with rasterio.Env():
        ds = rasterio.open(file)
        profile = ds.profile
        profile.update(dtype=rasterio.float32, count=1, nodata=-9999)
        red = np.float32(ds.read(1))
        green = np.float32(ds.read(2))
        blue = np.float32(ds.read(3))
        np.seterr(divide='ignore', invalid='ignore')
        if typ == 'ngrdi':
            indeks = calcNgrdi(red,green)
        elif typ == 'vari':
            indeks = calcVari(red, green, blue)
        elif typ == 'tgi':
            indeks = calcTgi(red, green, blue)

        with rasterio.open(outFileName, 'w', BIGTIFF="IF_SAFER", **profile) as dst:
            dst.write(indeks.astype(rasterio.float32), 1)
except rasterio.errors.RasterioIOError:
    print bcolors.FAIL + 'Orthophoto file not found or access denied' + bcolors.ENDC
    sys.exit()
