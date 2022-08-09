#!/usr/bin/env python3
# A script to calculate agricultural indices
# NDVI - Normalized Difference Vegetation Index - (NIR−RED)/(NIR + RED)
# NDRE - Normalized Difference Red Edge - (NIR−RE)/(NIR + RE)
# GNDVI - Green NDVI - (NIR−GREEN)/(NIR + GREEN)
# https://support.micasense.com/hc/en-us/articles/226531127-Creating-agricultural-indices-NDVI-NDRE-in-QGIS-
# requires python-gdal

import numpy
import argparse
import os.path
try:
    from osgeo import gdal
    from osgeo import osr
except ImportError:
    raise ImportError("You need to install python-gdal : \
                       run `sudo apt-get install libgdal-dev` \
                       # Check Gdal version with \
                       gdal-config --version \
                       #install corresponding gdal version with pip : \
                       pip3 install GDAL==2.4.0")


def parse_args():
    argument_parser = argparse.ArgumentParser('Createa from a multispectral orthophoto \
a Geotif with  NDVI, NDRE and GNDVI  agricultural indices')

    argument_parser.add_argument("orthophoto", metavar="<orthophoto.tif>",
                   type=argparse.FileType('r'),
                   help="The CIR orthophoto. Must be a GeoTiff.")
    argument_parser.add_argument("-red",  type=int,
                   help="Red band number")
    argument_parser.add_argument("-green", type=int,
                   help="Green band number")
    argument_parser.add_argument("-blue", type=int,
                   help="Blue band number")
    argument_parser.add_argument("-re", type=int,
                   help="RedEdge band number")
    argument_parser.add_argument("-nir", type=int,
                   help="NIR band number")
    argument_parser.add_argument("out", metavar="<outfile.tif>",
                   type=argparse.FileType('w'),
                   help="The output file.")
    argument_parser.add_argument("--overwrite", "-o",
                   action='store_true',
                   default=False,
                   help="Will overwrite output file if it exists. ")
    return argument_parser.parse_args()


if __name__ == "__main__":
    
    # Suppress/hide warning when dividing by zero
    numpy.seterr(divide='ignore', invalid='ignore')

    rootdir = os.path.dirname(os.path.abspath(__file__))

    # Parse args
    args = parse_args()

    if not args.overwrite and os.path.isfile(os.path.join(rootdir, args.out.name)):
        print("File exists, rename or use -o to overwrite.")
        exit()

    # import raster
    print("Reading file")
    raster = gdal.Open(args.orthophoto.name)
    orthophoto = raster.ReadAsArray()

    # parse out bands
    print("Reading rasters")
    red_matrix=orthophoto[args.red-1].astype(float)
    green_matrix=orthophoto[args.green-1].astype(float)
    blue_matrix=orthophoto[args.blue-1].astype(float)
    re_matrix=orthophoto[args.re-1].astype(float)
    nir_matrix=orthophoto[args.nir-1].astype(float)

    outfile = args.out

    # NDVI
    print("Computing NDVI")
    #ndvi = calc_ndvi(nir_matrix, red_matrix)
    ndvi = (nir_matrix.astype(float) - red_matrix.astype(float)) / (nir_matrix + red_matrix)
    # NDRE
    print("Computing NDRE")
    #ndre = calc_ndre(nir_matrix, re_matrix)
    ndre = (nir_matrix.astype(float) - re_matrix.astype(float)) / (nir_matrix + re_matrix)

    # GNDVI 
    print("Computing GNDVI")
    #gndvi = calc_gndvi(nir_matrix, green_matrix)
    gndvi = (nir_matrix.astype(float) - green_matrix.astype(float)) / (nir_matrix + green_matrix)

    __import__("IPython").embed()

    print("Saving Files")
    # export raster

    for name, matrix in zip(['ndvi', 'ndre', 'gndvi' ] ,[ndvi,ndre,gndvi] ):
        print(name)
        out_driver = gdal.GetDriverByName('GTiff')\
            .Create(name+'_'+outfile.name, int(ndvi.shape[1]), int(ndvi.shape[0]), 1, gdal.GDT_Float32)
        outband = out_driver.GetRasterBand(1)
        outband.SetDescription(name.capitalize())
        outband.WriteArray(matrix)
        outcrs = osr.SpatialReference()
        outcrs.ImportFromWkt(raster.GetProjectionRef())
        out_driver.SetProjection(outcrs.ExportToWkt())
        out_driver.SetGeoTransform(raster.GetGeoTransform())
        outband.FlushCache()


