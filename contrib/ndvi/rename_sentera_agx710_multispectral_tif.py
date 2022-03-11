#!/usr/bin/env python3
#  A script to rename.
# requires python-gdal

import argparse
import sys
try:
    from osgeo import gdal
except ImportError:
    raise ImportError("You need to install python-gdal : \
                       run `sudo apt-get install libgdal-dev` \
                       # Check Gdal version with \
                       gdal-config --version \
                       #install correspondig gdal version with pip : \
                       pip3 install GDAL==2.4.0")

def parse_args():
    """ Parse arguments """
    argument_parser = argparse.ArgumentParser(
        "A script that rename inplace Sentera AGX710 Geotiff orthophoto. ")
    argument_parser.add_argument("orthophoto", metavar="<orthophoto.tif>",
                   type=argparse.FileType('r'),
                   help="The input orthophoto. Must be a GeoTiff.")
    return argument_parser.parse_args()


def rename_sentera_agx710_layers(name):
    """ Only rename Geotif  built from Sentera AGX710 images with ODM """
    if raster.RasterCount != 7:
        raise ImportError(F'File {name} does not have 7 layers as a regular\
             Geotif  built from Sentera AGX710 images with ODM')

    if 'RedGreenBlue' in raster.GetRasterBand(1).GetDescription() and \
            'RedEdgeGarbageNIR' in raster.GetRasterBand(2).GetDescription():

        print("Sentera AGX710 Geotiff file has been detected.\
               Layers are name are :")
        print("RedGreenBlue for Band 1\nRedEdgeGarbageNIR for Band 2\
               \nNone for Band 3\nNone for Band 4\nNone for Band 5\nNone for Band 6")
        print("\nAfter renaming bands will be :")
        print("Red for Band 1\nGreen for Band 2\nBlue for Band 3\n\
               RedEdge for Band 4\nGarbage for Band 5\nNIR for Band 6")

        answer = input(
            "Are you sure you want to rename the layers of the input file ? [yes/no] ")
        if answer =='yes':
            raster.GetRasterBand(1).SetDescription('Red')
            raster.GetRasterBand(2).SetDescription('Green')
            raster.GetRasterBand(3).SetDescription('Blue')
            raster.GetRasterBand(4).SetDescription('RedEdge')
            raster.GetRasterBand(5).SetDescription('Garbage')
            raster.GetRasterBand(6).SetDescription('NIR')
            # raster.GetRasterBand(7).SetDescription('Alpha')
        else:
            print("No renaming")
    else :
        print(F'No need for band renaming in {name}')
        sys.exit()


if __name__ == "__main__":

    # Parse args
    args = parse_args()

    # import raster
    raster = gdal.Open(args.orthophoto.name, gdal.GA_Update)

    # Rename layers
    rename_sentera_agx710_layers(args.orthophoto.name)

    # de-reference the datasets, which triggers gdal to save
    raster = None
