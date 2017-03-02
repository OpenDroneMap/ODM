# A script to calculate the NDVI from a color-infrared orthophoto.
# requires python-gdal

import numpy
import argparse
import os.path
try:
    from osgeo import gdal
    from osgeo import osr
except ImportError:
    raise ImportError("You need to install python-gdal. run `apt-get install python-gdal`")
    exit()


def parse_args():
    p = argparse.ArgumentParser("A script that calculates the NDVI of a CIR orthophoto")

    p.add_argument("orthophoto", metavar="<orthophoto.tif>",
                   type=argparse.FileType('r'),
                   help="The CIR orthophoto. Must be a GeoTiff.")
    p.add_argument("nir", metavar="N", type=int,
                   help="NIR band number")
    p.add_argument("vis", metavar="N", type=int,
                   help="Vis band number")
    p.add_argument("out", metavar="<outfile.tif>",
                   type=argparse.FileType('w'),
                   help="The output file. Also must be in GeoTiff format")
    p.add_argument("--overwrite", "-o",
                   action='store_true',
                   default=False,
                   help="Will overwrite output file if it exists. ")
    return p.parse_args()


def calc_ndvi(nir, vis):
    """
    Calculates the NDVI of an orthophoto using nir and vis bands.
    :param nir: An array containing the nir band
    :param vis: An array containing the vis band
    :return: An array that will be exported as a tif
    """

    # Take the orthophoto and do nir - vis / nir + vis
    # for each cell, calculate ndvi (masking out where divide by 0)
    ndvi = numpy.empty(nir.shape, dtype=float)
    mask = numpy.not_equal((nirb + visb), 0.0)
    return numpy.choose(mask, (-1.0, numpy.true_divide(numpy.subtract(nirb, visb), numpy.add(nirb, visb))))


if __name__ == "__main__":

    rootdir = os.path.dirname(os.path.abspath(__file__))

    # Parse args
    args = parse_args()

    if not args.overwrite and os.path.isfile(os.path.join(rootdir, args.out.name)):
        print("File exists, rename or use -o to overwrite.")
        exit()

    # import raster
    raster = gdal.Open(args.orthophoto.name)
    orthophoto = raster.ReadAsArray()
    # parse out bands
    nirb = orthophoto[args.nir - 1].astype(float)
    visb = orthophoto[args.vis - 1].astype(float)

    outfile = args.out

    # Do ndvi calc
    ndvi = calc_ndvi(nirb, visb)

    # export raster
    out_driver = gdal.GetDriverByName('GTiff')\
        .Create(outfile.name, int(ndvi.shape[1]), int(ndvi.shape[0]), 1, gdal.GDT_Float32)
    outband = out_driver.GetRasterBand(1)
    outband.WriteArray(ndvi)
    outcrs = osr.SpatialReference()
    outcrs.ImportFromWkt(raster.GetProjectionRef())
    out_driver.SetProjection(outcrs.ExportToWkt())
    out_driver.SetGeoTransform(raster.GetGeoTransform())
    outband.FlushCache()
