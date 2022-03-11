#!/usr/bin/env python3
# ******************************************************************************
#  $Id: gdal_fillnodata.py 124baa7f71f15396a661014a81b6c5b0c82c8004 2020-10-14 17:29:39 +0300 Idan Miara $
#
#  Project:  GDAL Python Interface
#  Purpose:  Application for filling nodata areas in a raster by interpolation
#  Author:   Frank Warmerdam, warmerdam@pobox.com
#
# ******************************************************************************
#  Copyright (c) 2008, Frank Warmerdam
#  Copyright (c) 2009-2011, Even Rouault <even dot rouault at spatialys.com>
#
#  Permission is hereby granted, free of charge, to any person obtaining a
#  copy of this software and associated documentation files (the "Software"),
#  to deal in the Software without restriction, including without limitation
#  the rights to use, copy, modify, merge, publish, distribute, sublicense,
#  and/or sell copies of the Software, and to permit persons to whom the
#  Software is furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included
#  in all copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
#  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
#  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
#  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
#  DEALINGS IN THE SOFTWARE.
# ******************************************************************************

import sys

from osgeo import gdal


def CopyBand(srcband, dstband):
    for line in range(srcband.YSize):
        line_data = srcband.ReadRaster(0, line, srcband.XSize, 1)
        dstband.WriteRaster(0, line, srcband.XSize, 1, line_data,
                            buf_type=srcband.DataType)


def Usage():
    print("""
gdal_fillnodata [-q] [-md max_distance] [-si smooth_iterations]
                [-o name=value] [-b band]
                srcfile [-nomask] [-mask filename] [-of format] [-co name=value]* [dstfile]
""")
    sys.exit(1)


def main(argv):
    max_distance = 100
    smoothing_iterations = 0
    options = []
    quiet_flag = 0
    src_filename = None
    src_band = 1

    dst_filename = None
    frmt = 'GTiff'
    creation_options = []

    mask = 'default'

    gdal.AllRegister()
    argv = gdal.GeneralCmdLineProcessor(argv)
    if argv is None:
        sys.exit(0)

    # Parse command line arguments.
    i = 1
    while i < len(argv):
        arg = argv[i]

        if arg == '-of' or arg == '-f':
            i = i + 1
            frmt = argv[i]

        elif arg == '-co':
            i = i + 1
            creation_options.append(argv[i])

        elif arg == '-q' or arg == '-quiet':
            quiet_flag = 1

        elif arg == '-si':
            i = i + 1
            smoothing_iterations = int(argv[i])

        elif arg == '-b':
            i = i + 1
            src_band = int(argv[i])

        elif arg == '-md':
            i = i + 1
            max_distance = float(argv[i])

        elif arg == '-nomask':
            mask = 'none'

        elif arg == '-mask':
            i = i + 1
            mask = argv[i]

        elif arg == '-mask':
            i = i + 1
            mask = argv[i]

        elif arg[:2] == '-h':
            Usage()

        elif src_filename is None:
            src_filename = argv[i]

        elif dst_filename is None:
            dst_filename = argv[i]

        else:
            Usage()

        i = i + 1

    if src_filename is None:
        Usage()

    # =============================================================================
    # 	Verify we have next gen bindings with the sievefilter method.
    # =============================================================================
    try:
        gdal.FillNodata
    except AttributeError:
        print('')
        print('gdal.FillNodata() not available.  You are likely using "old gen"')
        print('bindings or an older version of the next gen bindings.')
        print('')
        sys.exit(1)

    # =============================================================================
    # Open source file
    # =============================================================================

    if dst_filename is None:
        src_ds = gdal.Open(src_filename, gdal.GA_Update)
    else:
        src_ds = gdal.Open(src_filename, gdal.GA_ReadOnly)

    if src_ds is None:
        print('Unable to open %s' % src_filename)
        sys.exit(1)

    srcband = src_ds.GetRasterBand(src_band)

    if mask == 'default':
        maskband = srcband.GetMaskBand()
    elif mask == 'none':
        maskband = None
    else:
        mask_ds = gdal.Open(mask)
        maskband = mask_ds.GetRasterBand(1)

    # =============================================================================
    #       Create output file if one is specified.
    # =============================================================================

    if dst_filename is not None:

        drv = gdal.GetDriverByName(frmt)
        dst_ds = drv.Create(dst_filename, src_ds.RasterXSize, src_ds.RasterYSize, 1,
                            srcband.DataType, creation_options)
        wkt = src_ds.GetProjection()
        if wkt != '':
            dst_ds.SetProjection(wkt)
        gt = src_ds.GetGeoTransform(can_return_null=True)
        if gt:
            dst_ds.SetGeoTransform(gt)

        dstband = dst_ds.GetRasterBand(1)
        CopyBand(srcband, dstband)
        ndv = srcband.GetNoDataValue()
        if ndv is not None:
            dstband.SetNoDataValue(ndv)

        color_interp = srcband.GetColorInterpretation()
        dstband.SetColorInterpretation(color_interp)
        if color_interp == gdal.GCI_PaletteIndex:
            color_table = srcband.GetColorTable()
            dstband.SetColorTable(color_table)

    else:
        dstband = srcband

    # =============================================================================
    # Invoke algorithm.
    # =============================================================================

    if quiet_flag:
        prog_func = None
    else:
        prog_func = gdal.TermProgress_nocb

    result = gdal.FillNodata(dstband, maskband,
                             max_distance, smoothing_iterations, options,
                             callback=prog_func)


    src_ds = None
    dst_ds = None
    mask_ds = None

    return result


if __name__ == '__main__':
    sys.exit(main(sys.argv))
