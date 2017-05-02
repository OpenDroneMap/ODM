#!/usr/bin/env python

# To run, set the following env variables:
# PYTHONHOME location of Python
# PYTHONPATH location of GRASS Python libs
# PATH include GRASS bin and lib
# GISBASE location of GRASS

import os
import sys
import grass.script as gscript
import grass.script.core
import grass.script.setup

rsurfName = 'odm_rsurf'
contourName = 'odm_contour'
orthophotoName = 'odm_orthophoto'
reliefName = 'odm_relief'
shadedReliefName = reliefName + '_shaded'

overwrite = True


def main():
    if len(sys.argv) < 2:
        sys.exit('Please provide the ODM project path.')

    projectHome = sys.argv[1]

    gisdb = projectHome+'/grassdata'
    location = 'odm'
    gisrc = gscript.setup.init(os.environ['GISBASE'], gisdb, location)

    # get srs and initial extents
    with open(projectHome+'/odm_georeferencing/coords.txt') as f:
        srs = f.readline().split()
        mean = f.readline().split()
        meanX = float(mean[0])
        meanY = float(mean[1])
        minX = float('inf')
        maxX = float('-inf')
        minY = float('inf')
        maxY = float('-inf')
        for line in f:
            xy = line.split()
            x = float(xy[0])
            y = float(xy[1])
            minX = min(x, minX)
            maxX = max(x, maxX)
            minY = min(y, minY)
            maxY = max(y, maxY)

    datum = srs[0]
    proj = srs[1]
    zone = srs[2]
    gscript.core.create_location(gisdb, location, datum=datum,
                                 proj4='+proj='+proj+' +zone='+zone,
                                 overwrite=overwrite)

    n = meanY + maxY
    s = meanY + minY
    e = meanX + maxX
    w = meanX + minX
    gscript.run_command('g.region', flags='s', n=n, s=s, e=e, w=w, res=0.01,
                        res3=0.01, overwrite=overwrite)

    contour(projectHome)
    relief(projectHome)

    os.remove(gisrc)


def contour(projectHome):
    """
    Creates a contour map based on the ODM project DEM model.
    """
    print 'Creating contour map'

    step = 0.25
    
    gscript.run_command('r.in.gdal', flags='o',
                        input=projectHome+'/odm_georeferencing/odm_georeferencing_model_dem.tif',
                        output=rsurfName, memory=2047,
                        overwrite=overwrite)

    gscript.run_command('r.contour', input=rsurfName, output=contourName,
                        step=step, overwrite=overwrite)

    gscript.run_command('v.out.ogr', input=contourName,
                        output=projectHome +
                        '/odm_georeferencing/odm_contour.shp',
                        overwrite=overwrite)


def relief(projectHome):
    """
    Creates a textured relief map in GeoTIFF format.
    NB: this is an RGBA raster and so is readable by image software.
    """
    print 'Creating relief map'

    gscript.run_command('r.in.gdal', flags='o',
                        input=projectHome+'/odm_orthophoto/odm_orthophoto.tif',
                        output=orthophotoName, memory=2047,
                        overwrite=overwrite)

    gscript.run_command('r.composite', red=orthophotoName+'.red',
                        green=orthophotoName+'.green',
                        blue=orthophotoName+'.blue',
                        output=orthophotoName+'.rgb',
                        overwrite=overwrite)

    gscript.run_command('r.relief', input=rsurfName, output=reliefName,
                        overwrite=overwrite)

    gscript.run_command('r.shade', shade=reliefName,
                        color=orthophotoName+'.rgb', output=shadedReliefName,
                        overwrite=overwrite)

    calc = ';'.join([
        '$shadedRelief.red = ' +
        'if(isnull($orthophoto.red), 0, r#$shadedRelief)',
        '$shadedRelief.green = ' +
        'if(isnull($orthophoto.green), 0, g#$shadedRelief)',
        '$shadedRelief.blue = ' +
        'if(isnull($orthophoto.blue), 0, b#$shadedRelief)',
        '$shadedRelief.alpha = ' +
        'if(isnull($orthophoto.alpha), 0, 255)'
    ])
    gscript.mapcalc(calc, shadedRelief=shadedReliefName,
                    orthophoto=orthophotoName, overwrite=overwrite)

    gscript.run_command('i.group', group=shadedReliefName+'.group',
                        input=shadedReliefName+'.red,' +
                        shadedReliefName+'.green,' +
                        shadedReliefName+'.blue,' +
                        shadedReliefName+'.alpha')

    gscript.run_command('r.out.gdal', flags='cm',
                        input=shadedReliefName+'.group',
                        output=projectHome+'/odm_orthophoto/odm_relief.tif',
                        format='GTiff', type='Byte',
                        createopt='TILED=yes,COMPRESS=DEFLATE,PREDICTOR=2,' +
                        'BLOCKXSIZE=512,BLOCKYSIZE=512',
                        nodata=0, overwrite=overwrite)


if __name__ == '__main__':
    main()
