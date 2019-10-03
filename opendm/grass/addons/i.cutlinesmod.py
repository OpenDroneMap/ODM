#!/usr/bin/env python

############################################################################
#
# MODULE:	i.cutlinesmod
# AUTHOR(S):	Moritz Lennert, with help of Stefanos Georganos, modified by
#               Piero Toffanin
#
# PURPOSE:	Create tiles the borders of which do not cut across semantically
#               meaningful objects
# COPYRIGHT:	(C) 1997-2018 by the GRASS Development Team
#
#		This program is free software under the GNU General Public
#		License (>=v2). Read the file COPYING that comes with GRASS
#		for details.
#############################################################################

#%Module
#% description: Creates semantically meaningful tile borders
#% keyword: imagery
#% keyword: tiling
#%end
#
#%option G_OPT_R_INPUT
#% description: Raster map to use as input for tiling
#% required: yes
#%end
#
#%option G_OPT_V_OUTPUT
#% description: Name of output vector map with cutline polygons
#%end
#
#%option
#% key: number_lines
#% type: integer
#% description: Number of tile border lines in each direction
#% required: yes
#%end
#
#%option
#% key: edge_detection
#% type: string
#% description: Edge detection algorithm to use
#% options: zc,canny
#% answer: zc
#% required: yes
#%end
#
#%option G_OPT_V_INPUTS
#% key: existing_cutlines
#% label: Input vector maps with existing cutlines
#% required: no
#%end
#
#%option
#% key: no_edge_friction
#% type: integer
#% description: Additional friction for non-edge pixels
#% required: yes
#% answer: 5
#%end
#
#%option
#% key: lane_border_multiplier
#% type: integer
#% description: Multiplier for borders of lanes compared to non-edge pixels
#% required: yes
#% answer: 10
#%end
#
#%option
#% key: min_tile_size
#% type: integer
#% description: Minimum size of tiles in map units
#% required: no
#%end
#
#%option
#% key: zc_threshold
#% type: double
#% label: Sensitivity of Gaussian filter (i.zc)
#% answer: 1
#% required: no
#% guisection: Zero-crossing
#%end
#
#%option
#% key: zc_width
#% type: integer
#% label: x-y extent of the Gaussian filter (i.zc)
#% answer: 9
#% required: no
#% guisection: Zero-crossing
#%end
#
#%option
#% key: canny_low_threshold
#% type: double
#% label: Low treshold for edges (i.edge)
#% answer: 3
#% required: no
#% guisection: Canny
#%end
#
#%option
#% key: canny_high_threshold
#% type: double
#% label: High treshold for edges (i.edge)
#% answer: 10
#% required: no
#% guisection: Canny
#%end
#
#%option
#% key: canny_sigma
#% type: double
#% label: Kernel radius (i.edge)
#% answer: 2
#% required: no
#% guisection: Canny
#%end
#
#%option
#% key: tile_width
#% type: integer
#% description: Width of tiles for tiled edge detection (pixels)
#% required: no
#% guisection: Parallel processing
#%end
#
#%option
#% key: tile_height
#% type: integer
#% description: Height of tiles for tiled edge detection (pixels)
#% required: no
#% guisection: Parallel processing
#%end
#
#%option
#% key: overlap
#% type: integer
#% description: Overlap between tiles for tiled edge detection (pixels)
#% required: no
#% answer: 1
#% guisection: Parallel processing
#%end
#
#%option
#% key: processes
#% type: integer
#% description: Number of parallel processes
#% answer: 1
#% required: yes
#% guisection: Parallel processing
#%end
#
#%option
#% key: memory
#% type: integer
#% description: RAM memory available (in MB)
#% answer: 300
#% required: yes
#%end
#
#%rules
#% collective: tile_width, tile_height, overlap
#%end

import os
import atexit
import grass.script as gscript
from grass.pygrass.modules.grid.grid import GridModule
from grass.pygrass.vector import VectorTopo
from grass.pygrass.vector import geometry as geom

def cleanup():
    gscript.message(_("Erasing temporary files..."))
    for temp_map, maptype in temp_maps:
        if gscript.find_file(temp_map, element=maptype)['name']:
            gscript.run_command('g.remove', flags='f', type=maptype,
                              name=temp_map, quiet=True)


def listzip(input1, input2):
    # python3 compatible
    out = zip(input1, input2)
    if not isinstance(out, list):
        out = list(zip(input1, input2))
    return out


def main():
    inputraster = options['input']
    number_lines = int(options['number_lines'])
    edge_detection_algorithm = options['edge_detection']
    no_edge_friction = int(options['no_edge_friction'])
    lane_border_multiplier = int(options['lane_border_multiplier'])
    min_tile_size = None
    if options['min_tile_size']:
        min_tile_size = float(options['min_tile_size'])
    existing_cutlines = None
    if options['existing_cutlines']:
        existing_cutlines = options['existing_cutlines'].split(',')
    tiles = options['output']
    memory = int(options['memory'])
    tiled = False

    if options['tile_width']:
        tiled = True
        gscript.message(_("Using tiles processing for edge detection"))
        width = int(options['tile_width'])
        height = int(options['tile_height'])
        overlap = int(options['overlap'])

    processes = int(options['processes'])

    global temp_maps
    temp_maps = []
    r = 'raster'
    v = 'vector'

    if existing_cutlines:
        existingcutlinesmap = 'temp_icutlines_existingcutlinesmap_%i' % os.getpid()
        if len(existing_cutlines) > 1:
            gscript.run_command('v.patch',
                                input_=existing_cutlines,
                                output=existingcutlinesmap,
                                quiet=True,
                                overwrite=True)
            existing_cutlines=existingcutlinesmap

        gscript.run_command('v.to.rast',
                            input_=existing_cutlines,
                            output=existingcutlinesmap,
                            use='val',
                            type_='line,boundary',
                            overwrite=True,
                            quiet=True)

        temp_maps.append([existingcutlinesmap, r])

    temp_edge_map = "temp_icutlines_edgemap_%d" % os.getpid()
    temp_maps.append([temp_edge_map, r])

    gscript.message(_("Creating edge map"))
    if edge_detection_algorithm == 'zc':
        kwargs = {'input' : inputraster,
                  'output' : temp_edge_map,
                  'width_' : int(options['zc_width']),
                  'threshold' : float(options['zc_threshold']),
                  'quiet' : True}

        if tiled:
            grd = GridModule('i.zc',
                             width=width,
                             height=height,
                             overlap=overlap,
                             processes=processes,
                             split=False,
                             **kwargs)
            grd.run()
        else:
            gscript.run_command('i.zc',
                                **kwargs)

    elif edge_detection_algorithm == 'canny':
        if not gscript.find_program('i.edge', '--help'):
                message = _("You need to install the addon i.edge to use ")
                message += _("the Canny edge detector.\n")
                message += _(" You can install the addon with 'g.extension i.edge'")
                gscript.fatal(message)

        kwargs = {'input' : inputraster,
                  'output' : temp_edge_map,
                  'low_threshold' : float(options['canny_low_threshold']),
                  'high_threshold' : float(options['canny_high_threshold']),
                  'sigma' : float(options['canny_sigma']),
                  'quiet' : True}


        if tiled:
            grd = GridModule('i.edge',
                             width=width,
                             height=height,
                             overlap=overlap,
                             processes=processes,
                             split=False,
                             **kwargs)
            grd.run()
        else:
            gscript.run_command('i.edge',
                                **kwargs)

    else:
        gscript.fatal("Only zero-crossing and Canny available as edge detection algorithms.")

    region = gscript.region()
    gscript.message(_("Finding cutlines in both directions"))

    nsrange = float(region.n - region.s - region.nsres)
    ewrange = float(region.e - region.w - region.ewres)

    if nsrange > ewrange:
        hnumber_lines = number_lines
        vnumber_lines = int(number_lines * (nsrange / ewrange))
    else:
        vnumber_lines = number_lines
        hnumber_lines = int(number_lines * (ewrange / nsrange))

    # Create the lines in horizonal direction
    nsstep = float(region.n - region.s - region.nsres) / hnumber_lines
    hpointsy = [((region.n - i * nsstep) - region.nsres / 2.0) for i in range(0, hnumber_lines+1)]
    hlanepointsy = [y - nsstep / 2.0 for y in hpointsy]
    hstartpoints = listzip([region.w + 0.2 * region.ewres] * len(hpointsy), hpointsy)
    hstoppoints = listzip([region.e - 0.2 * region.ewres] * len(hpointsy), hpointsy)
    hlanestartpoints = listzip([region.w + 0.2 * region.ewres] * len(hlanepointsy), hlanepointsy)
    hlanestoppoints = listzip([region.e - 0.2 * region.ewres] * len(hlanepointsy), hlanepointsy)

    hlanemap = 'temp_icutlines_hlanemap_%i' % os.getpid()
    temp_maps.append([hlanemap, v])
    temp_maps.append([hlanemap, r])

    os.environ['GRASS_VERBOSE'] = '0'
    new = VectorTopo(hlanemap)
    new.open('w')
    for line in listzip(hlanestartpoints,hlanestoppoints):
        new.write(geom.Line(line), cat=1)
    new.close()
    del os.environ['GRASS_VERBOSE']

    gscript.run_command('v.to.rast',
                        input_=hlanemap,
                        output=hlanemap,
                        use='val',
                        type_='line',
                        overwrite=True,
                        quiet=True)

    hbasemap = 'temp_icutlines_hbasemap_%i' % os.getpid()
    temp_maps.append([hbasemap, r])

    # Building the cost maps using the following logic
    # - Any pixel not on an edge, nor on an existing cutline gets a
    # no_edge_friction cost, or no_edge_friction_cost x 10  if there are
    # existing cutlines
    # - Any pixel on an edge gets a cost of 1 if there are no existing cutlines,
    # and a cost of no_edge_friction if there are
    # - A lane line gets a very high cost (lane_border_multiplier x cost of no
    # edge pixel - the latter depending on the existence of cutlines).

    mapcalc_expression = "%s = " % hbasemap
    mapcalc_expression += "if(isnull(%s), " % hlanemap
    if existing_cutlines:
        mapcalc_expression += "if(%s == 0 && isnull(%s), " % (temp_edge_map, existingcutlinesmap)
        mapcalc_expression += "%i, " % (no_edge_friction * 10)
        mapcalc_expression += "if(isnull(%s), %s, 1))," % (existingcutlinesmap, no_edge_friction)
        mapcalc_expression += "%i)" % (lane_border_multiplier * no_edge_friction * 10)
    else:
        mapcalc_expression += "if(%s == 0, " % temp_edge_map
        mapcalc_expression += "%i, " % no_edge_friction
        mapcalc_expression += "1), "
        mapcalc_expression += "%i)" % (lane_border_multiplier * no_edge_friction)
    gscript.run_command('r.mapcalc',
                        expression=mapcalc_expression,
                        quiet=True,
                        overwrite=True)

    hcumcost = 'temp_icutlines_hcumcost_%i' % os.getpid()
    temp_maps.append([hcumcost, r])
    hdir = 'temp_icutlines_hdir_%i' % os.getpid()
    temp_maps.append([hdir, r])


    # Create the lines in vertical direction
    ewstep = float(region.e - region.w - region.ewres) / vnumber_lines
    vpointsx = [((region.e - i * ewstep) - region.ewres / 2.0) for i in range(0, vnumber_lines+1)]
    vlanepointsx = [x + ewstep / 2.0 for x in vpointsx]
    vstartpoints = listzip(vpointsx, [region.n - 0.2 * region.nsres] * len(vpointsx))
    vstoppoints = listzip(vpointsx, [region.s + 0.2 * region.nsres] * len(vpointsx))
    vlanestartpoints = listzip(vlanepointsx, [region.n - 0.2 * region.nsres] * len(vlanepointsx))
    vlanestoppoints = listzip(vlanepointsx, [region.s + 0.2 * region.nsres] * len(vlanepointsx))


    vlanemap = 'temp_icutlines_vlanemap_%i' % os.getpid()
    temp_maps.append([vlanemap, v])
    temp_maps.append([vlanemap, r])

    os.environ['GRASS_VERBOSE'] = '0'
    new = VectorTopo(vlanemap)
    new.open('w')
    for line in listzip(vlanestartpoints,vlanestoppoints):
        new.write(geom.Line(line), cat=1)
    new.close()
    del os.environ['GRASS_VERBOSE']

    gscript.run_command('v.to.rast',
                        input_=vlanemap,
                        output=vlanemap,
                        use='val',
                        type_='line',
                        overwrite=True,
                        quiet=True)

    vbasemap = 'temp_icutlines_vbasemap_%i' % os.getpid()
    temp_maps.append([vbasemap, r])
    mapcalc_expression = "%s = " % vbasemap
    mapcalc_expression += "if(isnull(%s), " % vlanemap
    if existing_cutlines:
        mapcalc_expression += "if(%s == 0 && isnull(%s), " % (temp_edge_map, existingcutlinesmap)
        mapcalc_expression += "%i, " % (no_edge_friction  * 10)
        mapcalc_expression += "if(isnull(%s), %s, 1))," % (existingcutlinesmap, no_edge_friction)
        mapcalc_expression += "%i)" % (lane_border_multiplier * no_edge_friction * 10)
    else:
        mapcalc_expression += "if(%s == 0, " % temp_edge_map
        mapcalc_expression += "%i, " % no_edge_friction
        mapcalc_expression += "1), "
        mapcalc_expression += "%i)" % (lane_border_multiplier * no_edge_friction)
    gscript.run_command('r.mapcalc',
                        expression=mapcalc_expression,
                        quiet=True,
                        overwrite=True)

    vcumcost = 'temp_icutlines_vcumcost_%i' % os.getpid()
    temp_maps.append([vcumcost, r])
    vdir = 'temp_icutlines_vdir_%i' % os.getpid()
    temp_maps.append([vdir, r])

    if processes > 1:
        pmemory = memory / 2.0
        rcv = gscript.start_command('r.cost',
                                    input_=vbasemap,
                                    startcoordinates=vstartpoints,
                                    stopcoordinates=vstoppoints,
                                    output=vcumcost,
                                    outdir=vdir,
                                    memory=pmemory,
                                    quiet=True,
                                    overwrite=True)

        rch = gscript.start_command('r.cost',
                                    input_=hbasemap,
                                    startcoordinates=hstartpoints,
                                    stopcoordinates=hstoppoints,
                                    output=hcumcost,
                                    outdir=hdir,
                                    memory=pmemory,
                                    quiet=True,
                                    overwrite=True)
        rcv.wait()
        rch.wait()

    else:
        gscript.run_command('r.cost',
                            input_=vbasemap,
                            startcoordinates=vstartpoints,
                            stopcoordinates=vstoppoints,
                            output=vcumcost,
                            outdir=vdir,
                            memory=memory,
                            quiet=True,
                            overwrite=True)

        gscript.run_command('r.cost',
                            input_=hbasemap,
                            startcoordinates=hstartpoints,
                            stopcoordinates=hstoppoints,
                            output=hcumcost,
                            outdir=hdir,
                            memory=memory,
                            quiet=True,
                            overwrite=True)

    hlines = 'temp_icutlines_hlines_%i' % os.getpid()
    temp_maps.append([hlines, r])
    vlines = 'temp_icutlines_vlines_%i' % os.getpid()
    temp_maps.append([vlines, r])

    if processes > 1:
        rdh = gscript.start_command('r.drain',
                                    input_=hcumcost,
                                    direction=hdir,
                                    startcoordinates=hstoppoints,
                                    output=hlines,
                                    flags='d',
                                    quiet=True,
                                    overwrite=True)


        rdv = gscript.start_command('r.drain',
                                    input_=vcumcost,
                                    direction=vdir,
                                    startcoordinates=vstoppoints,
                                    output=vlines,
                                    flags='d',
                                    quiet=True,
                                    overwrite=True)

        rdh.wait()
        rdv.wait()

    else:
        gscript.run_command('r.drain',
                            input_=hcumcost,
                            direction=hdir,
                            startcoordinates=hstoppoints,
                            output=hlines,
                            flags='d',
                            quiet=True,
                            overwrite=True)


        gscript.run_command('r.drain',
                            input_=vcumcost,
                            direction=vdir,
                            startcoordinates=vstoppoints,
                            output=vlines,
                            flags='d',
                            quiet=True,
                            overwrite=True)

    # Combine horizonal and vertical lines
    temp_raster_tile_borders = 'temp_icutlines_raster_tile_borders_%i' % os.getpid()
    temp_maps.append([temp_raster_tile_borders, r])
    gscript.run_command('r.patch',
                        input_=[hlines,vlines],
                        output=temp_raster_tile_borders,
                        quiet=True,
                        overwrite=True)

    gscript.message(_("Creating vector polygons"))

    # Create vector polygons

    # First we need to shrink the region a bit to make sure that all vector
    # points / lines fall within the raster
    gscript.use_temp_region()
    gscript.run_command('g.region',
                        s=region.s+region.nsres,
                        e=region.e-region.ewres,
                        quiet=True)

    region_map = 'temp_icutlines_region_map_%i' % os.getpid()
    temp_maps.append([region_map, v])
    temp_maps.append([region_map, r])
    gscript.run_command('v.in.region',
                        output=region_map,
                        type_='line',
                        quiet=True,
                        overwrite=True)

    gscript.del_temp_region()

    gscript.run_command('v.to.rast',
                        input_=region_map,
                        output=region_map,
                        use='val',
                        type_='line',
                        quiet=True,
                        overwrite=True)

    temp_raster_polygons = 'temp_icutlines_raster_polygons_%i' % os.getpid()
    temp_maps.append([temp_raster_polygons, r])
    gscript.run_command('r.patch',
                        input_=[temp_raster_tile_borders,region_map],
                        output=temp_raster_polygons,
                        quiet=True,
                        overwrite=True)

    temp_raster_polygons_thin = 'temp_icutlines_raster_polygons_thin_%i' % os.getpid()
    temp_maps.append([temp_raster_polygons_thin, r])
    gscript.run_command('r.thin',
                        input_=temp_raster_polygons,
                        output=temp_raster_polygons_thin,
                        quiet=True,
                        overwrite=True)

    # Create a series of temporary map names as we have to go
    # through several steps until we reach the final map.
    temp_vector_polygons1 = 'temp_icutlines_vector_polygons1_%i' % os.getpid()
    temp_maps.append([temp_vector_polygons1, v])
    temp_vector_polygons2 = 'temp_icutlines_vector_polygons2_%i' % os.getpid()
    temp_maps.append([temp_vector_polygons2, v])
    temp_vector_polygons3 = 'temp_icutlines_vector_polygons3_%i' % os.getpid()
    temp_maps.append([temp_vector_polygons3, v])
    temp_vector_polygons4 = 'temp_icutlines_vector_polygons4_%i' % os.getpid()
    temp_maps.append([temp_vector_polygons4, v])

    gscript.run_command('r.to.vect',
                        input_=temp_raster_polygons_thin,
                        output=temp_vector_polygons1,
                        type_='line',
                        flags='t',
                        quiet=True,
                        overwrite=True)

    # Erase all category values from the lines
    gscript.run_command('v.category',
                        input_=temp_vector_polygons1,
                        op='del',
                        cat='-1',
                        output=temp_vector_polygons2,
                        quiet=True,
                        overwrite=True)

    # Transform lines to boundaries
    gscript.run_command('v.type',
                        input_=temp_vector_polygons2,
                        from_type='line',
                        to_type='boundary',
                        output=temp_vector_polygons3,
                        quiet=True,
                        overwrite=True)

    # Add centroids
    gscript.run_command('v.centroids',
                        input_=temp_vector_polygons3,
                        output=temp_vector_polygons4,
                        quiet=True,
                        overwrite=True)

    # If a threshold is given erase polygons that are too small
    if min_tile_size:
        gscript.run_command('v.clean',
                            input_=temp_vector_polygons4,
                            tool='rmarea',
                            threshold=min_tile_size,
                            output=tiles,
                            quiet=True,
                            overwrite=True)
    else:
        gscript.run_command('g.copy',
                            vect=[temp_vector_polygons4,tiles],
                            quiet=True,
                            overwrite=True)

    gscript.vector_history(tiles)

if __name__ == "__main__":
    options, flags = gscript.parser()
    atexit.register(cleanup)
    main()
