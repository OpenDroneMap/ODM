#!/usr/bin/env python
################################################################################
#   lidar2dems - utilities for creating DEMs from LiDAR data
#
#   AUTHOR: Matthew Hanson, matt.a.hanson@gmail.com
#
#   Copyright (C) 2015 Applied Geosolutions LLC, oss@appliedgeosolutions.com
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright notice, this
#     list of conditions and the following disclaimer.
#
#   * Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
################################################################################

# Library functions for creating DEMs from Lidar data

import os
import sys
import json as jsonlib
import tempfile
from opendm import system
from opendm import log
from opendm.utils import double_quote

from datetime import datetime


""" JSON Functions """


def json_base():
    """ Create initial JSON for PDAL pipeline """
    return {'pipeline': []}


def json_gdal_base(filename, output_type, radius, resolution=1, bounds=None):
    """ Create initial JSON for PDAL pipeline containing a Writer element """
    json = json_base()

    d = {
        'type': 'writers.gdal',
        'resolution': resolution,
        'radius': radius,
        'filename': filename,
        'output_type': output_type,
        'data_type': 'float'
    }

    if bounds is not None:
        d['bounds'] = "([%s,%s],[%s,%s])" % (bounds['minx'], bounds['maxx'], bounds['miny'], bounds['maxy'])

    json['pipeline'].insert(0, d)

    return json


def json_las_base(fout):
    """ Create initial JSON for writing to a LAS file """
    json = json_base()
    json['pipeline'].insert(0, {
        'type': 'writers.las',
        'filename': fout  
    })
    return json


def json_add_decimation_filter(json, step):
    """ Add decimation Filter element and return """
    json['pipeline'].insert(0, {
            'type': 'filters.decimation',
            'step': step
        })
    return json


def json_add_classification_filter(json, classification, equality="equals"):
    """ Add classification Filter element and return """
    limits = 'Classification[{0}:{0}]'.format(classification)
    if equality == 'max':
        limits = 'Classification[:{0}]'.format(classification)

    json['pipeline'].insert(0, {
            'type': 'filters.range',
            'limits': limits
        })
    return json


def is_ply_file(filename):
    _, ext = os.path.splitext(filename)
    return ext.lower() == '.ply'


def json_add_reader(json, filename):
    """ Add Reader Element and return """
    reader_type = 'readers.las' # default
    if is_ply_file(filename):
        reader_type = 'readers.ply'

    json['pipeline'].insert(0, {
            'type': reader_type,
            'filename': os.path.abspath(filename)
        })
    return json


def json_add_readers(json, filenames):
    """ Add merge Filter element and readers to a Writer element and return Filter element """
    for f in filenames:
        json_add_reader(json, f)

    if len(filenames) > 1:
        json['pipeline'].insert(0, {
                'type': 'filters.merge'
            })

    return json


""" Run PDAL commands """

def run_pipeline(json):
    """ Run PDAL Pipeline with provided JSON """

    # write to temp file
    f, jsonfile = tempfile.mkstemp(suffix='.json')
    os.write(f, jsonlib.dumps(json).encode('utf8'))
    os.close(f)

    cmd = [
        'pdal',
        'pipeline',
        '-i %s' % double_quote(jsonfile)
    ]
    system.run(' '.join(cmd))
    os.remove(jsonfile)


def run_pdaltranslate_smrf(fin, fout, scalar, slope, threshold, window):
    """ Run PDAL translate  """
    cmd = [
        'pdal',
        'translate',
        '-i %s' % fin,
        '-o %s' % fout,
        'smrf',
        '--filters.smrf.scalar=%s' % scalar,
        '--filters.smrf.slope=%s' % slope,
        '--filters.smrf.threshold=%s' % threshold,
        '--filters.smrf.window=%s' % window,
        '--writers.las.forward=scale,offset'
    ]

    system.run(' '.join(cmd))


def merge_point_clouds(input_files, output_file):
    if len(input_files) == 0:
        log.ODM_WARNING("Cannot merge point clouds, no point clouds to merge.")
        return

    cmd = [
        'pdal',
        'merge',
        ' '.join(map(double_quote, input_files + [output_file])),
    ]

    system.run(' '.join(cmd))


def translate(input, output):
    cmd = [
        'pdal',
        'translate',
        '-i "%s"' % input,
        '-o "%s"' % output,
    ]

    system.run(' '.join(cmd))