#!/usr/bin/env python3
# Authors: Piero Toffanin, Stephen Mather
# License: AGPLv3

import os
import glob
import sys
sys.path.insert(0, os.path.join("..", "..", os.path.dirname(__file__)))

import argparse
import multiprocessing
from opendm.dem import merge

parser = argparse.ArgumentParser(description='Merge and blend DEMs using OpenDroneMap\'s approach.')
parser.add_argument('input_dems',
                type=str,
                help='Path to input dems (.tif)')

args = parser.parse_args()

if not os.path.exists(args.input_dems):
    print("%s does not exist" % args.input_dems)
    exit(1)

output_dem = os.path.join(args.input_dems, 'merged_blended_dem.tif')
input_dem_path = os.path.join(args.input_dems, '*.tif')
input_dems = glob.glob(input_dem_path)

merge.euclidean_merge_dems(input_dems
                    ,output_dem=output_dem
                )
