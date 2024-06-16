#!/usr/bin/env python3
# Author: Piero Toffanin
# License: AGPLv3

import os
import sys
sys.path.insert(0, os.path.join("..", "..", os.path.dirname(__file__)))

import argparse
import multiprocessing
from opendm.dem import commands

parser = argparse.ArgumentParser(description='Generate DEMs from point clouds using ODM\'s algorithm.')
parser.add_argument('point_cloud',
                type=str,
                help='Path to point cloud file (.las, .laz, .ply)')
parser.add_argument('--type',
                    type=str,
                    choices=("dsm", "dtm"),
                    default="dsm",
                    help="Type of DEM. Default: %(default)s")
parser.add_argument('--resolution',
                type=float,
                default=0.05,
                help='Resolution in m/px. Default: %(default)s')
parser.add_argument('--gapfill-steps',
                    default=3,
                    type=int,
                    help='Number of steps used to fill areas with gaps. Set to 0 to disable gap filling. '
                            'Starting with a radius equal to the output resolution, N different DEMs are generated with '
                            'progressively bigger radius using the inverse distance weighted (IDW) algorithm '
                            'and merged together. Remaining gaps are then merged using nearest neighbor interpolation. '
                            'Default: %(default)s')
args = parser.parse_args()

if not os.path.exists(args.point_cloud):
    print("%s does not exist" % args.point_cloud)
    exit(1)

outdir = os.path.dirname(args.point_cloud)

radius_steps = [args.resolution / 2.0]
for _ in range(args.gapfill_steps - 1):
    radius_steps.append(radius_steps[-1] * 2) # 2 is arbitrary, maybe there's a better value?

commands.create_dem(args.point_cloud,
                    args.type,
                    output_type='idw' if args.type == 'dtm' else 'max',
                    radiuses=list(map(str, radius_steps)),
                    gapfill=args.gapfill_steps > 0,
                    outdir=outdir,
                    resolution=args.resolution,
                    decimation=1,
                    max_workers=multiprocessing.cpu_count()
                )