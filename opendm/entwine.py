import os
import json
import shutil
from pipes import quote
from opendm import io
from opendm import log
from opendm import system
from opendm import concurrency
import math

def closest_power_of_4(x):  
    if x <= 0:
        return 1
    n = 1
    while n < x:
        n *= 4
    return n

def get_num_points(scan_file):
    if not os.path.exists(scan_file):
        log.ODM_WARNING("%s does not exist, cannot get number of points." % scan_file)
        return 0

    with open(scan_file, "r") as f:
        scan = json.loads(f.read())
        if not 'points' in scan:
            log.ODM_WARNING("Cannot find number of points in point clouds (points key missing from scan.json). Returning 0")
            return 0

        return scan['points']

def build(input_point_cloud_files, output_path, max_concurrency=8, rerun=False):
    num_files = len(input_point_cloud_files)
    if num_files == 0:
        log.ODM_WARNING("No input point cloud files to process")
        return

    tmpdir = io.related_file_path(output_path, postfix="-tmp")

    if rerun and io.dir_exists(output_path):
        log.ODM_WARNING("Removing previous EPT directory: %s" % output_path)
        shutil.rmtree(output_path)

    kwargs = {
        'threads': max_concurrency,
        'tmpdir': tmpdir,
        'all_inputs': "-i " + " ".join(map(quote, input_point_cloud_files)),
        'outputdir': output_path,
        'scan_file': os.path.join(output_path, "scan.json")
    }

    # Run scan to compute number of points
    system.run('entwine scan --threads {threads} --tmp "{tmpdir}" {all_inputs} -o "{outputdir}"'.format(**kwargs))
    num_points = get_num_points(kwargs['scan_file'])

    # TODO: choose subset
    
    entwine_cmd = "entwine build --threads {threads} --tmp {tmpdir} -i {scan_file} -o {outputdir}".format(**kwargs)

    # Need to split into subsets?
    if num_files > 1:
        subsets = closest_power_of_4(num_files)
        for s in range(1, subsets + 1):
            system.run(entwine_cmd + " --subset %s %s" % (s, subsets))

        # Merge
        system.run("entwine merge --threads {threads} --tmp {tmpdir} -o {outputdir}".format(**kwargs))
    else:
        # Single run
        system.run(entwine_cmd)
        
        
    if os.path.exists(tmpdir):
        shutil.rmtree(tmpdir)