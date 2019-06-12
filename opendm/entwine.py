import os
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
        'input': "-i " + " ".join(map(quote, input_point_cloud_files)),
        'outputdir': output_path
    }

    entwine_cmd = "entwine build --threads {threads} --tmp {tmpdir} {input} -o {outputdir}".format(**kwargs)

    # Need to split into subsets?
    if num_files > 1:
        subsets = closest_power_of_4(num_files)
        for s in range(1, subsets + 1):
            system.run(entwine_cmd + " --subset %s %s" % (s, subsets))

        # Merge
        # TODO: add --tmp {tmpdir}
        system.run("entwine merge --threads {threads} -o {outputdir}".format(**kwargs))
    else:
        # Single run
        system.run(entwine_cmd)
        
    if os.path.exists(tmpdir):
        shutil.rmtree(tmpdir)