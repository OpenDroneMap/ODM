import os
import shutil
from pipes import quote
from opendm import io
from opendm import log
from opendm import system
from opendm import concurrency

def build(input_point_cloud_files, output_path, max_concurrency=8, rerun=False):
    if len(input_point_cloud_files) == 0:
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

    system.run("entwine build --threads {threads} --tmp {tmpdir} {input} -o {outputdir}".format(**kwargs))

    if os.path.exists(tmpdir):
        shutil.rmtree(tmpdir)